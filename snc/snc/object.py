import math

import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from snc.constants import OBJECT_MAP

# Camera calibration values for OAK-D camera on ROSbot PRO 3.
# To get exact values:  ros2 topic echo /oak/rgb/camera_info --once
FOCAL_LENGTH_X = 554.0       # fx  (pixels)
FOCAL_LENGTH_Y = 554.0       # fy  (pixels)
PRINCIPAL_POINT_X = 320.0    # cx  (pixels) — usually image_width  / 2
PRINCIPAL_POINT_Y = 240.0    # cy  (pixels) — usually image_height / 2
IMAGE_WIDTH = 640
LASER_WINDOW_DEG = 5.0

class DetectedObject:
    def __init__(self, data_slice: list, header: Header):
        """
        Abstract representation of an object detected via find_object_2d.

        :param data_slice: 12 floats (ID, Width, Height, and 3x3 homography matrix)
        :param header: The header from the incoming ROS message
        """
        self.raw_data = data_slice
        self.header = header
        self.object_id = int(data_slice[0])
        self.name = OBJECT_MAP.get(self.object_id, "Unknown")
        self.ref_width = data_slice[1]
        self.ref_height = data_slice[2]

        # Reconstruct 3x3 Homography Matrix from flat array
        # Layout: [h11, h12, h13, h21, h22, h23, h31, h32, h33]
        self.h_matrix = np.array(data_slice[3:12]).reshape(3, 3)

        # Project the centre of the reference image through the homography
        # to get the pixel location in the camera frame
        ref_center = np.array([self.ref_width / 2.0, self.ref_height / 2.0, 1.0])
        cam_center = np.dot(self.h_matrix, ref_center)

        # Normalise by the homogeneous coordinate w
        self.pixel_x = cam_center[0] / cam_center[2]
        self.pixel_y = cam_center[1] / cam_center[2]

        # Extract yaw — rotation of the object around the camera optical Z-axis
        self.yaw = math.atan2(self.h_matrix[1, 0], self.h_matrix[0, 0])

        # Depth: initialised from homography scale as a fallback only.
        # Call update_depth_from_laser() to replace with a real sensor value.
        scale = math.sqrt(self.h_matrix[0, 0] ** 2 + self.h_matrix[1, 0] ** 2)
        self._homography_depth = FOCAL_LENGTH_X / scale if scale > 1e-6 else 0.5
        self.depth = self._homography_depth
        self.depth_source = "homography"  # set to "laser" after scan update
 
        # Camera-frame PoseStamped — rebuilt after update_depth_from_laser()
        self.pose_stamped = self._create_camera_pose()

    def update_depth_from_laser(self, scan_msg) -> None:
        """
        Estimate object depth using the laser scan as the secondary sensor.
 
        The horizontal angle to the object is derived from pixel_x and the
        camera FOV.  We sample the scan at that angle (±LASER_WINDOW_DEG)
        and take the minimum valid range as the depth.
 
        After calling this, pose_stamped is rebuilt with the corrected depth.
 
        :param scan_msg: sensor_msgs/msg/LaserScan (latest scan)
        """
        if scan_msg is None:
            return
 
        # Horizontal FOV of the camera (radians)
        hfov = 2.0 * math.atan2(IMAGE_WIDTH / 2.0, FOCAL_LENGTH_X)
 
        # Angle from the optical axis to the object centre (radians)
        # pixel_x == cx  →  0 rad  (straight ahead)
        object_angle = ((self.pixel_x - PRINCIPAL_POINT_X) / IMAGE_WIDTH) * hfov
 
        # Map camera angle into laser frame.
        # On ROSbot PRO 3 the laser is forward-facing and aligned with the
        # camera so a direct negation is used (right in camera = negative laser
        # angle).  Adjust if your physical setup differs.
        laser_angle = -object_angle
 
        window = math.radians(LASER_WINDOW_DEG)
        min_range = float('inf')
 
        for i, r in enumerate(scan_msg.ranges):
            beam_angle = scan_msg.angle_min + i * scan_msg.angle_increment
            if abs(beam_angle - laser_angle) <= window:
                if scan_msg.range_min <= r <= scan_msg.range_max and not math.isnan(r):
                    min_range = min(min_range, r)
 
        if math.isfinite(min_range):
            self.depth = min_range
            self.depth_source = "laser"
            self.pose_stamped = self._create_camera_pose()

    def _create_camera_pose(self) -> PoseStamped:
        """Create a PoseStamped in the camera optical frame using pinhole projection."""
        ps = PoseStamped()
        ps.header = self.header

        # Back-project pixel coords + depth into 3-D camera space
        ps.pose.position.x = (self.pixel_x - PRINCIPAL_POINT_X) * self.depth / FOCAL_LENGTH_X
        ps.pose.position.y = (self.pixel_y - PRINCIPAL_POINT_Y) * self.depth / FOCAL_LENGTH_Y
        ps.pose.position.z = self.depth

        # Encode yaw as a quaternion (rotation around optical Z-axis)
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = math.sin(self.yaw / 2.0)
        ps.pose.orientation.w = math.cos(self.yaw / 2.0)

        return ps

    # def get_map_pose(self, tf_buffer, target_frame: str = "map") -> PoseStamped:
    #     """
    #     Transform the camera-frame pose into target_frame (default: map).
 
    #     Waits up to 0.5 s for the TF transform before giving up.
 
    #     :param tf_buffer: tf2_ros.Buffer
    #     :param target_frame: destination TF frame name
    #     :return: PoseStamped in target_frame, or None on failure
    #     """
    #     source_frame = self.pose_stamped.header.frame_id
    #     if not source_frame:
    #         print(
    #             "[DetectedObject] pose_stamped.header.frame_id is empty. "
    #             "Check: ros2 topic echo /oak/rgb/image_raw/compressed --once | grep frame_id"
    #         )
    #         return None
 
    #     try:
    #         tf_buffer.can_transform(
    #             target_frame,
    #             source_frame,
    #             self.pose_stamped.header.stamp,
    #             timeout=Duration(seconds=0.5),
    #         )
    #         return tf_buffer.transform(self.pose_stamped, target_frame)
    #     except Exception as e:
    #         print(
    #             f"[DetectedObject] TF transform failed: "
    #             f"{source_frame} -> {target_frame}: {e}"
    #         )
    #         return None

    def get_map_pose(self, tf_buffer, target_frame: str = "map") -> PoseStamped:
        source_frame = self.pose_stamped.header.frame_id
        if not source_frame:
            print(
                "[DetectedObject] pose_stamped.header.frame_id is empty. "
                "Check: ros2 topic echo /oak/rgb/image_raw/compressed --once | grep frame_id"
            )
            return None
    
        try:
            # Look up the transform from camera frame to map frame
            transform = tf_buffer.lookup_transform(
                target_frame,                           # target  (map)
                source_frame,                           # source  (camera optical frame)
                self.pose_stamped.header.stamp,         # time of the detection
                timeout=Duration(seconds=0.5),
            )
    
            # Extract translation and rotation from the transform
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            rx = transform.transform.rotation.x
            ry = transform.transform.rotation.y
            rz = transform.transform.rotation.z
            rw = transform.transform.rotation.w
    
            # Manually apply the transform to the camera-frame position
            # using quaternion rotation: p_map = q * p_cam * q_inv + t
            cx = self.pose_stamped.pose.position.x
            cy = self.pose_stamped.pose.position.y
            cz = self.pose_stamped.pose.position.z
    
            # Rotate the position vector using the quaternion
            # Formula: v' = q * v * q_conjugate
            # Expanded to avoid external libraries:
            px = (1 - 2*(ry**2 + rz**2)) * cx + 2*(rx*ry - rz*rw) * cy + 2*(rx*rz + ry*rw) * cz
            py = 2*(rx*ry + rz*rw) * cx + (1 - 2*(rx**2 + rz**2)) * cy + 2*(ry*rz - rx*rw) * cz
            pz = 2*(rx*rz - ry*rw) * cx + 2*(ry*rz + rx*rw) * cy + (1 - 2*(rx**2 + ry**2)) * cz
    
            # Add translation
            map_x = px + tx
            map_y = py + ty
            map_z = pz + tz
    
            # Build the result PoseStamped in the map frame
            result = PoseStamped()
            result.header.frame_id = target_frame
            result.header.stamp = self.pose_stamped.header.stamp
            result.pose.position.x = map_x
            result.pose.position.y = map_y
            result.pose.position.z = map_z
    
            # Combine quaternions: q_map = q_transform * q_camera
            ox = self.pose_stamped.pose.orientation.x
            oy = self.pose_stamped.pose.orientation.y
            oz = self.pose_stamped.pose.orientation.z
            ow = self.pose_stamped.pose.orientation.w
    
            result.pose.orientation.x = rw*ox + rx*ow + ry*oz - rz*oy
            result.pose.orientation.y = rw*oy - rx*oz + ry*ow + rz*ox
            result.pose.orientation.z = rw*oz + rx*oy - ry*ox + rz*ow
            result.pose.orientation.w = rw*ow - rx*ox - ry*oy - rz*oz
    
            return result
    
        except Exception as e:
            print(
                f"[DetectedObject] TF lookup_transform failed: "
                f"{source_frame} -> {target_frame}: {e}"
            )
            return None

class ObjectHandler:
    def __init__(self):
        self.objects = []
    
    def add_object(self, data_slice: list, header) -> None:
        """
        Creates an Object instance and adds it to the internal list.
        """
        if len(data_slice) == 12:
            new_obj = DetectedObject(data_slice, header)
            self.objects.append(new_obj)
        else:
            # This handles cases where the array might be malformed
            print(f"Warning: Received data_slice of length {len(data_slice)}, expected 12.")

    def _validate_header(self, header) -> None:
        """Ensures the header contains necessary information
        including timestamp and frame_id.
        
        :param header: The header from the incoming ROS message"""

        if not isinstance(header, Header):
            raise TypeError("Expected header to be of type Header")
        if not header.stamp:
            raise ValueError("Header is missing timestamp")
        if not header.frame_id:
            raise ValueError("Header is missing frame_id")

    def add_objects_from_message(self, msg) -> None:
        """
        Parse an ObjectsStamped message and populate self.objects.
        Clears previous detections — only current-frame objects are kept.
        """
        self.objects = []
        header = msg.header
        data = msg.objects.data  # flat Float32MultiArray

        # Each detected object occupies exactly 12 floats
        for i in range(0, len(data), 12):
            data_slice = data[i: i + 12]
            if len(data_slice) == 12:
                self.objects.append(DetectedObject(data_slice, header))
            else:
                print(
                    f"[ObjectHandler] Skipping malformed slice at index {i}: "
                    f"expected 12 floats, got {len(data_slice)}"
                )
    
    def update_depths_from_laser(self, scan_msg) -> None:
        """Apply the latest laser scan depth to all current objects."""
        for obj in self.objects:
            obj.update_depth_from_laser(scan_msg)

    def start_marker_detected(self) -> bool:
        """Return True if the 'Start' marker is among the currently detected objects."""
        return any(obj.name == "Start" for obj in self.objects)