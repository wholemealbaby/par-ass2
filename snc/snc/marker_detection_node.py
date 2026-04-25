#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty, String
from snc.constants import (
    START_CHALLENGE_INTERFACE,
    START_CHALLENGE_TOPIC,
    START_CHALLENGE_BUFFER_SIZE,
    OBJECTS_TOPIC,
    OBJECTS_BUFFER_SIZE,
    OBJECTS_INTERFACE,
    GO_HOME_TOPIC,
    GO_HOME_INTERFACE,
    GO_HOME_BUFFER_SIZE,
    TRIGGER_START_TOPIC,
    TRIGGER_START_INTERFACE,
    TRIGGER_START_BUFFER_SIZE,
    TRIGGER_HOME_TOPIC,
    TRIGGER_HOME_INTERFACE,
    TRIGGER_HOME_BUFFER_SIZE,
    TRIGGER_QOS,
    SNC_STATUS_TOPIC,
    SNC_STATUS_INTERFACE,
    SNC_STATUS_BUFFER_SIZE,
)
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
from snc.hazard import HazardManager
from snc.object import ObjectHandler

# Marker visualisation settings
HAZARD_MARKER_TOPIC = '/hazards'
HAZARD_MARKER_BUFFER_SIZE = 10

MARKER_SCALE = 0.3              # sphere diameter in metres
MARKER_COLOR = (1.0, 0.0, 0.0, 0.8)   # RGBA — red, 80 % opaque
MARKER_LIFETIME_SEC = 0         # 0 = marker persists until explicitly deleted

LASER_TOPIC = '/scan'
LASER_BUFFER_SIZE = 10
# Number of unique hazards required before go_home is triggered
HAZARDS_REQUIRED = 5

class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('marker_detection_node')
        self.get_logger().info('Marker detection node launched')

        # Ensure the start signal is only published once
        self.start_marker_detected = False
        # Set of unique hazard names confirmed (pose successfully transformed)
        self.confirmed_hazards: set = set()
        # go_home is triggered once this reaches HAZARDS_REQUIRED
        self.go_home_triggered = False
        # Latest laser scan — updated by laser_callback
        self.latest_scan = None

        # TF listener — used to transform hazard poses into the map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Decodes raw find_object_2d data into DetectedObject instances
        self.object_handler = ObjectHandler()

        # Tracks unique hazards and transforms their poses to the map frame
        self.hazard_manager = HazardManager()

        # Subscriber: incoming object detections from find_object_2d
        self.sub_objects = self.create_subscription(
            OBJECTS_INTERFACE,
            OBJECTS_TOPIC,
            self.objects_callback,
            OBJECTS_BUFFER_SIZE,
        )

        # Laser scan — secondary sensor for hazard depth
        self.sub_laser = self.create_subscription(
            LaserScan,
            LASER_TOPIC,
            self.laser_callback,
            LASER_BUFFER_SIZE,
        )

        # Contingency: assessor manually triggers start if visual detection fails
        self.sub_trigger_start = self.create_subscription(
            TRIGGER_START_INTERFACE,
            TRIGGER_START_TOPIC,
            self.trigger_start_callback,
            TRIGGER_START_BUFFER_SIZE,
        )
 
        # Contingency: assessor manually triggers return home
        self.sub_trigger_home = self.create_subscription(
            TRIGGER_HOME_INTERFACE,
            TRIGGER_HOME_TOPIC,
            self.trigger_home_callback,
            TRIGGER_QOS,
        )

        self.pub_trigger_start = self.create_publisher(
            TRIGGER_START_INTERFACE,
            TRIGGER_START_TOPIC,
            TRIGGER_START_BUFFER_SIZE,
        )
 
        self.pub_go_home = self.create_publisher(
            GO_HOME_INTERFACE,
            GO_HOME_TOPIC,
            TRIGGER_QOS,
        )
 
        self.pub_hazard_marker = self.create_publisher(
            Marker,
            HAZARD_MARKER_TOPIC,
            HAZARD_MARKER_BUFFER_SIZE,
        )
 
        # Status topic — required by spec Section 3.5
        self.pub_status = self.create_publisher(
            SNC_STATUS_INTERFACE,
            SNC_STATUS_TOPIC,
            SNC_STATUS_BUFFER_SIZE,
        )
 
        self._publish_status("Node 2 ready — waiting for start marker")

    def laser_callback(self, msg: LaserScan) -> None:
        """Cache the latest laser scan for use in depth estimation."""
        self.latest_scan = msg

    def objects_callback(self, msg) -> None:
        """Handle an incoming ObjectsStamped message from find_object_2d."""
        self.get_logger().debug('Received objects message')
 
        # Decode detections
        self.object_handler.add_objects_from_message(msg)
 
        # Apply laser depth to every detected object (secondary sensor)
        self.object_handler.update_depths_from_laser(self.latest_scan)
 
        # Trigger start once if the start marker is detected visually
        if not self.start_marker_detected and self.object_handler.start_marker_detected():
            self.trigger_start()
 
        # Update hazard list and publish markers
        self.hazard_manager.update_from_objects(self.object_handler.objects)
        self._publish_hazard_markers()
 
        # Check if enough unique hazards have been confirmed to go home
        if not self.go_home_triggered:
            self._check_go_home()

    def trigger_start_callback(self, msg) -> None:
        """Contingency: assessor manually triggers start via /trigger_start."""
        if not self.start_marker_detected:
            self.get_logger().warn('Manual start trigger received via /trigger_start')
            self.trigger_start()
 
    def trigger_home_callback(self, msg) -> None:
        """Contingency: assessor manually triggers go_home via /trigger_home."""
        if not self.go_home_triggered:
            self.get_logger().warn(
                f'Manual go_home trigger received via /trigger_home '
                f'({len(self.confirmed_hazards)} hazards confirmed so far)'
            )
            self.trigger_go_home()

    def trigger_start(self) -> None:
        """Set the start flag and publish an Empty message to /trigger_start."""
        self.start_marker_detected = True
        self.pub_trigger_start.publish(Empty())
        self.get_logger().info('Start marker detected! Published to /trigger_start.')
        
    def trigger_go_home(self) -> None:
        """Publish to /go_home and set the flag."""
        self.go_home_triggered = True
        self.pub_go_home.publish(Empty())
        msg = (
            f'All {HAZARDS_REQUIRED} hazards found — returning home. '
            f'Hazards: {sorted(self.confirmed_hazards)}'
        )
        self.get_logger().info(msg)
        self._publish_status(f"Returning home — {len(self.confirmed_hazards)} hazards confirmed")
 
    def _check_go_home(self) -> None:
        """Trigger go_home automatically once HAZARDS_REQUIRED unique hazards are confirmed."""
        if len(self.confirmed_hazards) >= HAZARDS_REQUIRED:
            self.trigger_go_home()

    def _publish_hazard_markers(self) -> None:
        """
        For every hazard in the current frame, transform its pose to the map
        frame and publish a visualisation Marker to /hazards.
        """
        for hazard in self.hazard_manager.list:
            # Skip the start marker — must NOT appear on /hazards (spec 3.4)
            if hazard.name == 'Start':
                continue
 
            map_pose = hazard.get_map_pose(self.tf_buffer, target_frame='map')
            if map_pose is None:
                self.get_logger().warn(
                    f'No map pose for {hazard.name} '
                    f'(depth_source={hazard.depth_source}) — skipping'
                )
                continue
            self.get_logger().debug(f'Found hazard: {hazard.name}')
            # Register as confirmed unique hazard on first successful pose
            if hazard.name not in self.confirmed_hazards:
                self.confirmed_hazards.add(hazard.name)
                self.get_logger().info(
                    f'New hazard confirmed: {hazard.name} '
                    f'({len(self.confirmed_hazards)}/{HAZARDS_REQUIRED}) '
                    f'depth_source={hazard.depth_source} '
                    f'depth={hazard.depth:.2f}m'
                )
                self._publish_status(
                    f"Hazard found: {hazard.name} "
                    f"({len(self.confirmed_hazards)}/{HAZARDS_REQUIRED})"
                )
 
            marker = self._build_marker(hazard.id, map_pose)
            self.pub_hazard_marker.publish(marker)
            self.get_logger().info(
                f'Published marker for {hazard.name} (ID {hazard.id}) '
                f'at ({map_pose.pose.position.x:.2f}, {map_pose.pose.position.y:.2f})'
            )

    def _build_marker(self, hazard_id: int, map_pose) -> Marker:
        """Construct a SPHERE Marker at the given map-frame pose."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'hazards'
        marker.id = hazard_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = map_pose.pose
        marker.scale.x = MARKER_SCALE
        marker.scale.y = MARKER_SCALE
        marker.scale.z = MARKER_SCALE
        r, g, b, a = MARKER_COLOR
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.lifetime = Duration(sec=MARKER_LIFETIME_SEC)
        return marker
    
    def _publish_status(self, message: str) -> None:
        """Publish a status string to /snc_status (required by spec 3.5)."""
        msg = String()
        msg.data = f"[Node2] {message}"
        self.pub_status.publish(msg)

def main():
    rclpy.init()
    node = MarkerDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()