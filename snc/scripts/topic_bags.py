import subprocess
import signal
import datetime
import os

# --- Configuration ---
try:
    import snc.constants as c
except ImportError:
    try:
        import constants as c
    except ImportError:
        print("Warning: constants file not found. Using manual fallbacks.")
        class c:
            START_CHALLENGE_TOPIC = '/snc_start'
            OBJECTS_TOPIC = "/objectsStamped"
            PATH_RETURN_TOPIC = '/path_return'
            PATH_EXPLORE_TOPIC = '/path_explore'
            HAZARD_SIGNAL_TOPIC = '/snc/hazard_signal'
            ROBOT_POSE_TOPIC = '/snc/robot_pose'
            GO_HOME_TOPIC = '/go_home'
            TRIGGER_START_TOPIC = '/trigger_start'
            TRIGGER_TELEOP_TOPIC = '/trigger_teleop'
            TRIGGER_HOME_TOPIC = '/trigger_home'
            SNC_STATUS_TOPIC = '/snc_status'
            STARTUP_SYNC_TOPIC = '/snc_startup_sync'
            COVERAGE_TOPIC = '/covered_cells_marker'
            HAZARD_MARKER_TOPIC = '/hazards'

def record_bag():
    
    topics_to_record = [
        c.START_CHALLENGE_TOPIC,
        c.OBJECTS_TOPIC,
        c.PATH_RETURN_TOPIC,
        c.PATH_EXPLORE_TOPIC,
        c.HAZARD_SIGNAL_TOPIC,
        c.ROBOT_POSE_TOPIC,
        c.GO_HOME_TOPIC,
        c.TRIGGER_START_TOPIC,
        c.TRIGGER_TELEOP_TOPIC,
        c.TRIGGER_HOME_TOPIC,
        c.SNC_STATUS_TOPIC,
        c.STARTUP_SYNC_TOPIC,
        c.COVERAGE_TOPIC,
        c.HAZARD_MARKER_TOPIC
    ]

    # Clean and deduplicate topic list
    topics_to_record = list(set(filter(None, topics_to_record)))

    # Define the output directory and create it if it doesn't exist
    output_dir = os.path.expanduser("~/Documents/par-as2-runs")
    os.makedirs(output_dir, exist_ok=True)
    
    # Create a unique bag name using a timestamp
    # Format: snc_run_20260423_143005
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = os.path.join(output_dir, f"snc_run_{timestamp}")
    
    # Construct the command
    # Using MCAP for better compatibility with Foxglove/Studio
    cmd = [
        "ros2", "bag", "record", 
        "-o", bag_name, 
        "--storage", "mcap"
    ] + topics_to_record

    print("-" * 40)
    print(f"RECORDER STARTED at {timestamp}")
    print(f"Output: {bag_name}")
    print(f"Recording {len(topics_to_record)} topics...")
    print("-" * 40)
    print("Press Ctrl+C to stop recording.")
    
    try:
        # Start the recording as a subprocess
        process = subprocess.Popen(cmd)
        
        # Keep the script alive while the process runs
        process.wait()
        
    except KeyboardInterrupt:
        # Gracefully shut down the bag recording
        print("\n" + "-" * 40)
        print("Finishing recording... please wait.")
        process.send_signal(signal.SIGINT)
        process.wait()
        print(f"Bag successfully saved to: {os.path.abspath(bag_name)}")
        print("-" * 40)

if __name__ == "__main__":
    record_bag()