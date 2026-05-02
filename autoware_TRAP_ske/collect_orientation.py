import subprocess
import yaml

def parse_message(yaml_text):
    try:
        # Parse the YAML output into a Python dictionary
        data = yaml.safe_load(yaml_text)
    except Exception as e:
        print("Error parsing YAML:", e)
        return

    if not data or "objects" not in data:
        return

    for obj in data["objects"]:
        try:
            # Extract position and orientation from the message
            pose = obj["kinematics"]["initial_pose_with_covariance"]["pose"]
            position = pose["position"]
            orientation = pose["orientation"]

            print("position:")
            print(f"        x: {position['x']}")
            print(f"        y: {position['y']}")
            print(f"        z: {position['z']}")
            print("orientation:")
            print(f"        x: {orientation['x']}")
            print(f"        y: {orientation['y']}")
            print(f"        z: {orientation['z']}")
            print(f"        w: {orientation['w']}")
            print()
            return orientation
        except KeyError as e:
            print("Missing field in object:", e)

def main():
    # Launch 'ros2 topic echo' as a subprocess
    process = subprocess.Popen(
        ["ros2", "topic", "echo", "/perception/object_recognition/objects"],
        stdout=subprocess.PIPE,
        text=True
    )

    message_buffer = ""
    last_message = None
    try:
        for line in process.stdout:
            # Messages are separated by '---'
            if line.strip() == "---":
                # Parse the buffered message when a separator is encountered
                if message_buffer.strip():
                    orientation = parse_message(message_buffer)
                    last_message = orientation
                message_buffer = ""  # Reset buffer for the next message
            else:
                message_buffer += line
    except KeyboardInterrupt:
        with open("output_orientation.txt", "a") as f:
                        f.write(str(last_message))
                        f.write("\n")
        print("Exiting...")
    finally:
        process.terminate()
        process.wait()

if __name__ == "__main__":
    main()
