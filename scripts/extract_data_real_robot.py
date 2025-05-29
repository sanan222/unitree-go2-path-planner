import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import csv
from unitree_go.msg import LowState, SportModeState  # Correct ROS 2 message type for real robot

def extract_required_data(bag_path, effort_csv, imu_csv, contacts_csv, contacts_force_csv, timestamp_csv):
    # Initialize ROS 2
    rclpy.init()

    # Set up the bag reader
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Topics to extract
    topics = {
        "/lowstate": LowState,  # Single topic for all required data
        "/sportmodestate": SportModeState  # Single topic for all required data
    }

    # Extract topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # Ensure topics exist in the bag
    for topic in topics.keys():
        if topic not in type_map:
            print(f"Warning: Topic {topic} not found in the bag.")
            return

    # Open CSV files
    with open(effort_csv, mode='w', newline='') as effort_file, \
         open(imu_csv, mode='w', newline='') as imu_file, \
         open(contacts_csv, mode='w', newline='') as contacts_file, \
         open(contacts_force_csv, mode='w', newline='') as contacts_force_file, \
         open(timestamp_csv, mode='w', newline='') as timestamp_file:

        # CSV Writers
        effort_writer = csv.writer(effort_file)
        imu_writer = csv.writer(imu_file)
        contacts_writer = csv.writer(contacts_file)
        contacts_force_writer = csv.writer(contacts_force_file)
        timestamp_writer = csv.writer(timestamp_file)

        # Write Headers
        effort_writer.writerow(["timestamp"] + [f"joint_{i+1}" for i in range(12)])  # 12 joints
        imu_writer.writerow(["timestamp", "orientation_x", "orientation_y", "orientation_z", "orientation_w",
                             "angular_vel_x", "angular_vel_y", "angular_vel_z",
                             "linear_acc_x", "linear_acc_y", "linear_acc_z"])
        contacts_writer.writerow(["timestamp", "contact_1", "contact_2", "contact_3", "contact_4"])
        contacts_force_writer.writerow(["timestamp", "contact_1", "contact_2", "contact_3", "contact_4"])
        timestamp_writer.writerow(["timestamp"])  # Timestamp CSV

        # Data Storage
        effort_data, imu_data, contacts_data, contacts_force_data, timestamps = [], [], [], [], []

        while reader.has_next():
            (topic_name, data, timestamp) = reader.read_next()
            time_in_sec = timestamp / 1e9  # Convert to seconds

            # Extract Data from /lowstate
            if topic_name == "/lowstate":
                msg = deserialize_message(data, LowState)

                # Extract Joint Effort Data (12 motors)
                joint_efforts = [motor.tau_est for motor in msg.motor_state[:12]]  # 12 joints
                effort_data.append([time_in_sec] + joint_efforts)

                # Extract IMU Data
                imu_data.append([
                    time_in_sec,
                    msg.imu_state.quaternion[0], msg.imu_state.quaternion[1],
                    msg.imu_state.quaternion[2], msg.imu_state.quaternion[3],
                    msg.imu_state.gyroscope[0], msg.imu_state.gyroscope[1], msg.imu_state.gyroscope[2],
                    msg.imu_state.accelerometer[0], msg.imu_state.accelerometer[1], msg.imu_state.accelerometer[2]
                ])


                # Store timestamps
                timestamps.append([time_in_sec])
            # Extract Foot Contact Data from /sportmodestate
            if topic_name == "/sportmodestate":
                msg = deserialize_message(data, SportModeState)

                # Extract foot forces (should be a list of 4 values)
                foot_forces = msg.foot_force

                # Set contact threshold (adjust if needed)
                contact_threshold = 20  # Adjust this based on your robot's data
                
                # Convert foot forces to binary contact states (1 = contact, 0 = no contact)
           
                contact_states = [1 if force > contact_threshold else 0 for force in foot_forces]
                contact_force = [force for force in foot_forces]

                # Append timestamp + correct binary contact states
                contacts_data.append([time_in_sec] + contact_states)
                contacts_force_data.append([time_in_sec] + contact_force)


        # Save Effort Data
        for row in effort_data:
            effort_writer.writerow(row)

        # Save IMU Data
        for row in imu_data:
            imu_writer.writerow(row)

        # Save Foot Contact Data
        for row in contacts_data:
            contacts_writer.writerow(row)

        # Save Timestamps
        for row in contacts_force_data:
            contacts_force_writer.writerow(row)

        # Save Timestamps
        for row in timestamps:
            timestamp_writer.writerow(row)
        

    print(f"Effort data saved to {effort_csv}")
    print(f"IMU data saved to {imu_csv}")
    print(f"Contact data saved to {contacts_csv}")
    print(f"Contact data saved to {contacts_force_csv}")
    print(f"Timestamps saved to {timestamp_csv}")

    rclpy.shutdown()

if __name__ == '__main__':
    bag_file = "/usr/app/comp0244_ws/data_comp0244/rosbag2_2025_02_22-02_23_45/rosbag2_test_output/rosbag2_test_output_0.db3"  # Replace with actual bag file path
    effort_csv = "output_effort.csv"  # Joint Efforts
    imu_csv = "output_imu.csv"  # IMU Data
    contacts_csv = "output_contacts.csv"  # Foot Contact Data
    contacts_force_csv = "output_contacts_force.csv"  # Foot Contact Data
    timestamp_csv = "timestamps.csv"  # Timestamps

    extract_required_data(bag_file, effort_csv, imu_csv, contacts_csv, contacts_force_csv, timestamp_csv)
