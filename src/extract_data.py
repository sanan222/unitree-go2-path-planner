import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import csv
from control_msgs.msg import JointTrajectoryControllerState  # Joint Efforts
from sensor_msgs.msg import Imu  # IMU Data
from champ_msgs.msg import ContactsStamped  # Foot Contacts

def extract_data_from_bag(bag_path, effort_csv, imu_csv, contacts_csv, timestamp_csv):
    # Initialize ROS 2
    rclpy.init()

    # Set up the bag reader
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Topics
    topics = {
        "/joint_group_effort_controller/controller_state": JointTrajectoryControllerState,
        "/imu/data": Imu,
        "/foot_contacts": ContactsStamped
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
         open(timestamp_csv, mode='w', newline='') as timestamp_file:

        # CSV Writers
        effort_writer = csv.writer(effort_file)
        imu_writer = csv.writer(imu_file)
        contacts_writer = csv.writer(contacts_file)
        timestamp_writer = csv.writer(timestamp_file)

        # Write Headers
        effort_writer.writerow(["timestamp", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
                                "joint_7", "joint_8", "joint_9", "joint_10", "joint_11", "joint_12"])
        imu_writer.writerow(["timestamp", "orientation_x", "orientation_y", "orientation_z", "orientation_w",
                             "angular_vel_x", "angular_vel_y", "angular_vel_z",
                             "linear_acc_x", "linear_acc_y", "linear_acc_z"])
        contacts_writer.writerow(["timestamp", "contact_1", "contact_2", "contact_3", "contact_4"])
        timestamp_writer.writerow(["timestamp"])  # Timestamp CSV

        # Data Storage
        effort_data, imu_data, contacts_data, timestamps = [], [], [], []

        while reader.has_next():
            (topic_name, data, timestamp) = reader.read_next()
            time_in_sec = timestamp / 1e9  # Convert to seconds

            # Extract Joint Effort Data
            if topic_name == "/joint_group_effort_controller/controller_state":
                msg = deserialize_message(data, JointTrajectoryControllerState)
                if len(msg.output.effort) == 12:
                    effort_data.append([time_in_sec] + list(msg.output.effort))

            # Extract IMU Data
            elif topic_name == "/imu/data":
                msg = deserialize_message(data, Imu)
                imu_data.append([
                    time_in_sec,
                    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                ])

            # Extract Foot Contact Data (Saving as True/False)
            elif topic_name == "/foot_contacts":
                msg = deserialize_message(data, ContactsStamped)
                contact_values = [bool(c) for c in msg.contacts]  # Convert to True/False
                contacts_data.append([time_in_sec] + contact_values)

            # Store timestamps
            timestamps.append([time_in_sec])

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
        for row in timestamps:
            timestamp_writer.writerow(row)

    print(f"Effort data saved to {effort_csv}")
    print(f"IMU data saved to {imu_csv}")
    print(f"Contact data saved to {contacts_csv}")
    print(f"Timestamps saved to {timestamp_csv}")

    rclpy.shutdown()

if __name__ == '__main__':
    bag_file = 'rosbag2_2025_01_30-20_23_47/rosbag2_2025_01_30-20_23_47_0.db3'  # Replace with your bag file path
    effort_csv = 'output_effort.csv'  # Joint Efforts
    imu_csv = 'output_imu.csv'  # IMU Data
    contacts_csv = 'output_contacts.csv'  # Foot Contact Data
    timestamp_csv = 'timestamps.csv'  # Timestamps

    extract_data_from_bag(bag_file, effort_csv, imu_csv, contacts_csv, timestamp_csv)
