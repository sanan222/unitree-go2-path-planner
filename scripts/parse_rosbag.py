import argparse
import numpy as np

import rclpy
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions, TopicMetadata

def parse_args():
    parser = argparse.ArgumentParser(description='Convert a ROS2 bag from input to output.')
    parser.add_argument('-i', '--input', required=True, help='Input rosbag directory')
    parser.add_argument('-o', '--output', required=True, help='Output rosbag directory')
    parser.add_argument('--input-storage', default='sqlite3', help='Storage format of input bag')
    parser.add_argument('--output-storage', default='sqlite3', help='Storage format of output bag')
    return parser.parse_args()

def main():
    args = parse_args()
    rclpy.init()

    # Setup reader
    reader = rosbag2_py.SequentialReader()
    reader_storage_options = StorageOptions(
        uri=args.input,
        storage_id=args.input_storage
    )
    reader_converter_options = ConverterOptions('', '')
    reader.open(reader_storage_options, reader_converter_options)

    # Get all topics and their metadata
    topics = reader.get_all_topics_and_types()
    topic_type_map = {topic.name: topic.type for topic in topics}

    # Setup writer
    writer = rosbag2_py.SequentialWriter()
    writer_storage_options = StorageOptions(
        uri=args.output,
        storage_id=args.output_storage
    )
    writer_converter_options = ConverterOptions('', '')
    writer.open(writer_storage_options, writer_converter_options)

    # Create topics in writer
    for topic in topics:
        writer.create_topic(TopicMetadata(
            name=topic.name,
            type=topic.type,
            serialization_format=topic.serialization_format
        ))

    # Process messages
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        # Get ROS 2 message type
        msg_type = get_message(topic_type_map[topic])
        deserialized_msg = deserialize_message(data, msg_type)
        
        try:
            deserialized_msg._foot_force.fill(0)
        except:
            pass

        # Reserialize properly
        serialized_data = serialize_message(deserialized_msg)
        writer.write(topic, serialized_data, timestamp)

    rclpy.shutdown()

if __name__ == '__main__':
    main()