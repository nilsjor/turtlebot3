# tf2_transform_node

The `tf2_transform_node` is a ROS 2 node that subscribes to a specified topic, transforms the incoming messages to a target frame using tf2, and publishes the transformed messages to an output topic. This node supports various message types and can be extended to handle additional message types.

## Supported Message Types

The node currently supports the following message types:
- `geometry_msgs/msg/PointStamped`
- `geometry_msgs/msg/PoseStamped`
- `geometry_msgs/msg/Vector3Stamped`
- `sensor_msgs/msg/PointCloud2`

## Usage

### Running the Node

To run the `tf2_transform_node`, use the following command:

```sh
ros2 run turtlebot3_support tf2_transform_node <input_topic> <message_type> <target_frame> [output_topic]
```

### Parameters

- `input_topic`: The topic to subscribe to for incoming messages.
- `message_type`: The type of the incoming messages. Supported types are:
  - `geometry_msgs/msg/PointStamped`
  - `geometry_msgs/msg/PoseStamped`
  - `geometry_msgs/msg/Vector3Stamped`
  - `sensor_msgs/msg/PointCloud2`
- `target_frame`: The target frame to transform the messages to.
- `output_topic`: (Optional) The topic to publish the transformed messages to. If not provided, the output topic will be `<input_topic>_<target_frame>`.

## Extending the Node

To extend the node to handle additional message types, follow these steps:

1. Include the necessary header files for the new message types at the top of the tf2_transform_node.cpp file.

```cpp
#include <new_message_type.hpp>
```

2. Add a new condition in the `create_handler` function to handle the new message type.

```cpp
else if (type == "new_message_type")
{
    return std::make_shared<GenericTransformHandler<new_message_type>>();
}
```

3. Rebuild your ROS 2 workspace to apply the changes.

```sh
colcon build
```

Now, the node can handle `sensor_msgs/msg/LaserScan` messages.

## License

This project is licensed under the MIT License. See the LICENSE file for details.
