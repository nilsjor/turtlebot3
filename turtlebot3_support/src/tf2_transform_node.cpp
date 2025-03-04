#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

// Base class for handling different message types
class TransformHandler
{
public:
    // Pure virtual function to subscribe to a topic and set up the transformation
    virtual void subscribe(rclcpp::Node::SharedPtr node, const std::string& input_topic, const std::string& output_topic,
                           const std::string& target_frame, const tf2_ros::Buffer& tf_buffer) = 0;
    virtual ~TransformHandler() = default;
};

// Template class for handling different message types
template<typename T>
class GenericTransformHandler : public TransformHandler
{
public:
    void subscribe(rclcpp::Node::SharedPtr node, const std::string& input_topic, const std::string& output_topic,
                   const std::string& target_frame, const tf2_ros::Buffer& tf_buffer) override
    {
        // Subscribe to the input topic
        subscription_ = node->create_subscription<T>(
            input_topic, 10, 
            [this, target_frame, &tf_buffer](const typename T::SharedPtr msg) {
                this->callback(msg, target_frame, tf_buffer);
            });

        // Create a publisher for the transformed messages
        publisher_ = node->create_publisher<T>(output_topic, 10);
        RCLCPP_INFO(node->get_logger(), "Transforming messages from '%s' to frame '%s' and publishing on '%s'.",
                    input_topic.c_str(), target_frame.c_str(), output_topic.c_str());
    }

private:
    // Callback function to handle incoming messages and perform the transformation
    void callback(const typename T::SharedPtr msg, 
                  const std::string& target_frame, const tf2_ros::Buffer& tf_buffer)
    {
        try
        {
            // Look up the transform from the source frame to the target frame
            geometry_msgs::msg::TransformStamped transform = tf_buffer.lookupTransform(
                target_frame, msg->header.frame_id, tf2::TimePointZero);
            // Transform the message
            T transformed_msg;
            tf2::doTransform(*msg, transformed_msg, transform);
            // Clone timestamp
            transformed_msg.header.stamp = msg->header.stamp;
            // Publish the transformed message
            publisher_->publish(transformed_msg);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(rclcpp::get_logger("GenericTransformHandler"), "Could not transform message: %s", ex.what());
        }
    }

    typename rclcpp::Subscription<T>::SharedPtr subscription_;
    typename rclcpp::Publisher<T>::SharedPtr publisher_;
};

// Factory function to create the appropriate handler based on the message type
std::shared_ptr<TransformHandler> create_handler(const std::string& type)
{
    if (type == "geometry_msgs/msg/PointStamped")
    {
        return std::make_shared<GenericTransformHandler<geometry_msgs::msg::PointStamped>>();
    }
    // Add more conditions here to handle other message types
    return nullptr;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 4)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run turtlebot3_support tf2_transform_node <topic> <type> <target_frame> [output_topic]");
        return 1;
    }

    std::string input_topic = argv[1];
    std::string type = argv[2];
    std::string target_frame = argv[3];
    std::string output_topic = (argc > 4) ? argv[4] : (input_topic + "_" + target_frame);

    // Create the appropriate handler based on the message type
    auto handler = create_handler(type);
    if (!handler)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unsupported message type: %s", type.c_str());
        return 1;
    }

    auto node = rclcpp::Node::make_shared("tf2_transform_node");
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Subscribe to the input topic and set up the transformation
    handler->subscribe(node, input_topic, output_topic, target_frame, tf_buffer);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}