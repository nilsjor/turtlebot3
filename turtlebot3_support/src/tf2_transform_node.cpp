#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>

class TransformHandler
{
public:
    virtual void subscribe(rclcpp::Node::SharedPtr node, const std::string& input_topic, const std::string& output_topic,
                           const std::string& target_frame, const tf2_ros::Buffer& tf_buffer) = 0;
    virtual ~TransformHandler() = default;
};

class PointStampedHandler : public TransformHandler
{
public:
    void subscribe(rclcpp::Node::SharedPtr node, const std::string& input_topic, const std::string& output_topic,
                   const std::string& target_frame, const tf2_ros::Buffer& tf_buffer) override
    {

        subscription_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
            input_topic, 10, 
            [this, target_frame, &tf_buffer](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                this->callback(msg, target_frame, tf_buffer);
            });

        publisher_ = node->create_publisher<geometry_msgs::msg::PointStamped>(output_topic, 10);
        RCLCPP_INFO(node->get_logger(), "Transforming points from '%s' to frame '%s' and publishing on '%s'.",
                    input_topic.c_str(), target_frame.c_str(), output_topic.c_str());
    }

private:
    void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg, 
                  const std::string& target_frame, const tf2_ros::Buffer& tf_buffer)
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform = tf_buffer.lookupTransform(
                target_frame, msg->header.frame_id, tf2::TimePointZero);
            geometry_msgs::msg::PointStamped transformed_msg;
            tf2::doTransform(*msg, transformed_msg, transform);
            publisher_->publish(transformed_msg);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(rclcpp::get_logger("PointStampedHandler"), "Could not transform point: %s", ex.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
};

std::shared_ptr<TransformHandler> create_handler(const std::string& type)
{
    if (type == "geometry_msgs/msg/PointStamped")
    {
        return std::make_shared<PointStampedHandler>();
    }
    return nullptr;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 4)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run turtlebot_support tf2_transform <topic> <type> <target_frame> [output_topic]");
        return 1;
    }

    std::string input_topic = argv[1];
    std::string type = argv[2];
    std::string target_frame = argv[3];
    std::string output_topic = (argc > 4) ? argv[4] : (input_topic + "_" + target_frame);

    auto handler = create_handler(type);
    if (!handler)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unsupported message type: %s", type.c_str());
        return 1;
    }

    auto node = rclcpp::Node::make_shared("tf2_transform_node");
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    handler->subscribe(node, input_topic, output_topic, target_frame, tf_buffer);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
