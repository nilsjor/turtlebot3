#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TF2TransformNode : public rclcpp::Node
{
public:
    TF2TransformNode(const std::string &input_topic, const std::string &target_frame, const std::string &output_topic)
        : Node("tf2_transform_node"), target_frame_(target_frame)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            input_topic, 10,
            std::bind(&TF2TransformNode::point_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(output_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Transforming points from '%s' to frame '%s' and publishing on '%s'.",
                    input_topic.c_str(), target_frame.c_str(), output_topic.c_str());
    }

private:
    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                target_frame_,
                msg->header.frame_id,
                tf2::TimePointZero);

            geometry_msgs::msg::PointStamped transformed_msg;
            tf2::doTransform(*msg, transformed_msg, transform_stamped);
            publisher_->publish(transformed_msg);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
        }
    }

    std::string target_frame_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run turtlebot_support tf2_transform <topic> <new_frame> [output_topic]");
        return 1;
    }

    std::string input_topic = argv[1];
    std::string target_frame = argv[2];
    std::string output_topic = (argc > 3) ? argv[3] : (input_topic + "_" + target_frame);

    auto node = std::make_shared<TF2TransformNode>(input_topic, target_frame, output_topic);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
