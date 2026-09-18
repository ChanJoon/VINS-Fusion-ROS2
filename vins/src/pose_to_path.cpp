#include <algorithm>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

class PoseToPath : public rclcpp::Node
{
public:
    PoseToPath()
        : Node("pose_to_path")
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/mavros/local_position/pose");
        output_topic_ = declare_parameter<std::string>("output_topic", "/mavros/local_position/path");
        fixed_frame_id_ = declare_parameter<std::string>("fixed_frame_id", "");
        max_poses_ = declare_parameter<int>("max_poses", 3000);
        if (max_poses_ <= 0)
        {
            max_poses_ = 3000;
        }

        publisher_ = create_publisher<nav_msgs::msg::Path>(output_topic_, rclcpp::QoS(10));
        auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            input_topic_, pose_qos,
            std::bind(&PoseToPath::poseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(), "Accumulating %s -> %s with fixed_frame_id=%s max_poses=%d",
            input_topic_.c_str(), output_topic_.c_str(), fixed_frame_id_.c_str(), max_poses_);
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose = *msg;
        if (!fixed_frame_id_.empty())
        {
            pose.header.frame_id = fixed_frame_id_;
        }

        path_.header = pose.header;
        path_.poses.push_back(pose);
        if (static_cast<int>(path_.poses.size()) > max_poses_)
        {
            const auto excess = path_.poses.size() - static_cast<size_t>(max_poses_);
            path_.poses.erase(path_.poses.begin(), path_.poses.begin() + static_cast<long>(excess));
        }
        publisher_->publish(path_);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string fixed_frame_id_;
    int max_poses_;
    nav_msgs::msg::Path path_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseToPath>());
    rclcpp::shutdown();
    return 0;
}
