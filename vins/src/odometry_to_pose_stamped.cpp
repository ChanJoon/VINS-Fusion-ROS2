#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class OdometryToPoseStamped : public rclcpp::Node
{
public:
    OdometryToPoseStamped()
        : Node("odometry_to_pose_stamped")
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/vins_estimator/odometry");
        output_topic_ = declare_parameter<std::string>("output_topic", "/mavros/vision_pose/pose");
        frame_id_ = declare_parameter<std::string>("frame_id", "odom");
        max_publish_rate_hz_ = declare_parameter<double>("max_publish_rate_hz", 40.0);

        if (!std::isfinite(max_publish_rate_hz_) || max_publish_rate_hz_ < 0.0)
        {
            RCLCPP_WARN(get_logger(), "max_publish_rate_hz must be finite and non-negative; disabling rate limit");
            max_publish_rate_hz_ = 0.0;
        }
        min_publish_period_ns_ =
            max_publish_rate_hz_ > 0.0
                ? static_cast<int64_t>(std::llround(1.0e9 / max_publish_rate_hz_))
                : 0;

        publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, rclcpp::QoS(10));
        subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            input_topic_, rclcpp::QoS(10),
            std::bind(&OdometryToPoseStamped::odometryCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(), "Forwarding pose %s -> %s as frame_id=%s at %s",
            input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(),
            rateLimitDescription().c_str());
    }

private:
    std::string rateLimitDescription() const
    {
        if (max_publish_rate_hz_ <= 0.0)
        {
            return "unlimited rate";
        }
        return std::to_string(max_publish_rate_hz_) + " Hz max";
    }

    int64_t messageStampNanoseconds(const nav_msgs::msg::Odometry &odometry) const
    {
        const int64_t stamp_ns = rclcpp::Time(odometry.header.stamp).nanoseconds();
        return stamp_ns > 0 ? stamp_ns : now().nanoseconds();
    }

    bool shouldSkipForRateLimit(const nav_msgs::msg::Odometry &odometry)
    {
        if (min_publish_period_ns_ <= 0)
        {
            return false;
        }

        const int64_t stamp_ns = messageStampNanoseconds(odometry);
        if (has_last_published_stamp_ &&
            stamp_ns >= last_published_stamp_ns_ &&
            stamp_ns - last_published_stamp_ns_ < min_publish_period_ns_)
        {
            return true;
        }

        last_published_stamp_ns_ = stamp_ns;
        has_last_published_stamp_ = true;
        return false;
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (shouldSkipForRateLimit(*msg))
        {
            return;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        if (!frame_id_.empty())
        {
            pose.header.frame_id = frame_id_;
        }
        pose.pose = msg->pose.pose;
        publisher_->publish(pose);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;
    double max_publish_rate_hz_;
    int64_t min_publish_period_ns_ = 0;
    int64_t last_published_stamp_ns_ = 0;
    bool has_last_published_stamp_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryToPoseStamped>());
    rclcpp::shutdown();
    return 0;
}
