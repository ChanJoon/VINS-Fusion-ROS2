#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>

class VinsToMavrosOdometry : public rclcpp::Node
{
public:
    VinsToMavrosOdometry()
        : Node("vins_to_mavros_odometry")
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/vins_estimator/odometry");
        output_topic_ = declare_parameter<std::string>("output_topic", "/mavros/odometry/out");
        frame_id_ = declare_parameter<std::string>("frame_id", "odom");
        child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
        max_publish_rate_hz_ = declare_parameter<double>("max_publish_rate_hz", 40.0);
        reject_large_jumps_ = declare_parameter<bool>("reject_large_jumps", true);
        max_position_jump_m_ = declare_parameter<double>("max_position_jump_m", 2.0);
        max_orientation_jump_deg_ = declare_parameter<double>("max_orientation_jump_deg", 45.0);
        convert_linear_velocity_to_child_frame_ =
            declare_parameter<bool>("convert_linear_velocity_to_child_frame", true);
        publish_static_tf_ = declare_parameter<bool>("publish_mavros_static_tf", true);
        pose_covariance_diagonal_ = declare_parameter<std::vector<double>>(
            "pose_covariance_diagonal", {0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
        twist_covariance_diagonal_ = declare_parameter<std::vector<double>>(
            "twist_covariance_diagonal", {0.04, 0.04, 0.04, 0.25, 0.25, 0.25});

        validateCovariance("pose_covariance_diagonal", pose_covariance_diagonal_);
        validateCovariance("twist_covariance_diagonal", twist_covariance_diagonal_);
        validatePublishRate();
        validateJumpThresholds();

        publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(10));
        subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            input_topic_, rclcpp::QoS(10),
            std::bind(&VinsToMavrosOdometry::odometryCallback, this, std::placeholders::_1));

        if (publish_static_tf_)
        {
            static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
            publishMavrosStaticTransforms();
        }

        RCLCPP_INFO(
            get_logger(), "Forwarding %s -> %s as %s -> %s at %s",
            input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(), child_frame_id_.c_str(),
            rateLimitDescription().c_str());
    }

private:
    static bool covarianceUnset(const std::array<double, 36> &covariance)
    {
        return std::all_of(covariance.begin(), covariance.end(), [](double value) {
            return std::abs(value) < 1e-12;
        });
    }

    static void fillCovarianceDiagonal(
        std::array<double, 36> &covariance,
        const std::vector<double> &diagonal)
    {
        covariance.fill(0.0);
        for (size_t i = 0; i < 6; ++i)
        {
            covariance[i * 6 + i] = diagonal[i];
        }
    }

    void validateCovariance(const std::string &name, std::vector<double> &diagonal)
    {
        if (diagonal.size() == 6)
        {
            return;
        }

        RCLCPP_WARN(
            get_logger(), "%s must contain 6 values; falling back to conservative defaults",
            name.c_str());
        diagonal = (name == "pose_covariance_diagonal")
                       ? std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01}
                       : std::vector<double>{0.04, 0.04, 0.04, 0.25, 0.25, 0.25};
    }

    static double clampUnit(double value)
    {
        return std::max(-1.0, std::min(1.0, value));
    }

    void validatePublishRate()
    {
        if (!std::isfinite(max_publish_rate_hz_) || max_publish_rate_hz_ < 0.0)
        {
            RCLCPP_WARN(
                get_logger(), "max_publish_rate_hz must be finite and non-negative; disabling rate limit");
            max_publish_rate_hz_ = 0.0;
        }

        min_publish_period_ns_ =
            max_publish_rate_hz_ > 0.0
                ? static_cast<int64_t>(std::llround(1.0e9 / max_publish_rate_hz_))
                : 0;
    }

    void validateJumpThresholds()
    {
        if (!std::isfinite(max_position_jump_m_) || max_position_jump_m_ <= 0.0)
        {
            RCLCPP_WARN(
                get_logger(), "max_position_jump_m must be positive; disabling position jump rejection");
            max_position_jump_m_ = 0.0;
        }
        if (!std::isfinite(max_orientation_jump_deg_) || max_orientation_jump_deg_ <= 0.0)
        {
            RCLCPP_WARN(
                get_logger(), "max_orientation_jump_deg must be positive; disabling orientation jump rejection");
            max_orientation_jump_deg_ = 0.0;
        }
    }

    std::string rateLimitDescription() const
    {
        if (max_publish_rate_hz_ <= 0.0)
        {
            return "unlimited rate";
        }
        return std::to_string(max_publish_rate_hz_) + " Hz max";
    }

    static geometry_msgs::msg::TransformStamped makeStaticTransform(
        const rclcpp::Time &stamp,
        const std::string &target_frame,
        const std::string &source_frame,
        const tf2::Quaternion &rotation)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = target_frame;
        transform.child_frame_id = source_frame;
        transform.transform.rotation.x = rotation.x();
        transform.transform.rotation.y = rotation.y();
        transform.transform.rotation.z = rotation.z();
        transform.transform.rotation.w = rotation.w();
        return transform;
    }

    void publishMavrosStaticTransforms()
    {
        tf2::Quaternion enu_to_ned;
        enu_to_ned.setValue(std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0);

        tf2::Quaternion flu_to_frd;
        flu_to_frd.setValue(1.0, 0.0, 0.0, 0.0);

        const auto stamp = now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(makeStaticTransform(stamp, frame_id_, frame_id_ + "_ned", enu_to_ned));
        transforms.push_back(makeStaticTransform(stamp, child_frame_id_, child_frame_id_ + "_frd", flu_to_frd));
        static_broadcaster_->sendTransform(transforms);
    }

    static bool finiteQuaternion(const geometry_msgs::msg::Quaternion &q)
    {
        return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
    }

    void convertLinearVelocityToChildFrame(nav_msgs::msg::Odometry &odometry)
    {
        const auto &orientation = odometry.pose.pose.orientation;
        if (!finiteQuaternion(orientation))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skipping velocity frame conversion because the pose quaternion is not finite");
            return;
        }

        tf2::Quaternion q(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);

        if (q.length2() < 1e-12)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skipping velocity frame conversion because the pose quaternion is near zero");
            return;
        }

        q.normalize();
        const tf2::Matrix3x3 rotation(q);
        const tf2::Vector3 velocity_parent(
            odometry.twist.twist.linear.x,
            odometry.twist.twist.linear.y,
            odometry.twist.twist.linear.z);
        const tf2::Vector3 velocity_child(
            rotation[0][0] * velocity_parent.x() + rotation[1][0] * velocity_parent.y() + rotation[2][0] * velocity_parent.z(),
            rotation[0][1] * velocity_parent.x() + rotation[1][1] * velocity_parent.y() + rotation[2][1] * velocity_parent.z(),
            rotation[0][2] * velocity_parent.x() + rotation[1][2] * velocity_parent.y() + rotation[2][2] * velocity_parent.z());

        odometry.twist.twist.linear.x = velocity_child.x();
        odometry.twist.twist.linear.y = velocity_child.y();
        odometry.twist.twist.linear.z = velocity_child.z();
    }

    int64_t messageStampNanoseconds(const nav_msgs::msg::Odometry &odometry) const
    {
        const int64_t stamp_ns = rclcpp::Time(odometry.header.stamp).nanoseconds();
        return stamp_ns > 0 ? stamp_ns : now().nanoseconds();
    }

    static double positionDistance(
        const geometry_msgs::msg::Point &a,
        const geometry_msgs::msg::Point &b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    static bool quaternionFromMsg(
        const geometry_msgs::msg::Quaternion &msg,
        tf2::Quaternion &q)
    {
        if (!finiteQuaternion(msg))
        {
            return false;
        }
        q.setValue(msg.x, msg.y, msg.z, msg.w);
        if (q.length2() < 1e-12)
        {
            return false;
        }
        q.normalize();
        return true;
    }

    static double orientationDistanceDeg(
        const geometry_msgs::msg::Quaternion &a,
        const geometry_msgs::msg::Quaternion &b)
    {
        tf2::Quaternion qa;
        tf2::Quaternion qb;
        if (!quaternionFromMsg(a, qa) || !quaternionFromMsg(b, qb))
        {
            return 0.0;
        }

        const double dot = std::abs(qa.dot(qb));
        constexpr double rad_to_deg = 180.0 / 3.14159265358979323846;
        return 2.0 * std::acos(clampUnit(dot)) * rad_to_deg;
    }

    bool shouldRejectLargeJump(const nav_msgs::msg::Odometry &odometry)
    {
        if (!reject_large_jumps_)
        {
            return false;
        }

        if (!has_last_published_pose_)
        {
            last_published_pose_ = odometry.pose.pose;
            has_last_published_pose_ = true;
            return false;
        }

        const double position_jump =
            positionDistance(odometry.pose.pose.position, last_published_pose_.position);
        const double orientation_jump =
            orientationDistanceDeg(odometry.pose.pose.orientation, last_published_pose_.orientation);

        const bool position_rejected =
            max_position_jump_m_ > 0.0 && position_jump > max_position_jump_m_;
        const bool orientation_rejected =
            max_orientation_jump_deg_ > 0.0 && orientation_jump > max_orientation_jump_deg_;

        if (position_rejected || orientation_rejected)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Rejecting VIO odometry jump before MAVROS: position %.3f m, orientation %.3f deg",
                position_jump, orientation_jump);
            return true;
        }

        last_published_pose_ = odometry.pose.pose;
        return false;
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
        auto odometry = *msg;
        if (shouldSkipForRateLimit(odometry))
        {
            return;
        }
        if (shouldRejectLargeJump(odometry))
        {
            return;
        }

        odometry.header.frame_id = frame_id_;
        odometry.child_frame_id = child_frame_id_;

        if (convert_linear_velocity_to_child_frame_)
        {
            convertLinearVelocityToChildFrame(odometry);
        }

        if (covarianceUnset(odometry.pose.covariance))
        {
            fillCovarianceDiagonal(odometry.pose.covariance, pose_covariance_diagonal_);
        }
        if (covarianceUnset(odometry.twist.covariance))
        {
            fillCovarianceDiagonal(odometry.twist.covariance, twist_covariance_diagonal_);
        }

        publisher_->publish(odometry);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;
    std::string child_frame_id_;
    double max_publish_rate_hz_;
    int64_t min_publish_period_ns_ = 0;
    int64_t last_published_stamp_ns_ = 0;
    bool has_last_published_stamp_ = false;
    bool reject_large_jumps_;
    double max_position_jump_m_;
    double max_orientation_jump_deg_;
    geometry_msgs::msg::Pose last_published_pose_;
    bool has_last_published_pose_ = false;
    bool convert_linear_velocity_to_child_frame_;
    bool publish_static_tf_;
    std::vector<double> pose_covariance_diagonal_;
    std::vector<double> twist_covariance_diagonal_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VinsToMavrosOdometry>());
    rclcpp::shutdown();
    return 0;
}
