#ifndef SWERVE_CONTROLLER_HPP
#define SWERVE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>


class SwerveController : public rclcpp::Node
{
public:
    SwerveController(const std::string& name);

private:
    void velCallback(const geometry_msgs::msg::TwistStamped &msg);

    void jointCallback(const sensor_msgs::msg::JointState &msg);

    void initializeWheelCmdMsg(std_msgs::msg::Float64MultiArray &wheel_cmd_msg);

    void initializeCasterCmdMsg(trajectory_msgs::msg::JointTrajectory &caster_cmd_msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr caster_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Odometry
    double wheel_radius_;
    double wheel_base_; //distance between front and rear wheels
    double track_width_; //distance between left and right wheels
    Eigen::Matrix2d speed_conversion_;
    //
    double front_left_wheel_prev_pos_;
    double front_right_wheel_prev_pos_;
    double back_left_wheel_prev_pos_;
    double back_right_wheel_prev_pos_;
    //
    double front_left_caster_prev_pos_;
    double front_right_caster_prev_pos_;
    double back_left_caster_prev_pos_;
    double back_right_caster_prev_pos_;
    //
    rclcpp::Time prev_time_;
    nav_msgs::msg::Odometry odom_msg_;
    double x_;
    double y_;
    double theta_;
    int module_number_ = 4;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif // SWERVE_CONTROLLER_HPP