#include "swervebot_controller/swerve_controller.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

using std::placeholders::_1;

SwerveController::SwerveController(const std::string& name)
                                    : Node(name),
                                      wheel_radius_(0.08),
                                      wheel_base_(0.604),
                                      track_width_(0.475),
                                      front_left_wheel_prev_pos_(0.0),
                                      front_right_wheel_prev_pos_(0.0),
                                      back_left_wheel_prev_pos_(0.0),
                                      back_right_wheel_prev_pos_(0.0),
                                      front_left_caster_prev_pos_(0.0),
                                      front_right_caster_prev_pos_(0.0),
                                      back_left_caster_prev_pos_(0.0),
                                      back_right_caster_prev_pos_(0.0),
                                      x_(0.0),
                                      y_(0.0),
                                      theta_(0.0),
                                      module_number_(4)
{
    declare_parameter("wheel_radius", 0.08);
    declare_parameter("wheel_base", 0.604);
    declare_parameter("track_width", 0.475);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_base_ = get_parameter("wheel_base").as_double();
    track_width_ = get_parameter("track_width").as_double();
    // RCLCPP_INFO(this->get_logger(), "SwerveController parameters: wheel_radius=%.3f, wheel_base=%.3f, track_width=%.3f",
    //             wheel_radius_, wheel_base_, track_width_);
    
    // Create publishers
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_trajectory_controller/commands", 10);
    caster_cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/caster_trajectory_controller/joint_trajectory", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Create subscribers
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10, std::bind(&SwerveController::velCallback, this, _1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SwerveController::jointCallback, this, _1));

    // Fill the Odometry message with invariant parameters
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";

    prev_time_ = get_clock()->now();
}

void SwerveController::initializeWheelCmdMsg(std_msgs::msg::Float64MultiArray &wheel_cmd_msg)
{
    wheel_cmd_msg.data.resize(module_number_); // 4 wheels
    for (int i = 0; i < module_number_; ++i) {
        wheel_cmd_msg.data[i] = 0.0;
    }
}

void SwerveController::initializeCasterCmdMsg(trajectory_msgs::msg::JointTrajectory &caster_cmd_msg)
{
    caster_cmd_msg.joint_names.resize(module_number_); // 4 casters
    for (int i = 0; i < module_number_; ++i) {
        if (i == 0) {
            caster_cmd_msg.joint_names[i] = "base_frontleft_caster_joint";
        }
        else if (i == 1) {
            caster_cmd_msg.joint_names[i] = "base_frontright_caster_joint";
        }
        else if (i == 2) {
            caster_cmd_msg.joint_names[i] = "base_backleft_caster_joint";
        }
        else if (i == 3) {
            caster_cmd_msg.joint_names[i] = "base_backright_caster_joint";
        }
    }
    caster_cmd_msg.points.resize(1);
    caster_cmd_msg.points[0].positions.resize(module_number_);
    for (int i = 0; i < module_number_; ++i) {
        caster_cmd_msg.points[0].positions[i] = 0.0;
    }
}

void SwerveController::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    std_msgs::msg::Float64MultiArray wheel_cmd_msg;
    initializeWheelCmdMsg(wheel_cmd_msg);

    trajectory_msgs::msg::JointTrajectory caster_cmd_msg;
    initializeCasterCmdMsg(caster_cmd_msg);

    // Process velocity command
    double vx = msg.twist.linear.x;
    double vy = msg.twist.linear.y;
    double omega = msg.twist.angular.z;

    // Compute wheel speeds
    for (int i = 0; i < module_number_; ++i) {
        float wheel_base_position_offset;
        float track_width_position_offset;
        if (i == 0) { // Front Left Module
            wheel_base_position_offset = wheel_base_ / 2.0;
            track_width_position_offset = track_width_ / 2.0;
        }
        else if (i == 1) { // Front Right Module
            wheel_base_position_offset = wheel_base_ / 2.0;
            track_width_position_offset = -track_width_ / 2.0;
        }
        else if (i == 2) { // Back Left Module
            wheel_base_position_offset = -wheel_base_ / 2.0;
            track_width_position_offset = track_width_ / 2.0;
        }
        else if (i == 3) { // Back Right Module
            wheel_base_position_offset = -wheel_base_ / 2.0;
            track_width_position_offset = -track_width_ / 2.0;
        }
        double wheel_vx = vx - omega * track_width_position_offset;
        double wheel_vy = vy + omega * wheel_base_position_offset;
        double wheel_speed = std::sqrt(wheel_vx * wheel_vx + wheel_vy * wheel_vy);
        double wheel_angle = std::atan2(wheel_vy, wheel_vx);
        wheel_cmd_msg.data[i] = wheel_speed / wheel_radius_; 
        caster_cmd_msg.points[0].positions[i] = wheel_angle;
    }

    // Publish wheel commands
    wheel_cmd_pub_->publish(wheel_cmd_msg);

    caster_cmd_pub_->publish(caster_cmd_msg);
}

void SwerveController::jointCallback(const sensor_msgs::msg::JointState &msg)
{
    
  const rclcpp::Time msg_time = msg.header.stamp;

  // First message init
  if (prev_time_.nanoseconds() == 0) {
    prev_time_ = msg_time;

    back_left_wheel_prev_pos_    = msg.position[0];
    back_right_wheel_prev_pos_   = msg.position[1];
    back_left_caster_prev_pos_   = msg.position[2];
    back_right_caster_prev_pos_  = msg.position[3];
    front_left_caster_prev_pos_  = msg.position[4];
    front_right_caster_prev_pos_ = msg.position[5];
    front_left_wheel_prev_pos_   = msg.position[6];
    front_right_wheel_prev_pos_  = msg.position[7];
    return;
  }

  const double dt = (msg_time - prev_time_).seconds();
  if (dt <= 0.0) {
    prev_time_ = msg_time;
    return;
  }

  // Wheel deltas
  const double dp_fl_w = msg.position[6] - front_left_wheel_prev_pos_;
  const double dp_fr_w = msg.position[7] - front_right_wheel_prev_pos_;
  const double dp_bl_w = msg.position[0] - back_left_wheel_prev_pos_;
  const double dp_br_w = msg.position[1] - back_right_wheel_prev_pos_;

  // Steering angles 
  const double a_fl = msg.position[4];
  const double a_fr = msg.position[5];
  const double a_bl = msg.position[2];
  const double a_br = msg.position[3];

  // Update prev positions 
  front_left_wheel_prev_pos_  = msg.position[6];
  front_right_wheel_prev_pos_ = msg.position[7];
  back_left_wheel_prev_pos_   = msg.position[0];
  back_right_wheel_prev_pos_  = msg.position[1];

  front_left_caster_prev_pos_  = msg.position[4];
  front_right_caster_prev_pos_ = msg.position[5];
  back_left_caster_prev_pos_   = msg.position[2];
  back_right_caster_prev_pos_  = msg.position[3];

  // Module linear speeds along wheel direction 
  const double s_fl = (dp_fl_w / dt) * wheel_radius_;
  const double s_fr = (dp_fr_w / dt) * wheel_radius_;
  const double s_bl = (dp_bl_w / dt) * wheel_radius_;
  const double s_br = (dp_br_w / dt) * wheel_radius_;

  // Module velocity vectors in base frame
  const double v_fl_x = s_fl * std::cos(a_fl);
  const double v_fl_y = s_fl * std::sin(a_fl);

  const double v_fr_x = s_fr * std::cos(a_fr);
  const double v_fr_y = s_fr * std::sin(a_fr);

  const double v_bl_x = s_bl * std::cos(a_bl);
  const double v_bl_y = s_bl * std::sin(a_bl);

  const double v_br_x = s_br * std::cos(a_br);
  const double v_br_y = s_br * std::sin(a_br);

  // Base vx, vy = average of module vectors
  const double vx = 0.25 * (v_fl_x + v_fr_x + v_bl_x + v_br_x);
  const double vy = 0.25 * (v_fl_y + v_fr_y + v_bl_y + v_br_y);

  // Estimate omega 
  const double lx = wheel_base_  * 0.5;
  const double ly = track_width_ * 0.5;

  double omega_num = 0.0;
  double omega_den = 0.0;

  auto add_module = [&](double x_i, double y_i, double vix, double viy) {
    omega_num += x_i * (viy - vy) - y_i * (vix - vx);
    omega_den += x_i * x_i + y_i * y_i;
  };

  add_module(+lx, +ly, v_fl_x, v_fl_y); // FL
  add_module(+lx, -ly, v_fr_x, v_fr_y); // FR
  add_module(-lx, +ly, v_bl_x, v_bl_y); // BL
  add_module(-lx, -ly, v_br_x, v_br_y); // BR

  const double omega = (omega_den > 1e-9) ? (omega_num / omega_den) : 0.0;

  // Integrate odometry
  const double theta_mid = theta_ + 0.5 * omega * dt;
  const double c = std::cos(theta_mid);
  const double s = std::sin(theta_mid);

  x_     += (vx * c - vy * s) * dt;
  y_     += (vx * s + vy * c) * dt;
  theta_ += omega * dt;

  // Normalize theta
  while (theta_ > M_PI)  theta_ -= 2.0 * M_PI;
  while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

  // Publish odom + TF
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);

  odom_msg_.header.stamp = msg_time;
  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;
  odom_msg_.pose.pose.orientation.x = q.getX();
  odom_msg_.pose.pose.orientation.y = q.getY();
  odom_msg_.pose.pose.orientation.z = q.getZ();
  odom_msg_.pose.pose.orientation.w = q.getW();
  odom_msg_.twist.twist.linear.x  = vx;
  odom_msg_.twist.twist.linear.y  = vy;
  odom_msg_.twist.twist.angular.z = omega;
  odom_pub_->publish(odom_msg_);

  transform_stamped_.header.stamp = msg_time;
  transform_stamped_.transform.translation.x = x_;
  transform_stamped_.transform.translation.y = y_;
  transform_stamped_.transform.rotation.x = q.getX();
  transform_stamped_.transform.rotation.y = q.getY();
  transform_stamped_.transform.rotation.z = q.getZ();
  transform_stamped_.transform.rotation.w = q.getW();
  transform_broadcaster_->sendTransform(transform_stamped_);

  // Update time
  prev_time_ = msg_time;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwerveController>("swerve_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}