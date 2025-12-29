#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>

#include <laser_geometry/laser_geometry.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <limits>
#include <cmath>
#include <string>
#include <vector>

using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;

class DualScanMerger : public rclcpp::Node
{
public:
  DualScanMerger()
  : Node("dual_scan_merger"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // --- Parameters ---
    scan1_topic_   = declare_parameter<std::string>("scan1_topic", "/scan1");
    scan2_topic_   = declare_parameter<std::string>("scan2_topic", "/scan2");
    output_topic_  = declare_parameter<std::string>("output_topic", "/scan_merged");
    target_frame_  = declare_parameter<std::string>("target_frame", "base_link");

    queue_size_    = declare_parameter<int>("queue_size", 20);
    slop_sec_      = declare_parameter<double>("slop", 0.05);  // ApproxTime slop
    stamp_policy_  = declare_parameter<std::string>("stamp_policy", "latest"); // "latest" or "now"

    out_angle_min_ = declare_parameter<double>("angle_min", std::numeric_limits<double>::quiet_NaN());
    out_angle_max_ = declare_parameter<double>("angle_max", std::numeric_limits<double>::quiet_NaN());
    out_angle_inc_ = declare_parameter<double>("angle_increment", std::numeric_limits<double>::quiet_NaN());

    out_range_min_ = declare_parameter<double>("range_min", std::numeric_limits<double>::quiet_NaN());
    out_range_max_ = declare_parameter<double>("range_max", std::numeric_limits<double>::quiet_NaN());

    // Publisher
    pub_ = create_publisher<LaserScan>(output_topic_, rclcpp::QoS(10).reliable());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_.setCreateTimerInterface(timer_interface);

    // Subscribers + Sync
    sub1_ = std::make_shared<message_filters::Subscriber<LaserScan>>(this, scan1_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    sub2_ = std::make_shared<message_filters::Subscriber<LaserScan>>(this, scan2_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());

    using Policy = message_filters::sync_policies::ApproximateTime<LaserScan, LaserScan>;
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(queue_size_), *sub1_, *sub2_);

    sync_->registerCallback(std::bind(&DualScanMerger::cb, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "Merging '%s' + '%s' -> '%s' in frame '%s' (queue=%d)",
      scan1_topic_.c_str(), scan2_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(), queue_size_);
  }

private:
  static bool is_nan(double v) { return std::isnan(v); }

  void choose_output_params(
    const LaserScan &s1, const LaserScan &s2,
    double &angle_min, double &angle_max, double &angle_inc,
    double &range_min, double &range_max, int &n_bins)
  {
    // Angle bins
    if (!is_nan(out_angle_min_) && !is_nan(out_angle_max_) && !is_nan(out_angle_inc_)) {
      angle_min = out_angle_min_;
      angle_max = out_angle_max_;
      angle_inc = out_angle_inc_;
    } else {
      angle_min = std::min(s1.angle_min, s2.angle_min);
      angle_max = std::max(s1.angle_max, s2.angle_max);
      angle_inc = std::min(s1.angle_increment, s2.angle_increment);
    }

    // Range limits
    range_min = (!is_nan(out_range_min_)) ? out_range_min_ : std::max(0.0f, std::min(s1.range_min, s2.range_min));
    range_max = (!is_nan(out_range_max_)) ? out_range_max_ : std::max(s1.range_max, s2.range_max);

    n_bins = static_cast<int>(std::round((angle_max - angle_min) / angle_inc)) + 1;
    if (n_bins <= 0 || angle_inc <= 0.0) {
      throw std::runtime_error("Invalid output scan parameters (bins <= 0 or angle_increment <= 0).");
    }
  }

  static builtin_interfaces::msg::Time timeToMsg(const rclcpp::Time & t)
    {
        builtin_interfaces::msg::Time m;
        const int64_t ns = t.nanoseconds();
        m.sec = static_cast<int32_t>(ns / 1000000000LL);
        m.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
        return m;
    }

  builtin_interfaces::msg::Time choose_stamp(const LaserScan &s1, const LaserScan &s2)
  {
    if (stamp_policy_ == "now") {
      return timeToMsg(this->get_clock()->now());
    }
    // latest
    rclcpp::Time t1(s1.header.stamp);
    rclcpp::Time t2(s2.header.stamp);
    return timeToMsg((t1 >= t2) ? t1 : t2);
  }

  PointCloud2 project_and_transform(const LaserScan &scan)
  {
    // LaserScan -> PointCloud2 in scan frame
    PointCloud2 cloud_scan;
    projector_.projectLaser(scan, cloud_scan);

    // TF to target frame
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        target_frame_,
        scan.header.frame_id,
        rclcpp::Time(scan.header.stamp),
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException &ex) {
      throw std::runtime_error(
        "TF lookup failed " + scan.header.frame_id + " -> " + target_frame_ + ": " + ex.what());
    }

    PointCloud2 cloud_out;
    tf2::doTransform(cloud_scan, cloud_out, tf);
    return cloud_out;
  }

  void integrate_cloud_into_ranges(
    const PointCloud2 &cloud,
    double angle_min, double angle_max, double angle_inc,
    double range_min, double range_max,
    std::vector<float> &ranges)
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    const int n = static_cast<int>(ranges.size());

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;

      const double r = std::hypot(x, y);
      if (r < range_min || r > range_max) {
        continue;
      }

      const double a = std::atan2(y, x);
      if (a < angle_min || a > angle_max) {
        continue;
      }

      const int idx = static_cast<int>((a - angle_min) / angle_inc);
      if (idx >= 0 && idx < n) {
        if (r < ranges[idx]) {
          ranges[idx] = static_cast<float>(r);
        }
      }
    }
  }

  void cb(const LaserScan::ConstSharedPtr s1, const LaserScan::ConstSharedPtr s2)
  {
    try {
      double angle_min, angle_max, angle_inc, range_min, range_max;
      int n_bins = 0;
      choose_output_params(*s1, *s2, angle_min, angle_max, angle_inc, range_min, range_max, n_bins);

      std::vector<float> ranges(n_bins, std::numeric_limits<float>::infinity());

      // Transform each scan -> target_frame as pointcloud
      const auto c1 = project_and_transform(*s1);
      const auto c2 = project_and_transform(*s2);

      integrate_cloud_into_ranges(c1, angle_min, angle_max, angle_inc, range_min, range_max, ranges);
      integrate_cloud_into_ranges(c2, angle_min, angle_max, angle_inc, range_min, range_max, ranges);

      LaserScan out;
      out.header.stamp = choose_stamp(*s1, *s2);
      out.header.frame_id = target_frame_;

      out.angle_min = static_cast<float>(angle_min);
      out.angle_max = static_cast<float>(angle_max);
      out.angle_increment = static_cast<float>(angle_inc);
      out.time_increment = 0.0f;
      out.scan_time = 0.0f;
      out.range_min = static_cast<float>(range_min);
      out.range_max = static_cast<float>(range_max);

      out.ranges = std::move(ranges);

      pub_->publish(out);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Merge failed: %s", e.what());
    }
  }

private:
  // Params
  std::string scan1_topic_;
  std::string scan2_topic_;
  std::string output_topic_;
  std::string target_frame_;
  int queue_size_;
  double slop_sec_;
  std::string stamp_policy_;

  double out_angle_min_;
  double out_angle_max_;
  double out_angle_inc_;
  double out_range_min_;
  double out_range_max_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Laser projection
  laser_geometry::LaserProjection projector_;

  // Pub/Sub
  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
  std::shared_ptr<message_filters::Subscriber<LaserScan>> sub1_;
  std::shared_ptr<message_filters::Subscriber<LaserScan>> sub2_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<LaserScan, LaserScan>>> sync_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualScanMerger>());
  rclcpp::shutdown();
  return 0;
}
