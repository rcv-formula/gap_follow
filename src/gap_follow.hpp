
#ifndef GAP_FOLLOW_HPP
#define GAP_FOLLOW_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <utility>
#include <vector>

class ReactiveGapFollow : public rclcpp::Node
{
  public:
    ReactiveGapFollow();

  private:
    struct PathPoint
    {
        double x = 0.0;
        double y = 0.0;
    };

    struct PathGuidance
    {
        bool active = false;
        float target_angle = 0.0F;
        float cross_track_error = 0.0F;
        float score_weight = 0.0F;
        PathPoint target;
    };

    struct TrajectoryRisk
    {
        float collision_distance = std::numeric_limits<float>::infinity();
        float collision_time = std::numeric_limits<float>::infinity();
    };

    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void localization_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void steering_feedback_callback(
        const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    PathGuidance get_path_guidance();
    static double normalize_angle(double angle);
    bool prepare_scan(std::vector<float>& ranges, float range_max, float range_min,
                          float angle_min, float angle_increment, float& first_scan_angle);
    void get_range_differences(std::vector<float> &ranges, std::vector<float> &range_differences);
    void find_obstacle_edges(std::vector<float> &range_differences, float edge_threshold,
                             std::vector<int> &obstacle_edges);
    int get_cover_count(double radius, double distance, double angle_increment);
    void cover_scan_points(std::vector<float> &ranges, int direction, int count, int start_index);
    void expand_obstacles(std::vector<float> &ranges, std::vector<int> &obstacle_edges,
                          double vehicle_radius, double angle_increment);
    std::vector<float> filter_ranges(const std::vector<float>& ranges) const;
    TrajectoryRisk get_transition_trajectory_risk(
        const std::vector<std::pair<float, float>>& obstacle_points,
        float current_steering_angle, float target_steering_angle,
        float speed, float check_distance) const;
    float get_safe_distance(const std::vector<float>& ranges, size_t center_index,
                              size_t check_width, float safety_level) const;
    size_t find_target_index(const std::vector<float>& ranges, size_t check_width,
                             float safety_level,
                             float first_scan_angle, float angle_increment,
                             const PathGuidance& path_guidance,
                             float& target_distance, float& target_score,
                             bool& path_preference_applied) const;
    void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void publish_debug_markers(
        const std_msgs::msg::Header& header, float steering_angle,
        float target_distance, float collision_distance,
        const PathGuidance& path_guidance, float current_steering_angle,
        float collision_ttc, float braking_distance,
        bool emergency, bool braking_for_risk);

    float set_speed_from_distance(float distance, float steering_angle);
    float get_speed_increase_ratio(float distance) const;
    float limit_speed_change(
        float desired_speed, float distance, float steering_angle, bool emergency,
        uint64_t current_time, float& acceleration);
    float smooth_steering(float new_value);

    std::deque<float> steering_window;
    float steering_sum = 0.0;
    float current_speed = 0.0;
    uint64_t last_update_time = 0;

    // Keep a committed obstacle-avoidance side long enough for the vehicle's
    // steering actuator to follow it. Release thresholds are derived from the
    // common candidate clearance and direction hold time.
    int avoidance_direction = 0;
    int64_t avoidance_direction_start_time = 0;
    int64_t avoidance_clear_start_time = 0;

    std::vector<PathPoint> reference_path;
    std::string reference_path_frame;
    bool path_locked = false;
    nav_msgs::msg::Odometry latest_localization;
    rclcpp::Time latest_localization_receive_time{0, 0, RCL_ROS_TIME};
    bool has_localization = false;
    float filtered_odom_speed = 0.0F;
    std::deque<float> odom_speed_window;

    float latest_steering_feedback = 0.0F;
    float latest_speed_feedback = 0.0F;
    float last_published_steering = 0.0F;
    rclcpp::Time latest_steering_feedback_receive_time{0, 0, RCL_ROS_TIME};
    bool has_steering_feedback = false;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscriber;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        steering_feedback_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
};

#endif  // GAP_FOLLOW_HPP
