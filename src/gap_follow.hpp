
#ifndef GAP_FOLLOW_HPP
#define GAP_FOLLOW_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

class ReactiveGapFollow : public rclcpp::Node
{
  public:
    ReactiveGapFollow();

  private:
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
    std::vector<float> predict_ranges_after_motion(
        const std::vector<float>& ranges, float first_scan_angle, float angle_increment,
        float range_max, float steering_angle, float travel_distance,
        float& predicted_yaw) const;
    float get_trajectory_collision_distance(
        const std::vector<float>& ranges, float first_scan_angle, float angle_increment,
        float range_max, float steering_angle) const;
    float get_safe_distance(const std::vector<float>& ranges, size_t center_index,
                              size_t check_width, float safety_level) const;
    size_t find_target_index(const std::vector<float>& ranges, float first_scan_angle,
                             float angle_increment, float range_max, size_t check_width,
                             float safety_level, float& target_distance) const;
    void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void publish_debug_markers(
        const std_msgs::msg::Header& header, float steering_angle,
        float target_distance, float collision_distance);

    float set_speed_from_distance(float distance, float steering_angle);
    float get_speed_increase_ratio(float distance) const;
    float limit_speed_change(
        float desired_speed, float distance, float collision_distance,
        float steering_angle, uint64_t current_time, float& acceleration);
    float smooth_steering(float new_value);

    std::deque<float> steering_window;
    float steering_sum = 0.0;
    float current_speed = 0.0;
    uint64_t last_update_time = 0;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
};

#endif  // GAP_FOLLOW_HPP
