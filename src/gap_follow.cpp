#include "gap_follow.hpp"
#include <algorithm>
#include <limits>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveGapFollow>());
    rclcpp::shutdown();
    return 0;
}

ReactiveGapFollow::ReactiveGapFollow():
    Node(
        "reactive_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
    const auto qos = static_cast<size_t>(this->get_parameter("default_qos").as_int());
    drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        this->get_parameter("drive_topic").as_string(), qos);
    target_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>(
        this->get_parameter("target_waypoint_topic").as_string(), qos);

    scan_subscriber
        = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("lidar_scan_topic").as_string(),
            qos,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                this->lidar_callback(msg);
            });
}

// Preprocess lidar points to remove invalid points
bool ReactiveGapFollow::prepare_scan(std::vector<float> &ranges, float range_max, float range_min,
                                         float angle_min, float angle_increment, float& first_scan_angle)
{
    if (ranges.empty() || angle_increment <= 0.0 || !std::isfinite(range_max) || range_max <= 0.0)
    {
        return false;
    }

    for (float& range : ranges)
    {
        if (!std::isfinite(range) || range > range_max)
        {
            range = range_max;
        }
        else if (range < range_min)
        {
            range = range_min;
        }
    }

    const double scan_angle
        = (this->get_parameter("max_scan_angle").as_double() / 180.0) * M_PI;
    int start_index = static_cast<int>(std::ceil((-scan_angle - angle_min) / angle_increment));
    int end_index = static_cast<int>(std::floor((scan_angle - angle_min) / angle_increment));
    const int last_index = static_cast<int>(ranges.size()) - 1;
    start_index = std::max(0, std::min(start_index, last_index));
    end_index = std::max(0, std::min(end_index, last_index));

    if (start_index > end_index)
    {
        return false;
    }

    first_scan_angle = angle_min + static_cast<float>(start_index) * angle_increment;
    ranges = std::vector<float>(ranges.begin() + start_index, ranges.begin() + end_index + 1);
    return !ranges.empty();
}

// Compute the difference between each points and the previous points
void ReactiveGapFollow::get_range_differences(std::vector<float> &ranges, std::vector<float> &range_differences)
{
    range_differences.push_back(0.0);
    for (size_t i = 1; i < ranges.size(); i++)
    {
        range_differences.push_back(std::abs(ranges[i] - ranges[i - 1]));
    }
}

// Find sharp range changes that indicate obstacle edges.
void ReactiveGapFollow::find_obstacle_edges(std::vector<float> &range_differences,
                                             float edge_threshold,
                                             std::vector<int> &obstacle_edges)
{
    for (size_t i = 0; i< range_differences.size(); i++)
    {
        if (range_differences[i] > edge_threshold)
        {
            obstacle_edges.push_back(i);
        }
    }
}

int ReactiveGapFollow::get_cover_count(double radius, double distance, double angle_increment)
{
    double angle = std::atan2(radius, distance);
    return std::ceil(angle/angle_increment);
}

// Cover a certain number of points when extending obstacle_edges
void ReactiveGapFollow::cover_scan_points(std::vector<float> &ranges,
                                           int direction,
                                           int count,
                                           int start_index)
{
    const double obstacle_distance = ranges[start_index];
    if (direction < 0)
    {
        for (int i = 0; i < count; i++)
        {
            const int next_index = start_index + 1 + i;
            if (next_index >= static_cast<int>(ranges.size())) break;
            if (ranges[next_index] > obstacle_distance)
            {
                ranges[next_index] = obstacle_distance;
            }
        }
    } else
    {
        for (int i = 0; i < count; i++)
        {
            const int next_index = start_index - 1 - i;
            if (next_index < 0) break;
            if (ranges[next_index] > obstacle_distance)
            {
                ranges[next_index] = obstacle_distance;
            }
        }
    }
}

void ReactiveGapFollow::expand_obstacles(std::vector<float> &ranges,
                                            std::vector<int> &obstacle_edges,
                                            double vehicle_radius,
                                            double angle_increment)
{
    for (size_t i = 0; i < obstacle_edges.size(); i++)
    {
        const int edge_index = obstacle_edges[i] - 1;
        const std::vector<double> edge_ranges(
            ranges.begin() + edge_index, ranges.begin() + edge_index + 2);
        const auto near_point = std::min_element(edge_ranges.begin(), edge_ranges.end());
        const auto far_point = std::max_element(edge_ranges.begin(), edge_ranges.end());
        const int near_offset = std::distance(edge_ranges.begin(), near_point);
        const int far_offset = std::distance(edge_ranges.begin(), far_point);
        const int near_index = edge_index + near_offset;
        const int far_index = edge_index + far_offset;
        const int count = this->get_cover_count(
            vehicle_radius, ranges[near_index], angle_increment);

        this->cover_scan_points(ranges, near_index - far_index, count, near_index);
    }
}

std::vector<float> ReactiveGapFollow::filter_ranges(const std::vector<float>& ranges) const
{
    const auto filter_window = static_cast<size_t>(
        this->get_parameter("scan_filter_window").as_int());
    if (ranges.empty() || filter_window <= 1)
    {
        return ranges;
    }

    const size_t half_window = filter_window / 2;
    std::vector<float> filtered(ranges.size());
    std::vector<float> window;
    window.reserve(filter_window);

    for (size_t i = 0; i < ranges.size(); ++i)
    {
        const size_t begin = i > half_window ? i - half_window : 0;
        const size_t end = std::min(ranges.size() - 1, i + half_window);
        window.assign(ranges.begin() + begin, ranges.begin() + end + 1);
        const size_t median_index = window.size() / 2;
        std::nth_element(window.begin(), window.begin() + median_index, window.end());
        filtered[i] = window[median_index];
    }

    return filtered;
}

float ReactiveGapFollow::get_safe_distance(const std::vector<float>& ranges, size_t center_index,
                                               size_t check_width, float safety_level) const
{
    if (ranges.empty()) {return 0.0;}

    center_index = std::min(center_index, ranges.size() - 1);
    const size_t begin = center_index > check_width ? center_index - check_width : 0;
    const size_t end = std::min(ranges.size() - 1, center_index + check_width);
    std::vector<float> window(ranges.begin() + begin, ranges.begin() + end + 1);

    const size_t selected_index = static_cast<size_t>(
        safety_level * static_cast<float>(window.size() - 1));
    std::nth_element(window.begin(), window.begin() + selected_index, window.end());
    return window[selected_index];
}

size_t ReactiveGapFollow::find_target_index(const std::vector<float>& ranges, float first_scan_angle,
                                             float angle_increment, size_t check_width,
                                             float safety_level,
                                             float& target_distance) const
{
    target_distance = 0.0;
    if (ranges.empty())
    {
        return 0;
    }

    size_t search_begin = check_width;
    size_t search_end = ranges.size() - 1;
    if (ranges.size() > 2 * check_width)
    {
        search_end -= check_width;
    }
    else
    {
        search_begin = 0;
    }

    size_t best_index = search_begin;
    float best_score = -std::numeric_limits<float>::infinity();
    const float center_preference = this->get_parameter("center_preference").as_double();
    const float gap_width_preference = std::max(
        0.0F,
        std::min(
            1.0F,
            static_cast<float>(this->get_parameter("gap_width_preference").as_double())));
    const size_t gap_width_check_width = static_cast<size_t>(
        std::ceil(
            (this->get_parameter("gap_width_check_angle").as_double() * M_PI / 180.0)
            / angle_increment));

    for (size_t i = search_begin; i <= search_end; ++i)
    {
        const float safe_distance = get_safe_distance(
            ranges, i, check_width, safety_level);

        // A narrow but deep opening has only a few long rays, whereas a wide gap
        // keeps the average range high around the candidate direction. This adds
        // a continuous width preference without a fixed gap-distance threshold.
        const size_t width_begin = i > gap_width_check_width
            ? i - gap_width_check_width
            : 0;
        const size_t width_end = std::min(
            ranges.size() - 1, i + gap_width_check_width);
        float range_sum = 0.0F;
        for (size_t j = width_begin; j <= width_end; ++j)
        {
            range_sum += ranges[j];
        }
        const float width_score = range_sum
            / static_cast<float>(width_end - width_begin + 1);

        const float angle = first_scan_angle + static_cast<float>(i) * angle_increment;
        const float score
            = (1.0F - gap_width_preference) * safe_distance
            + gap_width_preference * width_score
            - center_preference * std::abs(angle);
        if (score > best_score)
        {
            best_score = score;
            best_index = i;
            target_distance = safe_distance;
        }
    }

    return best_index;
}

void ReactiveGapFollow::lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    if (!scan_msg || scan_msg->ranges.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Invalid scan");
        return;
    }

    if (scan_msg->angle_increment <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid angle_increment");
        return;
    }
    std::vector<float>& ranges = scan_msg->ranges;
    const float angle_increment = scan_msg->angle_increment;

    float first_scan_angle = scan_msg->angle_min;
    if (!this->prepare_scan(
        ranges,
        scan_msg->range_max,
        scan_msg->range_min,
        scan_msg->angle_min,
        angle_increment,
        first_scan_angle))
    {
        RCLCPP_WARN(this->get_logger(), "Scan does not overlap the selected field of view");
        return;
    }
    std::vector<float> range_differences;
    std::vector<int> obstacle_edges;
    this->get_range_differences(ranges, range_differences);
    this->find_obstacle_edges(
        range_differences,
        this->get_parameter("obstacle_edge_threshold").as_double(),
        obstacle_edges);

    // Keep two versions of the scan. The preferred scan includes extra wall
    // clearance, while the physical scan only includes the vehicle footprint.
    // This lets the car fall back to a passable narrow gap instead of stopping
    // merely because the comfort margin does not fit.
    std::vector<float> preferred_ranges = ranges;
    this->expand_obstacles(
        ranges,
        obstacle_edges,
        this->get_parameter("vehicle_radius").as_double(),
        angle_increment);
    this->expand_obstacles(
        preferred_ranges,
        obstacle_edges,
        this->get_parameter("vehicle_radius").as_double()
            + std::max(0.0, this->get_parameter("gap_safety_margin").as_double()),
        angle_increment);

    // Remove isolated range spikes after obstacle expansion. A real obstacle edge has
    // already been widened above, so the median filter does not erase narrow obstacles.
    ranges = this->filter_ranges(ranges);
    preferred_ranges = this->filter_ranges(preferred_ranges);

    ackermann_msgs::msg::AckermannDriveStamped new_msg;

    const size_t path_check_width = static_cast<size_t>(
        std::ceil(
            (this->get_parameter("path_check_angle").as_double() * M_PI / 180.0)
            / angle_increment));
    const float safety_level = this->get_parameter("safety_level").as_double();
    float preferred_target_distance = 0.0;
    size_t target_index = this->find_target_index(
        preferred_ranges,
        first_scan_angle,
        angle_increment,
        path_check_width,
        safety_level,
        preferred_target_distance);

    const float fallback_distance = static_cast<float>(
        this->get_parameter("gap_fallback_distance").as_double());
    const bool using_narrow_gap = preferred_target_distance <= fallback_distance;
    if (using_narrow_gap)
    {
        float ignored_distance = 0.0F;
        target_index = this->find_target_index(
            ranges,
            first_scan_angle,
            angle_increment,
            path_check_width,
            safety_level,
            ignored_distance);
    }

    // Always use the physical-footprint scan for speed and emergency decisions.
    // The extra gap margin must influence preference, not create a false stop.
    const float target_distance = this->get_safe_distance(
        ranges, target_index, path_check_width, safety_level);
    const int forward_index = std::max(
        0,
        std::min(
            static_cast<int>(ranges.size()) - 1,
            static_cast<int>(std::round(-first_scan_angle / angle_increment))));
    const float front_distance = this->get_safe_distance(
        ranges, static_cast<size_t>(forward_index), path_check_width, safety_level);

    float target_angle = first_scan_angle + static_cast<float>(target_index) * angle_increment;
    target_angle = this->smooth_steering(target_angle);

    // A close front wall is relevant while driving almost straight. In a corner,
    // use the selected gap direction so a wall in front does not force a false stop.
    const float front_check_angle = static_cast<float>(
        this->get_parameter("front_safety_check_angle").as_double() * M_PI / 180.0);
    const float safe_distance = std::abs(target_angle) <= front_check_angle
        ? std::min(target_distance, front_distance)
        : target_distance;

    const auto command_time = this->now();
    new_msg.header.stamp = command_time;
    new_msg.header.frame_id = this->get_parameter("command_frame_id").as_string();
    new_msg.drive.steering_angle = target_angle;
    float desired_speed = set_speed_from_distance(safe_distance, target_angle);
    if (using_narrow_gap)
    {
        desired_speed = std::min(
            desired_speed,
            static_cast<float>(this->get_parameter("narrow_gap_max_speed").as_double()));
    }
    float acceleration = 0.0;
    new_msg.drive.speed = limit_speed_change(
        desired_speed,
        safe_distance,
        static_cast<uint64_t>(command_time.nanoseconds()),
        acceleration);
    new_msg.drive.acceleration = acceleration;

    drive_publisher->publish(new_msg);

    if (this->get_parameter("debug").as_bool())
    {
        geometry_msgs::msg::PointStamped target_waypoint_msg;
        target_waypoint_msg.header = scan_msg->header;
        target_waypoint_msg.point.x = target_distance * std::cos(target_angle);
        target_waypoint_msg.point.y = target_distance * std::sin(target_angle);
        target_publisher->publish(target_waypoint_msg);
    }
};

float ReactiveGapFollow::set_speed_from_distance(float distance, float steering_angle)
{
    float speed = distance * this->get_parameter("speed_factor").as_double();
    speed *= 1.0
        + (this->get_parameter("speed_increase_factor").as_double() - 1.0)
        * get_speed_increase_ratio(distance);

    const float turning_speed = std::sqrt(
        this->get_parameter("wheel_base").as_double()
        / std::sin(std::abs(steering_angle))
        * 9.81
        * this->get_parameter("friction").as_double());
    float desired_speed = std::min(
        std::min(speed, turning_speed),
        static_cast<float>(this->get_parameter("max_speed").as_double()));
    if (distance > this->get_parameter("emergency_stop_distance").as_double())
    {
        desired_speed = std::max(
            desired_speed,
            static_cast<float>(this->get_parameter("minimum_crawl_speed").as_double()));
    }
    return desired_speed;
}

float ReactiveGapFollow::get_speed_increase_ratio(float distance) const
{
    const double start = this->get_parameter("speed_increase_start").as_double();
    const double end = this->get_parameter("speed_increase_end").as_double();
    return std::max(0.0, std::min((distance - start) / (end - start), 1.0));
}

float ReactiveGapFollow::limit_speed_change(
    float desired_speed, float distance, uint64_t current_time, float& acceleration)
{
    float elapsed_time = this->get_parameter("initial_update_time").as_double();
    if (last_update_time != 0 && current_time > last_update_time)
    {
        elapsed_time = static_cast<float>(current_time - last_update_time) * 1.0e-9;
    }
    last_update_time = current_time;

    const float previous_speed = current_speed;

    // An obstacle inside the emergency margin bypasses the normal deceleration ramp.
    if (distance <= this->get_parameter("emergency_stop_distance").as_double())
    {
        current_speed = 0.0;
    }
    else if (desired_speed > previous_speed)
    {
        const float min_acceleration = this->get_parameter("min_acceleration").as_double();
        const float acceleration_limit = min_acceleration
            + (this->get_parameter("max_acceleration").as_double() - min_acceleration)
            * get_speed_increase_ratio(distance);
        current_speed = std::min(
            desired_speed, previous_speed + acceleration_limit * elapsed_time);
    }
    else
    {
        current_speed = std::max(
            desired_speed,
            previous_speed
                - static_cast<float>(this->get_parameter("max_deceleration").as_double())
                    * elapsed_time);
    }

    acceleration = (current_speed - previous_speed) / elapsed_time;
    return current_speed;
}

float ReactiveGapFollow::smooth_steering(float new_value)
{
    steering_window.push_back(new_value);
    steering_sum += new_value;
    if (steering_window.size()
        > static_cast<size_t>(this->get_parameter("steering_smooth_window").as_int()))
    {
        steering_sum -= steering_window.front();
        steering_window.pop_front();
    }

    return steering_sum / steering_window.size();
}
