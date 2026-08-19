#include "gap_follow.hpp"
#include <algorithm>
#include <limits>
#include <sstream>
#include <utility>

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
    marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        this->get_parameter("debug_marker_topic").as_string(), qos);

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

std::vector<float> ReactiveGapFollow::predict_ranges_after_motion(
    const std::vector<float>& ranges,
    float first_scan_angle,
    float angle_increment,
    float range_max,
    float steering_angle,
    float travel_distance,
    float& predicted_yaw) const
{
    predicted_yaw = 0.0F;
    if (ranges.empty() || travel_distance <= 0.0F || angle_increment <= 0.0F)
    {
        return ranges;
    }

    const float wheel_base = static_cast<float>(
        this->get_parameter("wheel_base").as_double());
    if (wheel_base <= std::numeric_limits<float>::epsilon())
    {
        return ranges;
    }

    const float maximum_steering = static_cast<float>(
        this->get_parameter("gap_prediction_max_steering_angle").as_double()
        * M_PI / 180.0);
    const float prediction_steering = std::max(
        -maximum_steering, std::min(steering_angle, maximum_steering));
    const float curvature = std::tan(prediction_steering) / wheel_base;

    float base_translation_x = travel_distance;
    float base_translation_y = 0.0F;
    if (std::abs(curvature) > 1.0e-5F)
    {
        predicted_yaw = travel_distance * curvature;
        base_translation_x = std::sin(predicted_yaw) / curvature;
        base_translation_y = (1.0F - std::cos(predicted_yaw)) / curvature;
    }

    const float cosine = std::cos(predicted_yaw);
    const float sine = std::sin(predicted_yaw);
    const float lidar_offset_x = static_cast<float>(
        this->get_parameter("lidar_offset_x").as_double());
    const float lidar_offset_y = static_cast<float>(
        this->get_parameter("lidar_offset_y").as_double());
    const float translation_x
        = base_translation_x + cosine * lidar_offset_x - sine * lidar_offset_y
        - lidar_offset_x;
    const float translation_y
        = base_translation_y + sine * lidar_offset_x + cosine * lidar_offset_y
        - lidar_offset_y;
    std::vector<float> predicted_ranges(ranges.size(), range_max);

    for (size_t i = 0; i < ranges.size(); ++i)
    {
        const float range = ranges[i];
        if (!std::isfinite(range) || range <= 0.0F || range >= range_max * 0.99F)
        {
            continue;
        }

        const float angle = first_scan_angle + static_cast<float>(i) * angle_increment;
        const float shifted_x = range * std::cos(angle) - translation_x;
        const float shifted_y = range * std::sin(angle) - translation_y;

        // Rotate the static obstacle point into the predicted vehicle frame.
        const float future_x = cosine * shifted_x + sine * shifted_y;
        const float future_y = -sine * shifted_x + cosine * shifted_y;
        const float future_angle = std::atan2(future_y, future_x);
        const int future_index = static_cast<int>(std::round(
            (future_angle - first_scan_angle) / angle_increment));
        if (future_index < 0 || future_index >= static_cast<int>(ranges.size()))
        {
            continue;
        }

        const float future_range = std::hypot(future_x, future_y);
        predicted_ranges[static_cast<size_t>(future_index)] = std::min(
            predicted_ranges[static_cast<size_t>(future_index)], future_range);
    }

    return this->filter_ranges(predicted_ranges);
}

float ReactiveGapFollow::get_trajectory_collision_distance(
    const std::vector<float>& ranges,
    float first_scan_angle,
    float angle_increment,
    float range_max,
    float steering_angle) const
{
    const float check_distance = std::max(
        0.0F,
        static_cast<float>(this->get_parameter("trajectory_check_distance").as_double()));
    const float check_step = std::max(
        0.01F,
        static_cast<float>(this->get_parameter("trajectory_check_step").as_double()));
    const float collision_radius = static_cast<float>(
        this->get_parameter("vehicle_radius").as_double()
        + std::max(
            0.0,
            this->get_parameter("trajectory_safety_margin").as_double()));
    const float collision_radius_squared = collision_radius * collision_radius;

    std::vector<std::pair<float, float>> obstacle_points;
    obstacle_points.reserve(ranges.size());
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        const float range = ranges[i];
        if (!std::isfinite(range) || range <= 0.0F || range >= range_max * 0.99F)
        {
            continue;
        }
        const float angle = first_scan_angle + static_cast<float>(i) * angle_increment;
        obstacle_points.emplace_back(
            range * std::cos(angle), range * std::sin(angle));
    }

    const float wheel_base = static_cast<float>(
        this->get_parameter("wheel_base").as_double());
    if (wheel_base <= std::numeric_limits<float>::epsilon())
    {
        return 0.0F;
    }
    const float maximum_steering = static_cast<float>(
        this->get_parameter("gap_prediction_max_steering_angle").as_double()
        * M_PI / 180.0);
    const float checked_steering = std::max(
        -maximum_steering, std::min(steering_angle, maximum_steering));
    const float curvature = std::tan(checked_steering) / wheel_base;
    const float lidar_offset_x = static_cast<float>(
        this->get_parameter("lidar_offset_x").as_double());
    const float lidar_offset_y = static_cast<float>(
        this->get_parameter("lidar_offset_y").as_double());

    for (float travelled = 0.0F; travelled <= check_distance; travelled += check_step)
    {
        float vehicle_x = travelled - lidar_offset_x;
        float vehicle_y = -lidar_offset_y;
        if (std::abs(curvature) > 1.0e-5F)
        {
            const float yaw = travelled * curvature;
            vehicle_x = std::sin(yaw) / curvature - lidar_offset_x;
            vehicle_y = (1.0F - std::cos(yaw)) / curvature - lidar_offset_y;
        }

        for (const auto& obstacle : obstacle_points)
        {
            const float difference_x = obstacle.first - vehicle_x;
            const float difference_y = obstacle.second - vehicle_y;
            if (difference_x * difference_x + difference_y * difference_y
                <= collision_radius_squared)
            {
                return travelled;
            }
        }
    }

    return std::numeric_limits<float>::infinity();
}

void ReactiveGapFollow::publish_debug_markers(
    const std_msgs::msg::Header& header,
    float steering_angle,
    float target_distance,
    float collision_distance)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker target_marker;
    target_marker.header = header;
    target_marker.ns = "gap_follow";
    target_marker.id = 0;
    target_marker.type = visualization_msgs::msg::Marker::ARROW;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.scale.x = 0.035;
    target_marker.scale.y = 0.09;
    target_marker.scale.z = 0.12;
    target_marker.color.r = 0.1F;
    target_marker.color.g = 0.6F;
    target_marker.color.b = 1.0F;
    target_marker.color.a = 1.0F;
    geometry_msgs::msg::Point origin;
    origin.z = 0.05;
    geometry_msgs::msg::Point target;
    target.x = target_distance * std::cos(steering_angle);
    target.y = target_distance * std::sin(steering_angle);
    target.z = 0.05;
    target_marker.points.push_back(origin);
    target_marker.points.push_back(target);
    marker_array.markers.push_back(target_marker);

    const float wheel_base = static_cast<float>(
        this->get_parameter("wheel_base").as_double());
    const float maximum_steering = static_cast<float>(
        this->get_parameter("gap_prediction_max_steering_angle").as_double()
        * M_PI / 180.0);
    const float checked_steering = std::max(
        -maximum_steering, std::min(steering_angle, maximum_steering));
    const float curvature = wheel_base > std::numeric_limits<float>::epsilon()
        ? std::tan(checked_steering) / wheel_base
        : 0.0F;
    const float check_distance = static_cast<float>(
        this->get_parameter("trajectory_check_distance").as_double());
    const float check_step = std::max(
        0.01F,
        static_cast<float>(this->get_parameter("trajectory_check_step").as_double()));
    const float lidar_offset_x = static_cast<float>(
        this->get_parameter("lidar_offset_x").as_double());
    const float lidar_offset_y = static_cast<float>(
        this->get_parameter("lidar_offset_y").as_double());
    const float collision_radius = static_cast<float>(
        this->get_parameter("vehicle_radius").as_double()
        + this->get_parameter("trajectory_safety_margin").as_double());
    const bool emergency = collision_distance
        <= this->get_parameter("emergency_stop_distance").as_double();

    visualization_msgs::msg::Marker trajectory_marker;
    trajectory_marker.header = header;
    trajectory_marker.ns = "gap_follow";
    trajectory_marker.id = 1;
    trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker.scale.x = 0.04;
    trajectory_marker.color.r = emergency ? 1.0F : 0.1F;
    trajectory_marker.color.g = emergency ? 0.1F : 1.0F;
    trajectory_marker.color.b = 0.1F;
    trajectory_marker.color.a = 1.0F;

    visualization_msgs::msg::Marker footprint_marker;
    footprint_marker.header = header;
    footprint_marker.ns = "gap_follow";
    footprint_marker.id = 2;
    footprint_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    footprint_marker.action = visualization_msgs::msg::Marker::ADD;
    footprint_marker.scale.x = collision_radius * 2.0F;
    footprint_marker.scale.y = collision_radius * 2.0F;
    footprint_marker.scale.z = 0.02;
    footprint_marker.color.r = emergency ? 1.0F : 0.1F;
    footprint_marker.color.g = emergency ? 0.1F : 0.8F;
    footprint_marker.color.b = 0.1F;
    footprint_marker.color.a = 0.08F;

    for (float travelled = 0.0F; travelled <= check_distance; travelled += check_step)
    {
        geometry_msgs::msg::Point point;
        point.x = travelled - lidar_offset_x;
        point.y = -lidar_offset_y;
        point.z = 0.03;
        if (std::abs(curvature) > 1.0e-5F)
        {
            const float yaw = travelled * curvature;
            point.x = std::sin(yaw) / curvature - lidar_offset_x;
            point.y = (1.0F - std::cos(yaw)) / curvature - lidar_offset_y;
        }
        trajectory_marker.points.push_back(point);
        footprint_marker.points.push_back(point);
    }
    marker_array.markers.push_back(trajectory_marker);
    marker_array.markers.push_back(footprint_marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header = header;
    text_marker.ns = "gap_follow";
    text_marker.id = 3;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = 0.3;
    text_marker.pose.position.y = 0.7;
    text_marker.pose.position.z = 0.35;
    text_marker.scale.z = 0.18;
    text_marker.color.r = emergency ? 1.0F : 1.0F;
    text_marker.color.g = emergency ? 0.2F : 1.0F;
    text_marker.color.b = emergency ? 0.2F : 1.0F;
    text_marker.color.a = 1.0F;
    std::ostringstream status;
    status << (emergency ? "STOP" : "CLEAR") << "  steer="
           << steering_angle * 180.0F / static_cast<float>(M_PI) << " deg";
    text_marker.text = status.str();
    marker_array.markers.push_back(text_marker);

    marker_publisher->publish(marker_array);
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
                                             float angle_increment, float range_max, size_t check_width,
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

    // Normalize every score term to a comparable ~[0, 1] scale so center_preference
    // and gap_width_preference behave as real relative weights. Without this, the
    // raw meter-scale distance terms swamp the angle-based penalty and the car
    // always chases whichever direction has the single longest sight line.
    const float distance_scale = range_max > 1.0e-3F ? range_max : 1.0F;
    const float angle_scale = std::max(
        1.0e-3F,
        static_cast<float>(this->get_parameter("max_scan_angle").as_double() * M_PI / 180.0));

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
            = (1.0F - gap_width_preference) * (safe_distance / distance_scale)
            + gap_width_preference * (width_score / distance_scale)
            - center_preference * (std::abs(angle) / angle_scale);
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
    const std::vector<float> raw_ranges = ranges;
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
        scan_msg->range_max,
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
            scan_msg->range_max,
            path_check_width,
            safety_level,
            ignored_distance);
    }

    float target_angle = first_scan_angle + static_cast<float>(target_index) * angle_increment;

    // Re-evaluate the gap from a short predicted Ackermann pose. Bringing the
    // future target back into the current frame accounts for how the visible gap
    // shifts after the commanded turn without requiring a map or wall extraction.
    const float prediction_distance = std::max(
        0.0F,
        static_cast<float>(this->get_parameter("gap_prediction_distance").as_double()));
    const float prediction_weight = std::max(
        0.0F,
        std::min(
            1.0F,
            static_cast<float>(this->get_parameter("gap_prediction_weight").as_double())));
    if (prediction_distance > 0.0F && prediction_weight > 0.0F)
    {
        const std::vector<float>& prediction_source = using_narrow_gap
            ? ranges
            : preferred_ranges;
        float predicted_yaw = 0.0F;
        const std::vector<float> predicted_ranges = this->predict_ranges_after_motion(
            prediction_source,
            first_scan_angle,
            angle_increment,
            scan_msg->range_max,
            target_angle,
            prediction_distance,
            predicted_yaw);
        float predicted_target_distance = 0.0F;
        const size_t predicted_target_index = this->find_target_index(
            predicted_ranges,
            first_scan_angle,
            angle_increment,
            scan_msg->range_max,
            path_check_width,
            safety_level,
            predicted_target_distance);
        const float predicted_target_angle
            = first_scan_angle
            + static_cast<float>(predicted_target_index) * angle_increment;
        const float predicted_target_in_current_frame
            = predicted_yaw + predicted_target_angle;
        target_angle
            = (1.0F - prediction_weight) * target_angle
            + prediction_weight * predicted_target_in_current_frame;
    }

    target_angle = std::max(
        first_scan_angle,
        std::min(
            first_scan_angle + static_cast<float>(ranges.size() - 1) * angle_increment,
            target_angle));
    target_angle = this->smooth_steering(target_angle);

    const size_t commanded_index = static_cast<size_t>(std::max(
        0,
        std::min(
            static_cast<int>(ranges.size()) - 1,
            static_cast<int>(std::round(
                (target_angle - first_scan_angle) / angle_increment)))));

    // Always use the physical-footprint scan for speed and emergency decisions.
    // The extra gap margin must influence preference, not create a false stop.
    const float target_distance = this->get_safe_distance(
        ranges, commanded_index, path_check_width, safety_level);
    const int forward_index = std::max(
        0,
        std::min(
            static_cast<int>(ranges.size()) - 1,
            static_cast<int>(std::round(-first_scan_angle / angle_increment))));
    const float front_distance = this->get_safe_distance(
        ranges, static_cast<size_t>(forward_index), path_check_width, safety_level);

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
    const float collision_distance = this->get_trajectory_collision_distance(
        raw_ranges,
        first_scan_angle,
        angle_increment,
        scan_msg->range_max,
        target_angle);
    float acceleration = 0.0;
    new_msg.drive.speed = limit_speed_change(
        desired_speed,
        safe_distance,
        collision_distance,
        target_angle,
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
        this->publish_debug_markers(
            scan_msg->header, target_angle, target_distance, collision_distance);
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
    // The trajectory collision check in limit_speed_change owns the hard-stop
    // decision. A short straight-ahead LiDAR distance alone only reduces speed.
    desired_speed = std::max(
        desired_speed,
        static_cast<float>(this->get_parameter("minimum_crawl_speed").as_double()));
    return desired_speed;
}

float ReactiveGapFollow::get_speed_increase_ratio(float distance) const
{
    const double start = this->get_parameter("speed_increase_start").as_double();
    const double end = this->get_parameter("speed_increase_end").as_double();
    return std::max(0.0, std::min((distance - start) / (end - start), 1.0));
}

float ReactiveGapFollow::limit_speed_change(
    float desired_speed,
    float distance,
    float collision_distance,
    float steering_angle,
    uint64_t current_time,
    float& acceleration)
{
    float elapsed_time = this->get_parameter("initial_update_time").as_double();
    if (last_update_time != 0 && current_time > last_update_time)
    {
        elapsed_time = static_cast<float>(current_time - last_update_time) * 1.0e-9;
    }
    last_update_time = current_time;

    const float previous_speed = current_speed;
    const float minimum_crawl_speed = static_cast<float>(
        this->get_parameter("minimum_crawl_speed").as_double());
    const float max_deceleration = static_cast<float>(
        this->get_parameter("max_deceleration").as_double());

    // Decelerate hard toward a crawl speed instead of snapping to a full stop.
    // At speed 0 the Ackermann model cannot turn in place, so a hard stop next
    // to a wall would leave the car unable to steer itself back out.
    if (collision_distance <= this->get_parameter("emergency_stop_distance").as_double())
    {
        current_speed = std::max(
            minimum_crawl_speed, previous_speed - max_deceleration * elapsed_time);
    }
    else if (desired_speed > previous_speed)
    {
        const float min_acceleration = this->get_parameter("min_acceleration").as_double();
        // Taper the acceleration ceiling as the commanded steering angle grows,
        // so the car does not keep sprinting straight into a turn it is about
        // to have to take sharply.
        const float steering_reference = static_cast<float>(
            this->get_parameter("gap_prediction_max_steering_angle").as_double()
            * M_PI / 180.0);
        const float steering_ratio = steering_reference > 0.0F
            ? std::max(0.0F, 1.0F - std::abs(steering_angle) / steering_reference)
            : 1.0F;
        const float acceleration_limit = min_acceleration
            + (this->get_parameter("max_acceleration").as_double() - min_acceleration)
            * get_speed_increase_ratio(distance) * steering_ratio;
        current_speed = std::min(
            desired_speed, previous_speed + acceleration_limit * elapsed_time);
    }
    else
    {
        current_speed = std::max(
            desired_speed, previous_speed - max_deceleration * elapsed_time);
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
