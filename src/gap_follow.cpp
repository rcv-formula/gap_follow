#include "gap_follow.hpp"
#include <algorithm>
#include <limits>
#include <sstream>
#include <utility>

namespace
{
std::string normalized_frame(std::string frame)
{
    while (!frame.empty() && frame.front() == '/')
    {
        frame.erase(frame.begin());
    }
    return frame;
}
}  // namespace

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

    if (this->get_parameter("enable_path_guidance").as_bool())
    {
        rclcpp::QoS path_qos(rclcpp::KeepLast(1));
        path_qos.reliable();
        if (this->get_parameter("global_path_transient_local").as_bool())
        {
            path_qos.transient_local();
        }
        else
        {
            path_qos.durability_volatile();
        }
        global_path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
            this->get_parameter("global_path_topic").as_string(),
            path_qos,
            [this](const nav_msgs::msg::Path::SharedPtr msg)
            {
                this->global_path_callback(msg);
            });
        localization_subscriber
            = this->create_subscription<nav_msgs::msg::Odometry>(
                this->get_parameter("localization_topic").as_string(),
                qos,
                [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    this->localization_callback(msg);
                });
    }
}

double ReactiveGapFollow::normalize_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

void ReactiveGapFollow::global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (path_locked)
    {
        return;
    }
    if (!msg || msg->poses.size() < 2)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Ignoring global path with fewer than two poses");
        return;
    }

    std::string path_frame = msg->header.frame_id;
    if (path_frame.empty())
    {
        for (const auto& pose : msg->poses)
        {
            if (!pose.header.frame_id.empty())
            {
                path_frame = pose.header.frame_id;
                break;
            }
        }
    }
    if (path_frame.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Ignoring global path without a frame_id");
        return;
    }

    std::vector<PathPoint> received_path;
    received_path.reserve(msg->poses.size());
    for (const auto& pose : msg->poses)
    {
        const double x = pose.pose.position.x;
        const double y = pose.pose.position.y;
        if (!std::isfinite(x) || !std::isfinite(y))
        {
            continue;
        }
        if (!received_path.empty())
        {
            const double dx = x - received_path.back().x;
            const double dy = y - received_path.back().y;
            if (dx * dx + dy * dy < 1.0e-8)
            {
                continue;
            }
        }
        PathPoint point;
        point.x = x;
        point.y = y;
        received_path.push_back(point);
    }
    if (received_path.size() < 2)
    {
        RCLCPP_WARN(this->get_logger(), "Ignoring global path without two valid distinct poses");
        return;
    }

    reference_path = std::move(received_path);
    reference_path_frame = normalized_frame(path_frame);
    path_locked = true;
    RCLCPP_INFO(
        this->get_logger(),
        "Locked first global path with %zu poses in frame '%s'; later updates will be ignored",
        reference_path.size(), reference_path_frame.c_str());
}

void ReactiveGapFollow::localization_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }
    const auto& position = msg->pose.pose.position;
    const auto& orientation = msg->pose.pose.orientation;
    const double quaternion_norm = orientation.x * orientation.x
        + orientation.y * orientation.y + orientation.z * orientation.z
        + orientation.w * orientation.w;
    if (!std::isfinite(position.x) || !std::isfinite(position.y)
        || !std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-8)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Ignoring invalid localization pose");
        return;
    }

    latest_localization = *msg;
    latest_localization_receive_time = this->now();
    has_localization = true;
}

ReactiveGapFollow::PathGuidance ReactiveGapFollow::get_path_guidance()
{
    PathGuidance guidance;
    if (!this->get_parameter("enable_path_guidance").as_bool()
        || reference_path.size() < 2 || !has_localization)
    {
        return guidance;
    }

    const double timeout = std::max(
        0.0, this->get_parameter("localization_timeout").as_double());
    const double pose_age = (this->now() - latest_localization_receive_time).seconds();
    if (pose_age < 0.0 || pose_age > timeout)
    {
        return guidance;
    }

    const std::string& path_frame = reference_path_frame;
    const std::string pose_frame = normalized_frame(latest_localization.header.frame_id);
    if (pose_frame.empty() || path_frame != pose_frame)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Path frame '%s' differs from localization frame '%s'; path guidance disabled",
            path_frame.c_str(), pose_frame.c_str());
        return guidance;
    }

    const auto& pose = latest_localization.pose.pose;
    const double sin_yaw = 2.0 * (
        pose.orientation.w * pose.orientation.z
        + pose.orientation.x * pose.orientation.y);
    const double cos_yaw = 1.0 - 2.0 * (
        pose.orientation.y * pose.orientation.y
        + pose.orientation.z * pose.orientation.z);
    const double yaw = std::atan2(sin_yaw, cos_yaw);
    const bool closed_path = this->get_parameter("path_is_closed").as_bool();
    const size_t segment_count = closed_path
        ? reference_path.size() : reference_path.size() - 1;
    const double heading_search_weight = std::max(
        0.0, this->get_parameter("path_heading_search_weight").as_double());

    size_t nearest_segment = 0;
    double nearest_ratio = 0.0;
    double nearest_distance_squared = std::numeric_limits<double>::infinity();
    double best_metric = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < segment_count; ++i)
    {
        const PathPoint& start = reference_path[i];
        const PathPoint& end = reference_path[(i + 1) % reference_path.size()];
        const double segment_x = end.x - start.x;
        const double segment_y = end.y - start.y;
        const double length_squared = segment_x * segment_x + segment_y * segment_y;
        if (length_squared < 1.0e-10)
        {
            continue;
        }
        const double ratio = std::max(
            0.0, std::min(
                ((pose.position.x - start.x) * segment_x
                    + (pose.position.y - start.y) * segment_y) / length_squared,
                1.0));
        const double projected_x = start.x + ratio * segment_x;
        const double projected_y = start.y + ratio * segment_y;
        const double dx = pose.position.x - projected_x;
        const double dy = pose.position.y - projected_y;
        const double distance_squared = dx * dx + dy * dy;
        const double segment_yaw = std::atan2(segment_y, segment_x);
        const double heading_difference = normalize_angle(segment_yaw - yaw);
        const double metric = std::sqrt(distance_squared)
            + heading_search_weight * (1.0 - std::cos(heading_difference));
        if (metric < best_metric)
        {
            best_metric = metric;
            nearest_distance_squared = distance_squared;
            nearest_segment = i;
            nearest_ratio = ratio;
        }
    }

    if (!std::isfinite(best_metric))
    {
        return guidance;
    }

    const double lookahead = std::max(
        0.0, this->get_parameter("path_lookahead_distance").as_double());
    PathPoint target;
    const PathPoint& nearest_start = reference_path[nearest_segment];
    const PathPoint& nearest_end
        = reference_path[(nearest_segment + 1) % reference_path.size()];
    target.x = nearest_start.x + nearest_ratio * (nearest_end.x - nearest_start.x);
    target.y = nearest_start.y + nearest_ratio * (nearest_end.y - nearest_start.y);

    double remaining = lookahead;
    size_t segment = nearest_segment;
    double ratio = nearest_ratio;
    for (size_t visited = 0; visited <= segment_count && remaining > 0.0; ++visited)
    {
        const PathPoint& start = reference_path[segment];
        const PathPoint& end = reference_path[(segment + 1) % reference_path.size()];
        const double segment_x = end.x - start.x;
        const double segment_y = end.y - start.y;
        const double segment_length = std::hypot(segment_x, segment_y);
        const double available = segment_length * (1.0 - ratio);
        if (segment_length > 1.0e-8 && remaining <= available)
        {
            const double target_ratio = ratio + remaining / segment_length;
            target.x = start.x + target_ratio * segment_x;
            target.y = start.y + target_ratio * segment_y;
            remaining = 0.0;
            break;
        }
        target = end;
        remaining -= available;
        if (!closed_path && segment + 1 >= segment_count)
        {
            break;
        }
        segment = (segment + 1) % segment_count;
        ratio = 0.0;
    }

    const double global_target_angle = std::atan2(
        target.y - pose.position.y, target.x - pose.position.x);
    guidance.target_angle = static_cast<float>(normalize_angle(global_target_angle - yaw));
    guidance.cross_track_error = static_cast<float>(std::sqrt(nearest_distance_squared));
    guidance.target = target;

    const double max_guidance_angle = std::max(
        0.0, this->get_parameter("path_max_guidance_angle").as_double())
        * M_PI / 180.0;
    if (std::abs(guidance.target_angle) > max_guidance_angle)
    {
        return guidance;
    }

    const double rejoin_distance = std::max(
        0.0, this->get_parameter("path_rejoin_distance").as_double());
    guidance.score_weight = static_cast<float>(guidance.cross_track_error > rejoin_distance
        ? std::max(0.0, this->get_parameter("path_rejoin_weight").as_double())
        : std::max(0.0, this->get_parameter("path_guidance_weight").as_double()));
    guidance.active = guidance.score_weight > 0.0F;
    return guidance;
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
        this->get_parameter("max_steering_angle").as_double()
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
    float collision_distance,
    const PathGuidance& path_guidance)
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
        this->get_parameter("max_steering_angle").as_double()
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
    if (path_guidance.active)
    {
        status << "  path="
               << path_guidance.target_angle * 180.0F / static_cast<float>(M_PI)
               << " deg  error=" << path_guidance.cross_track_error << " m";
    }
    text_marker.text = status.str();
    marker_array.markers.push_back(text_marker);

    if (!reference_path.empty())
    {
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.stamp = header.stamp;
        path_marker.header.frame_id = reference_path_frame;
        path_marker.ns = "reference_path";
        path_marker.id = 10;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.04;
        path_marker.color.r = 0.9F;
        path_marker.color.g = 0.7F;
        path_marker.color.b = 0.1F;
        path_marker.color.a = 0.65F;
        for (const auto& path_point : reference_path)
        {
            geometry_msgs::msg::Point point;
            point.x = path_point.x;
            point.y = path_point.y;
            point.z = 0.02;
            path_marker.points.push_back(point);
        }
        if (this->get_parameter("path_is_closed").as_bool()
            && reference_path.size() > 1)
        {
            path_marker.points.push_back(path_marker.points.front());
        }
        marker_array.markers.push_back(path_marker);
    }

    if (path_guidance.active)
    {
        visualization_msgs::msg::Marker path_target_marker;
        path_target_marker.header.stamp = header.stamp;
        path_target_marker.header.frame_id = reference_path_frame;
        path_target_marker.ns = "reference_path";
        path_target_marker.id = 11;
        path_target_marker.type = visualization_msgs::msg::Marker::SPHERE;
        path_target_marker.action = visualization_msgs::msg::Marker::ADD;
        path_target_marker.pose.position.x = path_guidance.target.x;
        path_target_marker.pose.position.y = path_guidance.target.y;
        path_target_marker.pose.position.z = 0.08;
        path_target_marker.pose.orientation.w = 1.0;
        path_target_marker.scale.x = 0.18;
        path_target_marker.scale.y = 0.18;
        path_target_marker.scale.z = 0.18;
        path_target_marker.color.r = 1.0F;
        path_target_marker.color.g = 0.2F;
        path_target_marker.color.b = 0.8F;
        path_target_marker.color.a = 0.95F;
        marker_array.markers.push_back(path_target_marker);
    }

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

size_t ReactiveGapFollow::find_target_index(const std::vector<float>& ranges,
                                             size_t check_width,
                                             float safety_level,
                                             float first_scan_angle,
                                             float angle_increment,
                                             const PathGuidance& path_guidance,
                                             float& target_distance,
                                             float& target_score,
                                             bool& path_preference_applied) const
{
    target_distance = 0.0;
    target_score = 0.0F;
    path_preference_applied = false;
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

    struct Candidate
    {
        float safe_distance = 0.0F;
        float base_score = -std::numeric_limits<float>::infinity();
    };
    std::vector<Candidate> candidates(ranges.size());
    size_t lidar_best_index = search_begin;
    float lidar_best_score = -std::numeric_limits<float>::infinity();
    const float gap_width_preference = std::max(
        0.0F,
        std::min(
            1.0F,
            static_cast<float>(
                this->get_parameter("gap_width_preference").as_double())));
    // Use the same angular neighbourhood for safe distance and gap width.
    const size_t gap_width_check_width = check_width;

    // Preserve useful metre-scale differences inside the track, but do not let
    // a very deep side opening keep gaining score.
    const float distance_cap = std::max(
        1.0e-3F,
        static_cast<float>(
            this->get_parameter("gap_score_distance_cap").as_double()));
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
            // Cap each ray before averaging. Otherwise one max-range return can
            // make a mostly narrow direction appear to be a wide open corridor.
            range_sum += std::min(ranges[j], distance_cap);
        }
        const float width_score = range_sum
            / static_cast<float>(width_end - width_begin + 1);
        const float capped_safe_distance = std::min(safe_distance, distance_cap);

        const float base_score
            = (1.0F - gap_width_preference) * capped_safe_distance
            + gap_width_preference * width_score;
        candidates[i].safe_distance = safe_distance;
        candidates[i].base_score = base_score;
        if (base_score > lidar_best_score)
        {
            lidar_best_score = base_score;
            lidar_best_index = i;
        }
    }

    // Path is only a tie-breaker among candidates that are already comparable
    // to the lidar-only best gap. It can never make a blocked/narrow direction
    // eligible merely because the global path points that way.
    const float minimum_clearance = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("path_candidate_min_clearance").as_double()));
    const float minimum_score_ratio = std::max(
        0.0F,
        std::min(
            1.0F,
            static_cast<float>(
                this->get_parameter("path_candidate_min_score_ratio").as_double())));
    const float minimum_clearance_ratio = std::max(
        0.0F,
        std::min(
            1.0F,
            static_cast<float>(
                this->get_parameter("path_candidate_min_clearance_ratio").as_double())));
    const float lidar_best_clearance = candidates[lidar_best_index].safe_distance;

    size_t best_index = lidar_best_index;
    float best_score = -std::numeric_limits<float>::infinity();
    for (size_t i = search_begin; i <= search_end; ++i)
    {
        float score = candidates[i].base_score;
        const bool path_safe_candidate = path_guidance.active
            && candidates[i].safe_distance >= minimum_clearance
            && candidates[i].safe_distance
                >= lidar_best_clearance * minimum_clearance_ratio
            && candidates[i].base_score >= lidar_best_score * minimum_score_ratio;
        if (path_safe_candidate)
        {
            const float candidate_angle
                = first_scan_angle + static_cast<float>(i) * angle_increment;
            const float difference = static_cast<float>(normalize_angle(
                candidate_angle - path_guidance.target_angle));
            const float alignment_sigma = std::max(
                1.0e-3F,
                static_cast<float>(
                    this->get_parameter("path_alignment_sigma").as_double()
                    * M_PI / 180.0));
            const float normalized_difference = difference / alignment_sigma;
            const float alignment = std::exp(
                -0.5F * normalized_difference * normalized_difference);
            score += path_guidance.score_weight * alignment;
        }
        if (score > best_score)
        {
            best_score = score;
            best_index = i;
            target_distance = candidates[i].safe_distance;
            path_preference_applied = path_safe_candidate;
        }
    }

    target_score = best_score;
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

    const float path_check_angle = static_cast<float>(
        this->get_parameter("path_check_angle").as_double() * M_PI / 180.0);
    const size_t path_check_width = static_cast<size_t>(std::ceil(
        path_check_angle / angle_increment));
    const float safety_level = this->get_parameter("safety_level").as_double();
    const PathGuidance path_guidance = this->get_path_guidance();
    float preferred_target_distance = 0.0;
    float target_score = 0.0F;
    bool selected_path_preference_applied = false;
    size_t target_index = this->find_target_index(
        preferred_ranges,
        path_check_width,
        safety_level,
        first_scan_angle,
        angle_increment,
        path_guidance,
        preferred_target_distance,
        target_score,
        selected_path_preference_applied);

    const float fallback_distance = static_cast<float>(
        this->get_parameter("gap_fallback_distance").as_double());
    const bool using_narrow_gap = preferred_target_distance <= fallback_distance;
    float selected_target_distance = preferred_target_distance;
    if (using_narrow_gap)
    {
        target_index = this->find_target_index(
            ranges,
            path_check_width,
            safety_level,
            first_scan_angle,
            angle_increment,
            path_guidance,
            selected_target_distance,
            target_score,
            selected_path_preference_applied);
    }

    float target_angle = first_scan_angle + static_cast<float>(target_index) * angle_increment;
    const auto index_for_angle = [&](float angle)
    {
        return static_cast<size_t>(std::max(
            0,
            std::min(
                static_cast<int>(ranges.size()) - 1,
                static_cast<int>(std::round(
                    (angle - first_scan_angle) / angle_increment)))));
    };
    const size_t forward_index = index_for_angle(0.0F);
    const auto clearance_for_angle = [&](float angle)
    {
        return this->get_safe_distance(
            ranges,
            index_for_angle(angle),
            path_check_width,
            safety_level);
    };
    const float maximum_steering_angle = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("max_steering_angle").as_double()
            * M_PI / 180.0));
    const auto clamp_steering = [&](float angle)
    {
        return std::max(
            -maximum_steering_angle, std::min(angle, maximum_steering_angle));
    };
    // Use the same physical steering range for the command and collision check.
    target_angle = clamp_steering(target_angle);

    // At a fork-like opening, a dead-end pocket can be slightly deeper than the
    // real track while both sides are still physically passable. Do not commit
    // hard until one side has a meaningful score advantage. Keeping the command
    // inside path_check_angle preserves enough room to take the other branch once
    // its opening becomes visible.
    float opposite_target_distance = 0.0F;
    float opposite_target_score = 0.0F;
    float opposite_target_angle = 0.0F;
    bool limiting_ambiguous_gap = false;
    const float ambiguous_gap_score_margin = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("ambiguous_gap_score_margin").as_double()));
    const float ambiguous_gap_opposite_weight = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("ambiguous_gap_opposite_weight").as_double()));
    if (std::abs(target_angle) > path_check_angle)
    {
        const std::vector<float>& selection_ranges = using_narrow_gap
            ? ranges
            : preferred_ranges;
        std::vector<float> opposite_ranges = selection_ranges;
        if (target_angle > 0.0F)
        {
            std::fill(
                opposite_ranges.begin() + forward_index,
                opposite_ranges.end(),
                0.0F);
        }
        else
        {
            std::fill(
                opposite_ranges.begin(),
                opposite_ranges.begin() + forward_index + 1,
                0.0F);
        }

        bool opposite_path_preference_applied = false;
        const size_t opposite_target_index = this->find_target_index(
            opposite_ranges,
            path_check_width,
            safety_level,
            first_scan_angle,
            angle_increment,
            path_guidance,
            opposite_target_distance,
            opposite_target_score,
            opposite_path_preference_applied);
        opposite_target_angle = clamp_steering(
            first_scan_angle
            + static_cast<float>(opposite_target_index) * angle_increment);

        // When both directions passed the lidar safety gate, a valid reference
        // path is the intended tie-breaker. Do not let the legacy ambiguous-fork
        // neutralization average that resolved choice back toward the other side.
        // If the path-side candidate was unsafe, it never received a path bonus,
        // so the selected gap will not satisfy this directional comparison.
        const float selected_path_difference = static_cast<float>(std::abs(
            normalize_angle(target_angle - path_guidance.target_angle)));
        const float opposite_path_difference = static_cast<float>(std::abs(
            normalize_angle(opposite_target_angle - path_guidance.target_angle)));
        const bool path_resolves_ambiguity = selected_path_preference_applied
            && selected_path_difference + angle_increment < opposite_path_difference;

        limiting_ambiguous_gap
            = selected_target_distance > fallback_distance
            && opposite_target_distance > fallback_distance
            && target_score
                <= opposite_target_score + ambiguous_gap_score_margin
            && !path_resolves_ambiguity;
        if (limiting_ambiguous_gap)
        {
            const float lateral_opening_angle = std::max(
                path_check_angle,
                maximum_steering_angle - path_check_angle);
            if (std::abs(opposite_target_angle) > std::abs(target_angle)
                && std::abs(opposite_target_angle)
                    >= lateral_opening_angle)
            {
                // A newly visible branch at a larger bearing can be the real
                // corner hidden behind an inner wall. Let it pull the command
                // across early, before the currently deeper pocket wins enough
                // lateral space to make that turn impossible.
                const float weighted_opposite_score
                    = opposite_target_score * ambiguous_gap_opposite_weight;
                const float combined_score
                    = target_score + weighted_opposite_score;
                const float balanced_angle = combined_score
                        > std::numeric_limits<float>::epsilon()
                    ? (target_angle * target_score
                        + opposite_target_angle * weighted_opposite_score)
                        / combined_score
                    : 0.0F;
                target_angle = std::max(
                    -path_check_angle,
                    std::min(balanced_angle, path_check_angle));
            }
            else
            {
                // In an ordinary corner the selected branch is already the more
                // lateral one. Preserve its direction and only postpone the hard
                // commitment until the other side stops being competitive.
                target_angle = std::copysign(
                    std::min(std::abs(target_angle), path_check_angle),
                    target_angle);
            }
        }
    }

    // A deep gap can still end in a corner. If the selected turn is physically
    // blocked within the current trajectory horizon, compare it with the best
    // gap on the opposite side and switch only when that path reaches farther.
    bool using_trajectory_fallback = false;
    if (std::abs(target_angle) > path_check_angle)
    {
        const float selected_collision_distance
            = this->get_trajectory_collision_distance(
                raw_ranges,
                first_scan_angle,
                angle_increment,
                scan_msg->range_max,
                target_angle);
        const float fallback_switch_distance = 0.5F * static_cast<float>(
            this->get_parameter("emergency_stop_distance").as_double()
            + this->get_parameter("trajectory_check_distance").as_double());
        if (std::isfinite(selected_collision_distance)
            && selected_collision_distance <= fallback_switch_distance)
        {
            const float opposite_collision_distance
                = this->get_trajectory_collision_distance(
                    raw_ranges,
                    first_scan_angle,
                    angle_increment,
                    scan_msg->range_max,
                    opposite_target_angle);
            const float minimum_improvement = static_cast<float>(
                this->get_parameter("trajectory_check_step").as_double());
            if (opposite_target_distance > 0.0F
                && (!std::isfinite(opposite_collision_distance)
                    || opposite_collision_distance
                        > selected_collision_distance + minimum_improvement))
            {
                target_angle = opposite_target_angle;
                using_trajectory_fallback = true;
            }
        }
    }

    target_angle = std::max(
        first_scan_angle,
        std::min(
            first_scan_angle + static_cast<float>(ranges.size() - 1) * angle_increment,
            target_angle));
    if (using_trajectory_fallback || limiting_ambiguous_gap)
    {
        // Do not let an old hard turn remain in the smoothing window and consume
        // the maneuvering room that this decision is trying to preserve.
        steering_window.clear();
        steering_sum = 0.0F;
    }
    const float unsmoothed_target_angle = target_angle;
    const float smoothed_target_angle = this->smooth_steering(target_angle);
    const float smoothing_required_clearance = std::min(
        clearance_for_angle(unsmoothed_target_angle), fallback_distance);
    // Temporal smoothing is also an angular blend, so apply the same guard.
    target_angle = clearance_for_angle(smoothed_target_angle)
        >= smoothing_required_clearance
        ? smoothed_target_angle
        : unsmoothed_target_angle;
    target_angle = clamp_steering(target_angle);

    // Before accepting an emergency crawl, search the remaining steering range
    // for a trajectory that clears the obstacle. This preserves normal
    // distance-based speed when steering alone is sufficient, while keeping the
    // original crawl fallback when every reachable trajectory is still blocked.
    float collision_distance = this->get_trajectory_collision_distance(
        raw_ranges,
        first_scan_angle,
        angle_increment,
        scan_msg->range_max,
        target_angle);
    bool using_steering_avoidance = false;
    if (this->get_parameter("enable_steering_before_crawl").as_bool()
        && collision_distance
            <= this->get_parameter("emergency_stop_distance").as_double())
    {
        const float scan_last_angle = first_scan_angle
            + static_cast<float>(ranges.size() - 1) * angle_increment;
        const float search_min_angle = std::max(
            first_scan_angle, -maximum_steering_angle);
        const float search_max_angle = std::min(
            scan_last_angle, maximum_steering_angle);
        const float search_step = std::max(
            angle_increment,
            static_cast<float>(
                this->get_parameter("avoidance_steering_step").as_double()
                * M_PI / 180.0));
        const float minimum_clearance = std::max(
            0.0F,
            static_cast<float>(
                this->get_parameter("avoidance_min_clearance").as_double()));
        const float required_collision_distance = std::max(
            static_cast<float>(
                this->get_parameter("emergency_stop_distance").as_double()),
            static_cast<float>(
                this->get_parameter("avoidance_collision_free_distance").as_double()));
        const float steering_change_penalty = std::max(
            0.0F,
            static_cast<float>(
                this->get_parameter("avoidance_steering_change_penalty").as_double()));
        const float distance_cap = std::max(
            1.0e-3F,
            static_cast<float>(
                this->get_parameter("gap_score_distance_cap").as_double()));

        float best_avoidance_score = -std::numeric_limits<float>::infinity();
        float best_avoidance_angle = target_angle;
        float best_avoidance_collision_distance = collision_distance;
        for (float candidate_angle = search_min_angle;
            candidate_angle <= search_max_angle + 0.5F * search_step;
            candidate_angle += search_step)
        {
            const float checked_angle = std::min(candidate_angle, search_max_angle);
            const float candidate_clearance = clearance_for_angle(checked_angle);
            if (candidate_clearance < minimum_clearance)
            {
                continue;
            }
            const float candidate_collision_distance
                = this->get_trajectory_collision_distance(
                    raw_ranges,
                    first_scan_angle,
                    angle_increment,
                    scan_msg->range_max,
                    checked_angle);
            if (std::isfinite(candidate_collision_distance)
                && candidate_collision_distance < required_collision_distance)
            {
                continue;
            }

            const float candidate_score
                = std::min(candidate_clearance, distance_cap)
                - steering_change_penalty * std::abs(checked_angle - target_angle);
            if (candidate_score > best_avoidance_score)
            {
                best_avoidance_score = candidate_score;
                best_avoidance_angle = checked_angle;
                best_avoidance_collision_distance = candidate_collision_distance;
            }
        }

        if (std::isfinite(best_avoidance_score))
        {
            target_angle = best_avoidance_angle;
            collision_distance = best_avoidance_collision_distance;
            using_steering_avoidance = true;
            // Do not blend the safe escape steering with older commands that
            // point toward the collision that triggered this search.
            steering_window.clear();
            steering_sum = 0.0F;
            target_angle = this->smooth_steering(target_angle);
        }
    }

    const size_t commanded_index = index_for_angle(target_angle);

    // Always use the physical-footprint scan for speed and emergency decisions.
    // The extra gap margin must influence preference, not create a false stop.
    const float target_distance = this->get_safe_distance(
        ranges, commanded_index, path_check_width, safety_level);
    const float front_distance = this->get_safe_distance(
        ranges, forward_index, path_check_width, safety_level);

    // A close front wall is relevant while driving almost straight. In a corner,
    // use the selected gap direction so a wall in front does not force a false stop.
    const float safe_distance = using_steering_avoidance
        ? target_distance
        : (std::abs(target_angle) <= path_check_angle
            ? std::min(target_distance, front_distance)
            : target_distance);

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
            scan_msg->header, target_angle, target_distance, collision_distance,
            path_guidance);
    }

    if (using_steering_avoidance)
    {
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "Avoided emergency crawl with steering %.1f deg",
            target_angle * 180.0F / static_cast<float>(M_PI));
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
    if (collision_distance
        <= this->get_parameter("emergency_stop_distance").as_double())
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
            this->get_parameter("max_steering_angle").as_double()
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
