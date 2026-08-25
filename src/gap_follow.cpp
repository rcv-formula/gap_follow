#include "gap_follow.hpp"
#include <algorithm>
#include <array>
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

// Robust command-to-curvature surface fitted from all seven available
// test_13--test_29 real-car bags.  The steering value is the Ackermann command
// seen by the servo bridge, not a measured tire angle, so a bicycle tan() model
// substantially over-predicts the turn at 4--6 m/s.  Keep this calibration in
// one place and interpolate it for collision prediction and turn-speed limits.
constexpr std::array<float, 9> kCurvatureCalibrationSpeeds = {
    1.75F, 2.25F, 2.75F, 3.25F, 3.75F,
    4.25F, 4.75F, 5.25F, 5.75F};
constexpr std::array<float, 8> kCurvatureCalibrationCommandsDeg = {
    0.0F, 7.5F, 12.5F, 17.5F, 22.5F, 27.5F, 35.0F, 38.57F};
constexpr std::array<std::array<float, 8>, 9> kMedianCurvature = {{
    {{0.0F, 0.2483F, 0.4413F, 0.5555F, 0.6484F, 0.8070F, 0.9473F, 1.0000F}},
    {{0.0F, 0.2613F, 0.3865F, 0.4551F, 0.5812F, 0.6607F, 0.7588F, 0.8244F}},
    {{0.0F, 0.2153F, 0.3182F, 0.3869F, 0.4953F, 0.5303F, 0.6432F, 0.6665F}},
    {{0.0F, 0.2035F, 0.2377F, 0.3231F, 0.3698F, 0.4130F, 0.4491F, 0.4928F}},
    {{0.0F, 0.1682F, 0.2194F, 0.2398F, 0.2900F, 0.3754F, 0.3904F, 0.4373F}},
    {{0.0F, 0.1281F, 0.1334F, 0.1620F, 0.2524F, 0.2904F, 0.3058F, 0.3856F}},
    {{0.0F, 0.0938F, 0.1027F, 0.1339F, 0.1971F, 0.2374F, 0.2374F, 0.2374F}},
    {{0.0F, 0.0967F, 0.1074F, 0.1249F, 0.1660F, 0.1674F, 0.1674F, 0.1954F}},
    {{0.0F, 0.0928F, 0.1037F, 0.1216F, 0.1327F, 0.1414F, 0.1414F, 0.1414F}},
}};
constexpr float kPositiveCurvatureScale = 1.115F;
constexpr float kNegativeCurvatureScale = 0.841F;

template <size_t N>
std::pair<size_t, float> interpolation_bracket(
    const std::array<float, N>& values, float query)
{
    if (query <= values.front())
    {
        return {0, 0.0F};
    }
    if (query >= values.back())
    {
        return {N - 2, 1.0F};
    }
    for (size_t i = 0; i + 1 < N; ++i)
    {
        if (query <= values[i + 1])
        {
            return {
                i,
                (query - values[i]) / (values[i + 1] - values[i])};
        }
    }
    return {N - 2, 1.0F};
}

float interpolate(float lower, float upper, float ratio)
{
    return lower + ratio * (upper - lower);
}

float calibrated_friction_coefficient(float low_speed_friction, float speed)
{
    constexpr float transition_start = 3.5F;
    constexpr float transition_end = 4.5F;
    constexpr float high_speed_ratio = 0.837F;
    const float checked_friction = std::max(0.0F, low_speed_friction);
    const float checked_speed = std::abs(speed);
    if (checked_speed <= transition_start)
    {
        return checked_friction;
    }
    if (checked_speed >= transition_end)
    {
        return checked_friction * high_speed_ratio;
    }
    const float ratio
        = (checked_speed - transition_start) / (transition_end - transition_start);
    const float smooth_ratio = ratio * ratio * ratio
        * (ratio * (ratio * 6.0F - 15.0F) + 10.0F);
    return interpolate(
        checked_friction,
        checked_friction * high_speed_ratio,
        smooth_ratio);
}

float modeled_steering_during_transition(
    float current_steering,
    float target_steering,
    float elapsed_time,
    float configured_response_time,
    float maximum_steering)
{
    const float response_time = std::max(1.0e-3F, configured_response_time);
    // Seven-bag actuator fit used by the simulator: about 0.02 s transport
    // delay and 0.14 s first-order response for a 0.20 s configured response.
    // Deriving the constants from the existing single parameter avoids adding
    // another group of tuning knobs.
    const float transport_delay = 0.10F * response_time;
    const float time_constant = std::max(1.0e-3F, 0.70F * response_time);
    if (elapsed_time <= transport_delay)
    {
        return current_steering;
    }
    const float response_elapsed = elapsed_time - transport_delay;
    const float first_order_ratio
        = 1.0F - std::exp(-response_elapsed / time_constant);
    const float first_order_delta
        = first_order_ratio * (target_steering - current_steering);
    const float steering_rate_limit = std::max(
        1.0e-3F, maximum_steering / time_constant);
    const float maximum_delta = steering_rate_limit * response_elapsed;
    const float limited_delta = std::max(
        -maximum_delta, std::min(first_order_delta, maximum_delta));
    return current_steering + limited_delta;
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
        "gap_follow_ver2",
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

    localization_subscriber
        = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("localization_topic").as_string(),
            qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                this->localization_callback(msg);
            });

    const std::string speed_odometry_topic
        = this->get_parameter("speed_odometry_topic").as_string();
    if (!speed_odometry_topic.empty())
    {
        speed_odometry_subscriber
            = this->create_subscription<nav_msgs::msg::Odometry>(
                speed_odometry_topic,
                qos,
                [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    this->speed_odometry_callback(msg);
                });
    }

    const std::string steering_feedback_topic
        = this->get_parameter("steering_feedback_topic").as_string();
    if (!steering_feedback_topic.empty())
    {
        steering_feedback_subscriber
            = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
                steering_feedback_topic,
                qos,
                [this](
                    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
                {
                    this->steering_feedback_callback(msg);
                });
    }

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
        const double speed = pose.pose.position.z;
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
                if (std::isfinite(speed) && speed >= 0.0)
                {
                    received_path.back().speed = speed;
                }
                continue;
            }
        }
        PathPoint point;
        point.x = x;
        point.y = y;
        if (std::isfinite(speed) && speed >= 0.0)
        {
            point.speed = speed;
        }
        received_path.push_back(point);
    }
    if (received_path.size() < 2)
    {
        RCLCPP_WARN(this->get_logger(), "Ignoring global path without two valid distinct poses");
        return;
    }

    reference_path = std::move(received_path);
    const float path_speed_scale = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("path_speed_scale").as_double()));
    reference_path_max_speed = -std::numeric_limits<float>::infinity();
    for (const auto& point : reference_path)
    {
        if (std::isfinite(point.speed))
        {
            reference_path_max_speed = std::max(
                reference_path_max_speed,
                path_speed_scale * static_cast<float>(point.speed));
        }
    }
    if (!std::isfinite(reference_path_max_speed))
    {
        reference_path_max_speed = std::numeric_limits<float>::infinity();
    }
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

void ReactiveGapFollow::speed_odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!msg || !std::isfinite(msg->twist.twist.linear.x))
    {
        return;
    }

    const float maximum_vehicle_speed_measurement = 1.25F * std::max(
        0.1F,
        static_cast<float>(this->get_parameter("max_speed").as_double()));
    const float vehicle_speed = std::min(
        maximum_vehicle_speed_measurement,
        this->wheel_speed_to_vehicle_speed(
            static_cast<float>(std::abs(msg->twist.twist.linear.x))));
    // This value drives TTC and the speed-dependent vehicle model. Reusing the
    // 11-sample LiDAR median window here delayed fast acceleration by several
    // m/s, so collision checks were performed at a speed the car had already
    // exceeded. Odometry is already a state estimate; use its latest calibrated
    // value and keep scan denoising confined to LaserScan data.
    filtered_vehicle_speed = vehicle_speed;
    latest_speed_odometry_receive_time = this->now();
    has_speed_odometry = true;
}

void ReactiveGapFollow::steering_feedback_callback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    if (!msg || !std::isfinite(msg->drive.steering_angle)
        || !std::isfinite(msg->drive.speed))
    {
        return;
    }
    latest_steering_feedback = msg->drive.steering_angle;
    latest_vehicle_speed_feedback = this->wheel_speed_to_vehicle_speed(
        std::abs(msg->drive.speed));
    latest_steering_feedback_receive_time = this->now();
    has_steering_feedback = true;
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
        0.0, this->get_parameter("odom_timeout").as_double());
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
    if (std::isfinite(nearest_start.speed) && std::isfinite(nearest_end.speed))
    {
        target.speed = nearest_start.speed
            + nearest_ratio * (nearest_end.speed - nearest_start.speed);
    }

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
            if (std::isfinite(start.speed) && std::isfinite(end.speed))
            {
                target.speed = start.speed
                    + target_ratio * (end.speed - start.speed);
            }
            else
            {
                target.speed = std::numeric_limits<double>::quiet_NaN();
            }
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
    guidance.available = true;
    if (std::isfinite(target.speed) && target.speed >= 0.0)
    {
        const float path_speed_scale = std::max(
            0.0F,
            static_cast<float>(
                this->get_parameter("path_speed_scale").as_double()));
        guidance.target_speed
            = path_speed_scale * static_cast<float>(target.speed);
        guidance.speed_available = true;
    }

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
                                            double half_vehicle_width,
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
            half_vehicle_width, ranges[near_index], angle_increment);

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

ReactiveGapFollow::TrajectoryRisk ReactiveGapFollow::get_transition_trajectory_risk(
    const std::vector<std::pair<float, float>>& obstacle_points,
    float current_steering_angle,
    float target_steering_angle,
    float vehicle_speed,
    float check_distance) const
{
    const float check_step = std::max(
        0.01F,
        static_cast<float>(this->get_parameter("trajectory_check_step").as_double()));
    const float safety_margin = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("trajectory_safety_margin").as_double()));
    const float half_vehicle_length = 0.5F * std::max(
        0.0F,
        static_cast<float>(this->get_parameter("vehicle_length").as_double()))
        + safety_margin;
    const float half_vehicle_width = 0.5F * std::max(
        0.0F,
        static_cast<float>(this->get_parameter("vehicle_width").as_double()))
        + safety_margin;

    const float maximum_steering = static_cast<float>(
        this->get_parameter("max_steering_angle").as_double()
        * M_PI / 180.0);
    const float checked_current_steering = std::max(
        -maximum_steering, std::min(current_steering_angle, maximum_steering));
    const float checked_target_steering = std::max(
        -maximum_steering, std::min(target_steering_angle, maximum_steering));
    const float lidar_offset_x = static_cast<float>(
        this->get_parameter("lidar_offset_x").as_double());
    const float lidar_offset_y = static_cast<float>(
        this->get_parameter("lidar_offset_y").as_double());
    const float prediction_speed = std::max(0.05F, std::abs(vehicle_speed));
    const float transition_time = std::max(
        1.0e-3F,
        static_cast<float>(
            this->get_parameter("steering_transition_time").as_double()));
    TrajectoryRisk risk;
    float travelled = 0.0F;
    float vehicle_x = -lidar_offset_x;
    float vehicle_y = -lidar_offset_y;
    float yaw = 0.0F;
    while (travelled <= check_distance + 1.0e-4F)
    {
        const float cos_yaw = std::cos(yaw);
        const float sin_yaw = std::sin(yaw);
        for (const auto& obstacle : obstacle_points)
        {
            const float difference_x = obstacle.first - vehicle_x;
            const float difference_y = obstacle.second - vehicle_y;
            // Transform each LiDAR obstacle point into the predicted vehicle
            // frame and test the full rectangular footprint. base_link is the
            // rectangle centre; the scan origin is shifted by lidar_offset_*.
            const float local_x = cos_yaw * difference_x + sin_yaw * difference_y;
            const float local_y = -sin_yaw * difference_x + cos_yaw * difference_y;
            if (std::abs(local_x) <= half_vehicle_length
                && std::abs(local_y) <= half_vehicle_width)
            {
                risk.collision_distance = travelled;
                risk.collision_time = travelled / prediction_speed;
                risk.collision_obstacle_x = obstacle.first;
                risk.collision_obstacle_y = obstacle.second;
                return risk;
            }
        }
        if (travelled >= check_distance)
        {
            break;
        }

        const float distance_step = std::min(check_step, check_distance - travelled);
        const float midpoint_time
            = (travelled + 0.5F * distance_step) / prediction_speed;
        const float steering = modeled_steering_during_transition(
            checked_current_steering,
            checked_target_steering,
            midpoint_time,
            transition_time,
            maximum_steering);
        const float curvature = this->get_modeled_curvature(
            steering, prediction_speed);
        const float midpoint_yaw = yaw + 0.5F * curvature * distance_step;
        vehicle_x += std::cos(midpoint_yaw) * distance_step;
        vehicle_y += std::sin(midpoint_yaw) * distance_step;
        yaw += curvature * distance_step;
        travelled += distance_step;
    }

    return risk;
}

float ReactiveGapFollow::get_speed_calibration_factor() const
{
    return std::max(
        1.0e-3F,
        static_cast<float>(
            this->get_parameter("speed_calibration_factor").as_double()));
}

float ReactiveGapFollow::wheel_speed_to_vehicle_speed(float wheel_speed) const
{
    return wheel_speed * this->get_speed_calibration_factor();
}

float ReactiveGapFollow::vehicle_speed_to_wheel_speed(float vehicle_speed) const
{
    return vehicle_speed / this->get_speed_calibration_factor();
}

float ReactiveGapFollow::get_modeled_curvature(
    float steering_angle, float vehicle_speed) const
{
    const float maximum_steering = static_cast<float>(
        this->get_parameter("max_steering_angle").as_double()
        * M_PI / 180.0);
    const float checked_steering = std::max(
        -maximum_steering, std::min(steering_angle, maximum_steering));
    const float checked_speed = std::abs(vehicle_speed);
    const float command_degrees = std::abs(
        checked_steering * 180.0F / static_cast<float>(M_PI));
    const auto speed_bracket = interpolation_bracket(
        kCurvatureCalibrationSpeeds, checked_speed);
    const auto command_bracket = interpolation_bracket(
        kCurvatureCalibrationCommandsDeg, command_degrees);

    const float lower_command_curvature = interpolate(
        kMedianCurvature[speed_bracket.first][command_bracket.first],
        kMedianCurvature[speed_bracket.first + 1][command_bracket.first],
        speed_bracket.second);
    const float upper_command_curvature = interpolate(
        kMedianCurvature[speed_bracket.first][command_bracket.first + 1],
        kMedianCurvature[speed_bracket.first + 1][command_bracket.first + 1],
        speed_bracket.second);
    const float magnitude = interpolate(
        lower_command_curvature,
        std::max(lower_command_curvature, upper_command_curvature),
        command_bracket.second);
    const float direction_scale = checked_steering >= 0.0F
        ? kPositiveCurvatureScale : kNegativeCurvatureScale;
    float modeled_curvature
        = std::copysign(direction_scale * magnitude, checked_steering);
    const float speed_squared = checked_speed * checked_speed;
    if (speed_squared > 0.04F)
    {
        const float friction = calibrated_friction_coefficient(
            static_cast<float>(this->get_parameter("friction").as_double()),
            checked_speed);
        const float curvature_limit = friction * 9.81F / speed_squared;
        modeled_curvature = std::max(
            -curvature_limit,
            std::min(modeled_curvature, curvature_limit));
    }
    return modeled_curvature;
}

float ReactiveGapFollow::get_turning_speed_limit(
    float steering_angle, float speed_reference) const
{
    const float modeled_curvature = std::abs(
        this->get_modeled_curvature(steering_angle, speed_reference));
    if (modeled_curvature <= 1.0e-5F)
    {
        return std::numeric_limits<float>::infinity();
    }
    const float friction = calibrated_friction_coefficient(
        static_cast<float>(this->get_parameter("friction").as_double()),
        speed_reference);
    return std::sqrt(9.81F * friction / modeled_curvature);
}

void ReactiveGapFollow::publish_debug_markers(
    const std_msgs::msg::Header& header,
    float steering_angle,
    float target_distance,
    float collision_distance,
    const PathGuidance& path_guidance,
    float current_steering_angle,
    float prediction_speed,
    float commanded_speed,
    float collision_ttc,
    float braking_distance,
    float turning_speed_cap,
    float trajectory_speed_cap,
    bool emergency,
    bool braking_for_risk,
    bool preview_tracking_active,
    bool steering_avoidance_active,
    int steering_avoidance_direction)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker target_marker;
    target_marker.header = header;
    target_marker.ns = "gap_follow_ver2";
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

    const float check_distance = static_cast<float>(
        this->get_parameter("trajectory_check_distance").as_double());
    const float check_step = std::max(
        0.01F,
        static_cast<float>(this->get_parameter("trajectory_check_step").as_double()));
    const float lidar_offset_x = static_cast<float>(
        this->get_parameter("lidar_offset_x").as_double());
    const float lidar_offset_y = static_cast<float>(
        this->get_parameter("lidar_offset_y").as_double());
    const float safety_margin = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("trajectory_safety_margin").as_double()));
    const float half_vehicle_length = 0.5F * std::max(
        0.0F,
        static_cast<float>(this->get_parameter("vehicle_length").as_double()))
        + safety_margin;
    const float half_vehicle_width = 0.5F * std::max(
        0.0F,
        static_cast<float>(this->get_parameter("vehicle_width").as_double()))
        + safety_margin;
    const float maximum_steering = static_cast<float>(
        this->get_parameter("max_steering_angle").as_double()
        * M_PI / 180.0);
    const float checked_current_steering = std::max(
        -maximum_steering,
        std::min(current_steering_angle, maximum_steering));
    const float checked_target_steering = std::max(
        -maximum_steering, std::min(steering_angle, maximum_steering));
    const float transition_time = std::max(
        1.0e-3F,
        static_cast<float>(
            this->get_parameter("steering_transition_time").as_double()));
    const float rollout_speed = std::max(0.05F, std::abs(prediction_speed));
    // A control frame only commits the actuator transition, not a full 1.5 m
    // of constant target steering. Drawing beyond this time made a hard turn
    // look like a commanded circle even though the controller replans every
    // scan. The blue target arrow continues to show the longer-term intent.
    const float committed_rollout_distance = std::min(
        check_distance,
        std::max(check_step, rollout_speed * transition_time));
    float state_color_r = 0.1F;
    float state_color_g = 1.0F;
    float state_color_b = 0.2F;
    if (emergency)
    {
        // STOP: red
        state_color_r = 1.0F;
        state_color_g = 0.1F;
        state_color_b = 0.1F;
    }
    else if (braking_for_risk)
    {
        // BRAKE: amber
        state_color_r = 1.0F;
        state_color_g = 0.65F;
        state_color_b = 0.0F;
    }
    else if (steering_avoidance_active)
    {
        // Safe steering avoidance: cyan. It is collision-free, but it is not
        // ordinary path/gap following and must not be reported as plain CLEAR.
        state_color_r = 0.0F;
        state_color_g = 0.85F;
        state_color_b = 1.0F;
    }
    else if (preview_tracking_active)
    {
        // PREVIEW: a distant path blocker is being watched, but it has not
        // entered the physical braking gate and therefore does not reduce speed.
        state_color_r = 0.75F;
        state_color_g = 0.25F;
        state_color_b = 1.0F;
    }

    visualization_msgs::msg::Marker trajectory_marker;
    trajectory_marker.header = header;
    trajectory_marker.ns = "gap_follow_ver2";
    trajectory_marker.id = 1;
    trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker.scale.x = 0.04;
    trajectory_marker.color.r = state_color_r;
    trajectory_marker.color.g = state_color_g;
    trajectory_marker.color.b = state_color_b;
    trajectory_marker.color.a = 1.0F;

    visualization_msgs::msg::Marker footprint_marker;
    footprint_marker.header = header;
    footprint_marker.ns = "gap_follow_ver2";
    footprint_marker.id = 2;
    footprint_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    footprint_marker.action = visualization_msgs::msg::Marker::ADD;
    footprint_marker.scale.x = 0.012;
    footprint_marker.color.r = state_color_r;
    footprint_marker.color.g = state_color_g;
    footprint_marker.color.b = state_color_b;
    footprint_marker.color.a = 0.08F;

    float travelled = 0.0F;
    float vehicle_x = -lidar_offset_x;
    float vehicle_y = -lidar_offset_y;
    float vehicle_yaw = 0.0F;
    while (travelled <= committed_rollout_distance + 1.0e-4F)
    {
        geometry_msgs::msg::Point point;
        point.x = vehicle_x;
        point.y = vehicle_y;
        point.z = 0.03;
        trajectory_marker.points.push_back(point);

        const float cos_yaw = std::cos(vehicle_yaw);
        const float sin_yaw = std::sin(vehicle_yaw);
        const std::pair<float, float> local_corners[4] = {
            {half_vehicle_length, half_vehicle_width},
            {half_vehicle_length, -half_vehicle_width},
            {-half_vehicle_length, -half_vehicle_width},
            {-half_vehicle_length, half_vehicle_width},
        };
        geometry_msgs::msg::Point corners[4];
        for (size_t corner_index = 0; corner_index < 4; ++corner_index)
        {
            corners[corner_index].x = point.x
                + cos_yaw * local_corners[corner_index].first
                - sin_yaw * local_corners[corner_index].second;
            corners[corner_index].y = point.y
                + sin_yaw * local_corners[corner_index].first
                + cos_yaw * local_corners[corner_index].second;
            corners[corner_index].z = 0.025;
        }
        for (size_t corner_index = 0; corner_index < 4; ++corner_index)
        {
            footprint_marker.points.push_back(corners[corner_index]);
            footprint_marker.points.push_back(corners[(corner_index + 1) % 4]);
        }

        if (travelled >= committed_rollout_distance)
        {
            break;
        }
        const float distance_step = std::min(
            check_step, committed_rollout_distance - travelled);
        const float midpoint_time
            = (travelled + 0.5F * distance_step) / rollout_speed;
        const float transition_steering = modeled_steering_during_transition(
            checked_current_steering,
            checked_target_steering,
            midpoint_time,
            transition_time,
            maximum_steering);
        const float transition_curvature = this->get_modeled_curvature(
            transition_steering, prediction_speed);
        const float midpoint_yaw
            = vehicle_yaw + 0.5F * transition_curvature * distance_step;
        vehicle_x += std::cos(midpoint_yaw) * distance_step;
        vehicle_y += std::sin(midpoint_yaw) * distance_step;
        vehicle_yaw += transition_curvature * distance_step;
        travelled += distance_step;
    }
    marker_array.markers.push_back(trajectory_marker);
    marker_array.markers.push_back(footprint_marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header = header;
    text_marker.ns = "gap_follow_ver2";
    text_marker.id = 3;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = 0.3;
    text_marker.pose.position.y = 0.7;
    text_marker.pose.position.z = 0.45;
    text_marker.scale.z = 0.28;
    text_marker.color.r = state_color_r;
    text_marker.color.g = state_color_g;
    text_marker.color.b = state_color_b;
    text_marker.color.a = 1.0F;
    std::ostringstream status;
    const char* motion_state = emergency
        ? "STOP"
        : (braking_for_risk
            ? "BRAKE"
            : (steering_avoidance_active
                ? "AVOID"
                : (preview_tracking_active ? "PREVIEW" : "CLEAR")));
    status << motion_state << "  steer_state=";
    if (!steering_avoidance_active)
    {
        status << "FOLLOW";
    }
    else if (steering_avoidance_direction > 0)
    {
        status << "AVOID_LEFT";
    }
    else if (steering_avoidance_direction < 0)
    {
        status << "AVOID_RIGHT";
    }
    else
    {
        status << "AVOID";
    }
    status << "  speed=" << prediction_speed
           << " m/s  cmd_v=" << commanded_speed
           << " m/s  cmd_out=" << this->vehicle_speed_to_wheel_speed(commanded_speed)
           << "  speed_src="
           << (braking_for_risk
                ? "RISK"
                : (path_guidance.speed_available ? "PATH" : "LIDAR"))
           << "  steer="
           << steering_angle * 180.0F / static_cast<float>(M_PI)
           << " deg  current="
           << current_steering_angle * 180.0F / static_cast<float>(M_PI)
           << " deg  collision=" << collision_distance
           << " m  ttc=" << collision_ttc
           << " s  brake=" << braking_distance << " m"
           << "  turn_cap=" << turning_speed_cap << " m/s"
           << "  traj_cap=" << trajectory_speed_cap << " m/s"
           << "  detect="
           << this->get_parameter("obstacle_detection_distance").as_double()
           << " m  rollout=" << committed_rollout_distance << " m";
    if (path_guidance.available)
    {
        status << "  path="
               << path_guidance.target_angle * 180.0F / static_cast<float>(M_PI)
               << " deg  error=" << path_guidance.cross_track_error << " m";
        if (path_guidance.speed_available)
        {
            status << "  path_v=" << path_guidance.target_speed << " m/s";
        }
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

    if (path_guidance.available)
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
        float guard_clearance = 0.0F;
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
    const float candidate_safety_level = std::max(
        0.0F,
        std::min(
            1.0F,
            static_cast<float>(
                this->get_parameter("candidate_safety_level").as_double())));

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
        const float guard_clearance = get_safe_distance(
            ranges, i, check_width, candidate_safety_level);

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
        candidates[i].guard_clearance = guard_clearance;
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
            this->get_parameter("candidate_min_clearance").as_double()));
    const float minimum_score_ratio = std::max(
        0.0F,
        std::min(
            1.0F,
            static_cast<float>(
                this->get_parameter("path_candidate_min_score_ratio").as_double())));
    size_t best_index = lidar_best_index;
    float best_score = -std::numeric_limits<float>::infinity();
    for (size_t i = search_begin; i <= search_end; ++i)
    {
        float score = candidates[i].base_score;
        const bool path_safe_candidate = path_guidance.active
            && candidates[i].guard_clearance >= minimum_clearance
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
    const auto decision_time = this->now();

    const float obstacle_detection_distance = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("obstacle_detection_distance").as_double()));
    std::vector<std::pair<float, float>> obstacle_points;
    obstacle_points.reserve(raw_ranges.size());
    for (size_t i = 0; i < raw_ranges.size(); ++i)
    {
        const float range = raw_ranges[i];
        if (!std::isfinite(range) || range <= 0.0F
            || range > obstacle_detection_distance
            || range >= scan_msg->range_max * 0.99F)
        {
            continue;
        }
        const float angle = first_scan_angle + static_cast<float>(i) * angle_increment;
        obstacle_points.emplace_back(
            range * std::cos(angle), range * std::sin(angle));
    }

    const double odometry_timeout = std::max(
        0.0, this->get_parameter("odom_timeout").as_double());
    const double speed_odometry_age = has_speed_odometry
        ? (decision_time - latest_speed_odometry_receive_time).seconds()
        : std::numeric_limits<double>::infinity();
    const bool speed_odometry_is_fresh = has_speed_odometry
        && speed_odometry_age >= 0.0 && speed_odometry_age <= odometry_timeout;
    const double steering_feedback_age = has_steering_feedback
        ? (decision_time - latest_steering_feedback_receive_time).seconds()
        : std::numeric_limits<double>::infinity();
    const bool steering_feedback_is_fresh = has_steering_feedback
        && steering_feedback_age >= 0.0
        && steering_feedback_age
            <= this->get_parameter("steering_transition_time").as_double();
    const float steering_command_reference = steering_feedback_is_fresh
        ? latest_steering_feedback : last_published_steering;
    const int64_t decision_time_ns = decision_time.nanoseconds();
    const float steering_response_time = std::max(
        1.0e-3F,
        static_cast<float>(
            this->get_parameter("steering_transition_time").as_double()));
    if (estimated_steering_update_time == 0
        || decision_time_ns < estimated_steering_update_time)
    {
        estimated_current_steering = steering_command_reference;
    }
    else
    {
        const float elapsed = static_cast<float>(
            decision_time_ns - estimated_steering_update_time) * 1.0e-9F;
        // A command topic reports where the servo was asked to go. Model the
        // physical steering response with the already configured transition
        // time before using it as the initial state of every candidate rollout.
        // This prevents a rapid command reversal from being treated as if the
        // wheels had already crossed through the new target angle.
        const float response_ratio = 1.0F - std::exp(
            -std::max(0.0F, elapsed) / steering_response_time);
        estimated_current_steering += response_ratio * (
            steering_command_reference - estimated_current_steering);
    }
    estimated_steering_update_time = decision_time_ns;
    const float current_steering_angle = estimated_current_steering;
    const float maximum_vehicle_speed_measurement = 1.25F * std::max(
        0.1F,
        static_cast<float>(this->get_parameter("max_speed").as_double()));
    const float feedback_vehicle_speed = steering_feedback_is_fresh
        ? std::min(
            latest_vehicle_speed_feedback, maximum_vehicle_speed_measurement)
        : 0.0F;
    const float fallback_vehicle_speed = std::max(
        current_vehicle_speed, feedback_vehicle_speed);
    // Fresh wheel odometry is the actual calibrated vehicle speed and is the
    // only measured-speed input to TTC, braking distance, steering transition
    // prediction and the speed-dependent curvature model. The requested-speed
    // gate below separately validates acceleration targets, so an older command
    // must not masquerade as measured motion while the vehicle is stationary.
    const float vehicle_speed = speed_odometry_is_fresh
        ? filtered_vehicle_speed
        : fallback_vehicle_speed;
    const PathGuidance path_guidance = this->get_path_guidance();
    // TTC and braking below must use measured speed. Candidate geometry is
    // different: while launching from rest the car will accelerate toward the
    // path command during the 0.2 s steering response. Evaluating candidate
    // curvature at exactly zero speed produces an unrealistically tiny turning
    // circle and can reject the path side before the car moves at all.
    const float braking_deceleration = std::max(
        0.1F,
        static_cast<float>(
            this->get_parameter("normal_deceleration").as_double()));
    const float brake_reaction_time = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("steering_transition_time").as_double()));
    const float braking_margin = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("trajectory_safety_margin").as_double()));
    const float braking_distance = vehicle_speed * brake_reaction_time
        + vehicle_speed * vehicle_speed / (2.0F * braking_deceleration)
        + braking_margin;
    const float configured_trajectory_check_distance = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("trajectory_check_distance").as_double()));
    const float vehicle_length = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("vehicle_length").as_double()));
    const auto trajectory_check_distance_for_speed = [&](float candidate_speed)
    {
        // A high-speed candidate must cover at least the distance travelled
        // while the actuator moves from the measured steering to the target,
        // plus the front half of the rectangular footprint. The configured
        // distance is a baseline, not an upper bound; LiDAR range remains the
        // only upper bound. This prevents a 1.5 m rollout from approving a
        // 9 m/s command whose 0.2 s steering response already consumes 1.8 m.
        const float steering_response_distance
            = std::abs(candidate_speed) * brake_reaction_time
            + 0.5F * vehicle_length + braking_margin;
        return std::min(
            obstacle_detection_distance,
            std::max(
                configured_trajectory_check_distance,
                std::max(vehicle_length + braking_margin,
                         steering_response_distance)));
    };
    const float trajectory_check_distance
        = trajectory_check_distance_for_speed(vehicle_speed);
    // The complete detection horizon starts steering search.  It is deliberately
    // independent from braking_distance: seeing an obstacle earlier must buy
    // lateral response time, not automatically reduce speed.  Once a blocker is
    // found, the candidate rollout below is shortened to just past that blocker
    // so a distant side wall does not reject every usable detour.
    const float avoidance_preview_distance = obstacle_detection_distance;

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
    const double half_vehicle_width = 0.5 * std::max(
        0.0, this->get_parameter("vehicle_width").as_double());
    this->expand_obstacles(
        ranges,
        obstacle_edges,
        half_vehicle_width,
        angle_increment);
    this->expand_obstacles(
        preferred_ranges,
        obstacle_edges,
        half_vehicle_width
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
    const float candidate_safety_level = std::max(
        0.0F,
        std::min(
            1.0F,
            static_cast<float>(
                this->get_parameter("candidate_safety_level").as_double())));
    const auto guard_clearance_for_angle = [&](float angle)
    {
        return this->get_safe_distance(
            ranges,
            index_for_angle(angle),
            path_check_width,
            candidate_safety_level);
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
        const TrajectoryRisk selected_risk
            = this->get_transition_trajectory_risk(
                obstacle_points,
                current_steering_angle,
                target_angle,
                vehicle_speed,
                trajectory_check_distance);
        const float fallback_switch_distance = 0.5F * static_cast<float>(
            this->get_parameter("candidate_min_clearance").as_double()
            + trajectory_check_distance);
        if (std::isfinite(selected_risk.collision_distance)
            && selected_risk.collision_distance <= fallback_switch_distance)
        {
            const TrajectoryRisk opposite_risk
                = this->get_transition_trajectory_risk(
                    obstacle_points,
                    current_steering_angle,
                    opposite_target_angle,
                    vehicle_speed,
                    trajectory_check_distance);
            const float minimum_improvement = static_cast<float>(
                this->get_parameter("trajectory_check_step").as_double());
            if (opposite_target_distance > 0.0F
                && (!std::isfinite(opposite_risk.collision_distance)
                    || opposite_risk.collision_distance
                        > selected_risk.collision_distance + minimum_improvement))
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
        guard_clearance_for_angle(unsmoothed_target_angle), fallback_distance);
    // Temporal smoothing is also an angular blend, so apply the same guard.
    target_angle = guard_clearance_for_angle(smoothed_target_angle)
        >= smoothing_required_clearance
        ? smoothed_target_angle
        : unsmoothed_target_angle;
    target_angle = clamp_steering(target_angle);

    // Before accepting an emergency crawl, search the remaining steering range
    // for a trajectory that clears the obstacle. This preserves normal
    // distance-based speed when steering alone is sufficient, while keeping the
    // original crawl fallback when every reachable trajectory is still blocked.
    TrajectoryRisk target_risk = this->get_transition_trajectory_risk(
        obstacle_points,
        current_steering_angle,
        target_angle,
        vehicle_speed,
        trajectory_check_distance);
    float collision_distance = target_risk.collision_distance;
    float collision_ttc = target_risk.collision_time;
    bool using_steering_avoidance = false;
    const float minimum_avoidance_clearance = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("candidate_min_clearance").as_double()));
    const float candidate_minimum_speed = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("minimum_non_emergency_speed").as_double()));
    const float candidate_speed_ceiling = std::min(
        static_cast<float>(this->get_parameter("max_speed").as_double()),
        reference_path_max_speed);
    float candidate_requested_speed = candidate_minimum_speed;
    if (path_guidance.speed_available)
    {
        candidate_requested_speed = std::max(
            candidate_minimum_speed, path_guidance.target_speed);
    }
    else
    {
        // Match the fallback branch used by the final speed layer.  This avoids
        // choosing steering at only the minimum speed and discovering later
        // that the actual distance/approach-speed request needs another turn.
        candidate_requested_speed = std::max(
            set_speed_from_distance(
                selected_target_distance, target_angle, vehicle_speed),
            current_vehicle_speed);
    }
    candidate_requested_speed = std::min(
        candidate_requested_speed, candidate_speed_ceiling);
    const float candidate_trajectory_speed = std::max(
        vehicle_speed, candidate_requested_speed);
    // Candidate eligibility stays local even though the steering trigger sees
    // the full 3 m horizon.  Extending every constant-command rollout to the
    // full LiDAR distance rejects valid detours when their hypothetical later
    // continuation reaches a side wall.  The extra preview gate below only
    // requires the candidate to improve the distant blocker.
    const float local_avoidance_candidate_check_distance = std::min(
        obstacle_detection_distance,
        std::max(
            trajectory_check_distance_for_speed(candidate_trajectory_speed),
            configured_trajectory_check_distance
                + 1.5F * vehicle_length + braking_margin));
    const float minimum_preview_improvement = std::max(
        static_cast<float>(
            this->get_parameter("trajectory_check_step").as_double()),
        0.5F * static_cast<float>(
            this->get_parameter("vehicle_width").as_double()));

    struct AvoidanceCandidate
    {
        bool valid = false;
        float angle = 0.0F;
        float collision_distance = std::numeric_limits<float>::infinity();
        float collision_time = std::numeric_limits<float>::infinity();
        TrajectoryRisk current_speed_preview_risk;
        TrajectoryRisk requested_speed_preview_risk;
        float score = -std::numeric_limits<float>::infinity();
    };

    const auto find_avoidance_candidate = [&] (
        bool require_preview_improvement,
        const TrajectoryRisk& current_speed_baseline_preview_risk,
        const TrajectoryRisk& requested_speed_baseline_preview_risk)
    {
        AvoidanceCandidate best;
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
        const float steering_change_penalty = std::max(
            0.0F,
            static_cast<float>(
                this->get_parameter("avoidance_steering_change_penalty").as_double()));
        const float distance_cap = std::max(
            1.0e-3F,
            static_cast<float>(
                this->get_parameter("gap_score_distance_cap").as_double()));
        const float path_alignment_sigma = std::max(
            1.0e-3F,
            static_cast<float>(
                this->get_parameter("path_alignment_sigma").as_double()
                * M_PI / 180.0));

        std::vector<AvoidanceCandidate> safe_candidates;
        float lidar_best_score = -std::numeric_limits<float>::infinity();
        for (float candidate_angle = search_min_angle;
            candidate_angle <= search_max_angle + 0.5F * search_step;
            candidate_angle += search_step)
        {
            const float checked_angle = std::min(candidate_angle, search_max_angle);
            // Eligibility uses the lower candidate quantile so a narrow static
            // obstacle cannot be hidden by the broad p80 scoring clearance.
            const float guard_clearance
                = guard_clearance_for_angle(checked_angle);
            if (guard_clearance < minimum_avoidance_clearance)
            {
                continue;
            }
            const TrajectoryRisk candidate_risk
                = this->get_transition_trajectory_risk(
                    obstacle_points,
                    current_steering_angle,
                    checked_angle,
                    candidate_trajectory_speed,
                    local_avoidance_candidate_check_distance);
            // The candidate must clear the complete maneuver horizon above.
            // A later side-wall intersection of the hypothetical constant-turn
            // continuation is not part of this maneuver and is replanned from
            // the next scan instead.
            if (std::isfinite(candidate_risk.collision_distance))
            {
                continue;
            }
            const bool distinct_candidate_speed
                = std::abs(candidate_trajectory_speed - vehicle_speed) > 0.05F;
            const TrajectoryRisk current_speed_candidate_risk
                = distinct_candidate_speed
                ? this->get_transition_trajectory_risk(
                    obstacle_points,
                    current_steering_angle,
                    checked_angle,
                    vehicle_speed,
                    local_avoidance_candidate_check_distance)
                : candidate_risk;
            if (std::isfinite(current_speed_candidate_risk.collision_distance))
            {
                continue;
            }

            TrajectoryRisk current_speed_candidate_preview_risk;
            TrajectoryRisk requested_speed_candidate_preview_risk;
            if (require_preview_improvement)
            {
                requested_speed_candidate_preview_risk
                    = this->get_transition_trajectory_risk(
                    obstacle_points,
                    current_steering_angle,
                    checked_angle,
                    candidate_trajectory_speed,
                    avoidance_preview_distance);
                current_speed_candidate_preview_risk = distinct_candidate_speed
                    ? this->get_transition_trajectory_risk(
                        obstacle_points,
                        current_steering_angle,
                        checked_angle,
                        vehicle_speed,
                        avoidance_preview_distance)
                    : requested_speed_candidate_preview_risk;
                const auto preserves_or_improves = [&] (
                    const TrajectoryRisk& candidate_preview_risk,
                    const TrajectoryRisk& baseline_preview_risk)
                {
                    if (!std::isfinite(baseline_preview_risk.collision_distance))
                    {
                        return !std::isfinite(
                            candidate_preview_risk.collision_distance);
                    }
                    return !std::isfinite(candidate_preview_risk.collision_distance)
                        || candidate_preview_risk.collision_distance
                            >= baseline_preview_risk.collision_distance
                                + minimum_preview_improvement;
                };
                if (!preserves_or_improves(
                        current_speed_candidate_preview_risk,
                        current_speed_baseline_preview_risk)
                    || !preserves_or_improves(
                        requested_speed_candidate_preview_risk,
                        requested_speed_baseline_preview_risk))
                {
                    continue;
                }
            }

            // Rank only after the guard-clearance and rectangular transition
            // gates above have accepted the candidate. Path alignment therefore
            // chooses among safe escape trajectories and cannot make a blocked
            // direction eligible.
            const float candidate_clearance = clearance_for_angle(checked_angle);
            float candidate_score
                = std::min(candidate_clearance, distance_cap)
                - steering_change_penalty * (
                    std::abs(checked_angle - target_angle)
                    + std::abs(checked_angle - current_steering_angle));
            AvoidanceCandidate candidate;
            candidate.valid = true;
            candidate.score = candidate_score;
            candidate.angle = checked_angle;
            candidate.collision_distance = candidate_risk.collision_distance;
            candidate.collision_time = candidate_risk.collision_time;
            candidate.current_speed_preview_risk
                = current_speed_candidate_preview_risk;
            candidate.requested_speed_preview_risk
                = requested_speed_candidate_preview_risk;
            safe_candidates.push_back(candidate);
            lidar_best_score = std::max(lidar_best_score, candidate_score);
        }

        // Every entry already passed the guard and rectangular-trajectory gates.
        // Path alignment is a soft score only.  A hard sign(path_angle) filter
        // made the chosen side flip whenever a near-zero path bearing crossed
        // zero, cancelling the physical steering response despite no latch.
        const float minimum_score_ratio = std::max(
            0.0F,
            std::min(
                1.0F,
                static_cast<float>(
                    this->get_parameter("path_candidate_min_score_ratio").as_double())));
        for (auto candidate : safe_candidates)
        {
            const bool path_safe_candidate = path_guidance.active
                && candidate.score >= lidar_best_score * minimum_score_ratio;
            if (path_safe_candidate)
            {
                const float path_difference = static_cast<float>(normalize_angle(
                    candidate.angle - path_guidance.target_angle));
                const float normalized_difference
                    = path_difference / path_alignment_sigma;
                const float alignment = std::exp(
                    -0.5F * normalized_difference * normalized_difference);
                candidate.score += path_guidance.score_weight * alignment;
            }
            if (candidate.score > best.score)
            {
                best = candidate;
            }
        }
        return best;
    };

    const auto apply_avoidance_candidate = [&](const AvoidanceCandidate& candidate)
    {
        target_angle = candidate.angle;
        collision_distance = candidate.collision_distance;
        collision_ttc = candidate.collision_time;
        using_steering_avoidance = true;
        // Do not blend the safe escape steering with older commands that point
        // toward the collision that triggered this search.
        steering_window.clear();
        steering_sum = 0.0F;
        target_angle = this->smooth_steering(target_angle);
    };

    // Look across the full detection range at the speed the path may request.
    // This is a steering trigger, not a distance-based braking trigger.  The
    // selected candidate only needs to be locally collision-free and to move
    // the distant collision meaningfully farther away; the controller replans
    // before that later point is reached.
    const bool distinct_candidate_speed
        = std::abs(candidate_trajectory_speed - vehicle_speed) > 0.05F;
    const TrajectoryRisk requested_speed_target_preview_risk
        = this->get_transition_trajectory_risk(
            obstacle_points,
            current_steering_angle,
            target_angle,
            candidate_trajectory_speed,
            avoidance_preview_distance);
    const TrajectoryRisk current_speed_target_preview_risk
        = distinct_candidate_speed
        ? this->get_transition_trajectory_risk(
            obstacle_points,
            current_steering_angle,
            target_angle,
            vehicle_speed,
            avoidance_preview_distance)
        : requested_speed_target_preview_risk;
    TrajectoryRisk requested_speed_route_preview_risk;
    TrajectoryRisk current_speed_route_preview_risk;
    if (path_guidance.active)
    {
        const float reference_path_angle
            = clamp_steering(path_guidance.target_angle);
        requested_speed_route_preview_risk
            = this->get_transition_trajectory_risk(
                obstacle_points,
                current_steering_angle,
                reference_path_angle,
                candidate_trajectory_speed,
                avoidance_preview_distance);
        current_speed_route_preview_risk = distinct_candidate_speed
            ? this->get_transition_trajectory_risk(
                obstacle_points,
                current_steering_angle,
                reference_path_angle,
                vehicle_speed,
                avoidance_preview_distance)
            : requested_speed_route_preview_risk;
    }
    else
    {
        requested_speed_route_preview_risk
            = this->get_transition_trajectory_risk(
                obstacle_points,
                current_steering_angle,
                0.0F,
                candidate_trajectory_speed,
                avoidance_preview_distance);
        current_speed_route_preview_risk = distinct_candidate_speed
            ? this->get_transition_trajectory_risk(
                obstacle_points,
                current_steering_angle,
                0.0F,
                vehicle_speed,
                avoidance_preview_distance)
            : requested_speed_route_preview_risk;
    }

    const bool target_preview_is_blocked
        = std::isfinite(current_speed_target_preview_risk.collision_distance)
        || std::isfinite(requested_speed_target_preview_risk.collision_distance);
    const bool route_preview_is_blocked
        = std::isfinite(current_speed_route_preview_risk.collision_distance)
        || std::isfinite(requested_speed_route_preview_risk.collision_distance);
    const bool preview_tracked
        = target_preview_is_blocked || route_preview_is_blocked;
    TrajectoryRisk current_speed_selected_preview_risk
        = current_speed_target_preview_risk;
    TrajectoryRisk requested_speed_selected_preview_risk
        = requested_speed_target_preview_risk;
    if (target_preview_is_blocked)
    {
        const AvoidanceCandidate early_candidate
            = find_avoidance_candidate(
                true,
                current_speed_target_preview_risk,
                requested_speed_target_preview_risk);
        if (early_candidate.valid)
        {
            apply_avoidance_candidate(early_candidate);
            current_speed_selected_preview_risk
                = early_candidate.current_speed_preview_risk;
            requested_speed_selected_preview_risk
                = early_candidate.requested_speed_preview_risk;
        }
    }
    else if (route_preview_is_blocked)
    {
        // The ordinary gap target is already a full-horizon safe detour around
        // a blocked reference/straight route. Keep that better target exactly
        // as-is and only expose that it is active avoidance.
        using_steering_avoidance = true;
    }

    const TrajectoryRisk straight_risk = this->get_transition_trajectory_risk(
        obstacle_points,
        current_steering_angle,
        0.0F,
        vehicle_speed,
        avoidance_preview_distance);
    const float straight_collision_distance = straight_risk.collision_distance;
    // Final independent safety gate. Recompute the exact transition after
    // smoothing and per-scan path preference. If it is blocked, compare every
    // safe steering candidate again without preserving a previous side.
    target_risk = this->get_transition_trajectory_risk(
        obstacle_points,
        current_steering_angle,
        target_angle,
        vehicle_speed,
        trajectory_check_distance);
    collision_distance = target_risk.collision_distance;
    collision_ttc = target_risk.collision_time;
    if (std::isfinite(collision_distance))
    {
        const AvoidanceCandidate candidate = find_avoidance_candidate(
            preview_tracked,
            current_speed_selected_preview_risk,
            requested_speed_selected_preview_risk);
        if (candidate.valid)
        {
            apply_avoidance_candidate(candidate);
            if (preview_tracked)
            {
                current_speed_selected_preview_risk
                    = candidate.current_speed_preview_risk;
                requested_speed_selected_preview_risk
                    = candidate.requested_speed_preview_risk;
            }
            // apply_avoidance_candidate() resets smoothing, but verify its final
            // output rather than relying on the candidate's pre-apply values.
            target_risk = this->get_transition_trajectory_risk(
                obstacle_points,
                current_steering_angle,
                target_angle,
                vehicle_speed,
                trajectory_check_distance);
            collision_distance = target_risk.collision_distance;
            collision_ttc = target_risk.collision_time;
        }
    }

    // A far blocker that has no completely clear candidate is kept separate
    // from immediate risk.  It reaches the speed layer only when the selected
    // transition is inside the physically attainable stopping distance.  This
    // fixes the old overwrite where a 3 m risk was immediately replaced by a
    // short 1.5 m CLEAR result.
    if (preview_tracked)
    {
        current_speed_selected_preview_risk
            = this->get_transition_trajectory_risk(
            obstacle_points,
            current_steering_angle,
            target_angle,
            vehicle_speed,
            avoidance_preview_distance);
        const float preview_braking_gate = braking_distance + std::max(
            0.01F,
            static_cast<float>(
                this->get_parameter("trajectory_check_step").as_double()));
        if (std::isfinite(
                current_speed_selected_preview_risk.collision_distance)
            && current_speed_selected_preview_risk.collision_distance
                <= preview_braking_gate
            && (!std::isfinite(collision_distance)
                || current_speed_selected_preview_risk.collision_distance
                    < collision_distance))
        {
            collision_distance
                = current_speed_selected_preview_risk.collision_distance;
            collision_ttc = current_speed_selected_preview_risk.collision_time;
        }
    }

    const float final_guard_clearance = guard_clearance_for_angle(target_angle);
    const bool final_trajectory_is_safe
        = final_guard_clearance >= minimum_avoidance_clearance
        && !std::isfinite(collision_distance);
    const bool obstacle_on_straight_trajectory
        = std::isfinite(straight_collision_distance);

    // A straight-only trigger starts too late once the vehicle has already
    // yawed away from a centerline obstacle. Detect the same condition on the
    // locked global-path direction. This is recomputed every scan and does not
    // commit either steering side over time.
    bool obstacle_on_reference_path = false;
    if (path_guidance.active)
    {
        const float reference_path_angle
            = clamp_steering(path_guidance.target_angle);
        const TrajectoryRisk reference_path_risk
            = this->get_transition_trajectory_risk(
                obstacle_points,
                current_steering_angle,
                reference_path_angle,
                vehicle_speed,
                avoidance_preview_distance);
        const float reference_path_guard_clearance
            = guard_clearance_for_angle(reference_path_angle);
        const bool reference_path_is_blocked
            = reference_path_guard_clearance < minimum_avoidance_clearance
            || std::isfinite(reference_path_risk.collision_distance);
        const float detour_from_reference_path = static_cast<float>(std::abs(
            normalize_angle(target_angle - reference_path_angle)));
        const float path_detour_threshold = 0.5F * std::max(
            angle_increment,
            static_cast<float>(
                this->get_parameter("avoidance_steering_step").as_double()
                * M_PI / 180.0));
        obstacle_on_reference_path
            = reference_path_is_blocked
            && detour_from_reference_path >= path_detour_threshold;
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

    // Preserve the risk of executing the selected steering at the measured
    // speed. A second check at the requested speed is performed below, after
    // path and curvature speed targets have been combined.
    const float critical_ttc = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("steering_transition_time").as_double()));
    const bool current_trajectory_has_risk
        = std::isfinite(collision_distance);
    const bool emergency = current_trajectory_has_risk
        && collision_ttc <= critical_ttc;
    float risk_speed_cap = std::numeric_limits<float>::infinity();
    if (current_trajectory_has_risk)
    {
        const float available_distance = std::max(
            0.0F, collision_distance - braking_margin);
        const float reaction_term = braking_deceleration * brake_reaction_time;
        risk_speed_cap = -reaction_term + std::sqrt(
            reaction_term * reaction_term
            + 2.0F * braking_deceleration * available_distance);
        risk_speed_cap = std::max(
            static_cast<float>(
                this->get_parameter("minimum_crawl_speed").as_double()),
            risk_speed_cap);
    }

    const auto command_time = this->now();
    new_msg.header.stamp = command_time;
    new_msg.header.frame_id = this->get_parameter("command_frame_id").as_string();
    new_msg.drive.steering_angle = target_angle;
    float desired_speed = set_speed_from_distance(
        safe_distance, target_angle, vehicle_speed);
    const bool safe_avoidance_context
        = using_steering_avoidance
        || preview_tracked
        || obstacle_on_straight_trajectory
        || obstacle_on_reference_path;
    const bool preserve_safe_avoidance_speed
        = final_trajectory_is_safe
        && !current_trajectory_has_risk
        && safe_avoidance_context;
    const bool use_path_speed
        = final_trajectory_is_safe
        && !current_trajectory_has_risk
        && path_guidance.speed_available;
    if (use_path_speed)
    {
        // In every collision-free state, position.z on the locked global path
        // is the speed target. Corridor width and nearby side walls choose the
        // steering candidate but must not quietly turn into a speed command.
        const float maximum_speed = static_cast<float>(
            this->get_parameter("max_speed").as_double());
        desired_speed = std::max(
            static_cast<float>(
                this->get_parameter("minimum_non_emergency_speed").as_double()),
            std::min(path_guidance.target_speed, maximum_speed));
    }
    else if (preserve_safe_avoidance_speed)
    {
        // With no valid path speed, retain the approach speed when possible.
        // The common curvature and requested-speed trajectory gates below are
        // still authoritative and may lower it when that speed is not feasible.
        const float maximum_speed = static_cast<float>(
            this->get_parameter("max_speed").as_double());
        // A path without a valid z value retains the legacy behavior as a
        // fallback, rather than inventing a speed.
        const float retained_speed = std::min(
            current_vehicle_speed, maximum_speed);
        desired_speed = std::max(desired_speed, retained_speed);
    }
    if (using_narrow_gap
        && !use_path_speed
        && !preserve_safe_avoidance_speed)
    {
        desired_speed = std::min(
            desired_speed,
            static_cast<float>(this->get_parameter("narrow_gap_max_speed").as_double()));
    }
    const float configured_maximum_speed = static_cast<float>(
        this->get_parameter("max_speed").as_double());
    const float command_speed_ceiling = std::min(
        configured_maximum_speed, reference_path_max_speed);
    desired_speed = std::min(desired_speed, command_speed_ceiling);

    // The path z value is a target, not permission to violate the measured
    // speed-dependent turning radius. Apply the lateral-acceleration limit
    // after every path/avoidance override so none of those branches can erase
    // it again.
    const float requested_speed_before_turn_cap = desired_speed;
    const float turning_speed_cap = this->get_turning_speed_limit(
        target_angle, std::max(vehicle_speed, desired_speed));
    desired_speed = std::min(desired_speed, turning_speed_cap);
    const float speed_comparison_epsilon = 0.02F;
    const bool turning_speed_limited
        = desired_speed + speed_comparison_epsilon
            < requested_speed_before_turn_cap;

    // Recheck the exact final steering transition at the speed we are about to
    // request. Steering candidates were selected using the measured speed; a
    // path-speed override can be much faster and therefore have a larger turn
    // radius and a longer actuator-response distance. When the requested speed
    // is unsafe, search for the highest lower speed that makes the same steering
    // trajectory collision-free. This preserves speed when steering alone is
    // sufficient and reduces only as much as the selected trajectory requires.
    desired_speed = std::min(desired_speed, risk_speed_cap);
    const float minimum_crawl_speed = std::max(
        0.0F,
        static_cast<float>(
            this->get_parameter("minimum_crawl_speed").as_double()));
    const auto risk_for_candidate_speed = [&](float candidate_speed)
    {
        float speed_check_distance
            = trajectory_check_distance_for_speed(candidate_speed);
        const float candidate_braking_distance
            = std::abs(candidate_speed) * brake_reaction_time
            + candidate_speed * candidate_speed
                / (2.0F * braking_deceleration)
            + braking_margin;
        speed_check_distance = std::min(
            avoidance_preview_distance,
            std::max(speed_check_distance, candidate_braking_distance));
        return this->get_transition_trajectory_risk(
            obstacle_points,
            current_steering_angle,
            target_angle,
            candidate_speed,
            speed_check_distance);
    };
    const float requested_trajectory_speed = desired_speed;
    const TrajectoryRisk requested_speed_risk
        = risk_for_candidate_speed(requested_trajectory_speed);
    const bool requested_trajectory_has_risk
        = std::isfinite(requested_speed_risk.collision_distance);
    float trajectory_speed_cap = std::numeric_limits<float>::infinity();
    if (requested_trajectory_has_risk)
    {
        // Show the requested-speed failure in debug when the currently measured
        // speed is still safe. The emergency flag remains based only on the
        // measured-speed trajectory, so this gate cannot create a false hard stop
        // while merely preventing unsafe acceleration.
        if (!current_trajectory_has_risk)
        {
            collision_distance = requested_speed_risk.collision_distance;
            collision_ttc = requested_speed_risk.collision_time;
        }

        const float search_floor = std::min(
            requested_trajectory_speed, minimum_crawl_speed);
        float previous_unsafe_speed = requested_trajectory_speed;
        float highest_safe_speed = -1.0F;
        constexpr int speed_search_samples = 12;
        for (int sample = 1; sample <= speed_search_samples; ++sample)
        {
            const float ratio
                = static_cast<float>(sample)
                / static_cast<float>(speed_search_samples);
            const float candidate_speed = requested_trajectory_speed
                + ratio * (search_floor - requested_trajectory_speed);
            const TrajectoryRisk candidate_risk
                = risk_for_candidate_speed(candidate_speed);
            if (!std::isfinite(candidate_risk.collision_distance))
            {
                highest_safe_speed = candidate_speed;
                break;
            }
            previous_unsafe_speed = candidate_speed;
        }

        if (highest_safe_speed >= 0.0F)
        {
            // Refine the highest safe boundary without adding another tuning
            // parameter. Ten iterations resolve far below command precision.
            float safe_speed = highest_safe_speed;
            float unsafe_speed = previous_unsafe_speed;
            for (int iteration = 0; iteration < 10; ++iteration)
            {
                const float candidate_speed
                    = 0.5F * (safe_speed + unsafe_speed);
                const TrajectoryRisk candidate_risk
                    = risk_for_candidate_speed(candidate_speed);
                if (std::isfinite(candidate_risk.collision_distance))
                {
                    unsafe_speed = candidate_speed;
                }
                else
                {
                    safe_speed = candidate_speed;
                }
            }
            trajectory_speed_cap = safe_speed;
        }
        else
        {
            // The selected steering remains blocked even at crawl. Keep the
            // vehicle steerable, but do not pretend that path speed is safe.
            trajectory_speed_cap = search_floor;
        }
        desired_speed = std::min(desired_speed, trajectory_speed_cap);
    }

    bool braking_for_risk
        = current_trajectory_has_risk
        || turning_speed_limited
        || requested_trajectory_has_risk;
    // These are safety ceilings, not merely desired targets. The ordinary
    // deceleration limiter may smooth a comfort slowdown, but it must never
    // leave the published command above a speed that the selected turn or
    // transition trajectory can physically support.
    const float safety_speed_ceiling = std::min(
        command_speed_ceiling,
        std::min(
            risk_speed_cap,
            std::min(turning_speed_cap, trajectory_speed_cap)));
    desired_speed = std::min(desired_speed, safety_speed_ceiling);
    const float minimum_command_speed = std::min(
        safety_speed_ceiling,
        braking_for_risk
            ? minimum_crawl_speed
            : static_cast<float>(
                this->get_parameter("minimum_non_emergency_speed").as_double()));
    // A collision-free path command must not inherit the old distance-based
    // acceleration taper. Otherwise a narrow but safe corridor remains at the
    // minimum speed for its entire length even though desired_speed is high.
    const float acceleration_distance = use_path_speed
        ? static_cast<float>(
            this->get_parameter("speed_increase_end").as_double())
        : safe_distance;
    const uint64_t command_time_ns
        = static_cast<uint64_t>(command_time.nanoseconds());
    float command_elapsed_time = static_cast<float>(
        this->get_parameter("initial_update_time").as_double());
    if (last_update_time != 0 && command_time_ns > last_update_time)
    {
        command_elapsed_time
            = static_cast<float>(command_time_ns - last_update_time) * 1.0e-9F;
    }
    command_elapsed_time = std::max(1.0e-3F, command_elapsed_time);
    const float previous_command_speed = current_vehicle_speed;
    float acceleration = 0.0;
    float commanded_vehicle_speed = limit_speed_change(
        desired_speed,
        acceleration_distance,
        target_angle,
        emergency,
        minimum_command_speed,
        safety_speed_ceiling,
        command_time_ns,
        acceleration);

    // The acceleration limiter can publish a speed between the measured and
    // requested endpoints.  Curvature is speed-dependent and not monotonic, so
    // checking only those endpoints is insufficient. Validate the exact value
    // that will be put on the wire and lower it only if that value is unsafe.
    const TrajectoryRisk published_speed_risk
        = risk_for_candidate_speed(commanded_vehicle_speed);
    if (std::isfinite(published_speed_risk.collision_distance))
    {
        collision_distance = published_speed_risk.collision_distance;
        collision_ttc = published_speed_risk.collision_time;
        const float search_floor = std::min(
            commanded_vehicle_speed, minimum_crawl_speed);
        float previous_unsafe_speed = commanded_vehicle_speed;
        float highest_safe_speed = -1.0F;
        constexpr int publish_speed_search_samples = 16;
        for (int sample = 1; sample <= publish_speed_search_samples; ++sample)
        {
            const float ratio = static_cast<float>(sample)
                / static_cast<float>(publish_speed_search_samples);
            const float candidate_speed = commanded_vehicle_speed
                + ratio * (search_floor - commanded_vehicle_speed);
            const TrajectoryRisk candidate_risk
                = risk_for_candidate_speed(candidate_speed);
            if (!std::isfinite(candidate_risk.collision_distance))
            {
                highest_safe_speed = candidate_speed;
                break;
            }
            previous_unsafe_speed = candidate_speed;
        }
        if (highest_safe_speed >= 0.0F)
        {
            float safe_speed = highest_safe_speed;
            float unsafe_speed = previous_unsafe_speed;
            for (int iteration = 0; iteration < 10; ++iteration)
            {
                const float candidate_speed
                    = 0.5F * (safe_speed + unsafe_speed);
                const TrajectoryRisk candidate_risk
                    = risk_for_candidate_speed(candidate_speed);
                if (std::isfinite(candidate_risk.collision_distance))
                {
                    unsafe_speed = candidate_speed;
                }
                else
                {
                    safe_speed = candidate_speed;
                }
            }
            commanded_vehicle_speed = safe_speed;
        }
        else
        {
            commanded_vehicle_speed = search_floor;
        }
        trajectory_speed_cap = std::min(
            trajectory_speed_cap, commanded_vehicle_speed);
        current_vehicle_speed = commanded_vehicle_speed;
        acceleration = (commanded_vehicle_speed - previous_command_speed)
            / command_elapsed_time;
        braking_for_risk = true;
    }
    new_msg.drive.speed = this->vehicle_speed_to_wheel_speed(
        commanded_vehicle_speed);
    new_msg.drive.acceleration = this->vehicle_speed_to_wheel_speed(acceleration);

    drive_publisher->publish(new_msg);
    last_published_steering = target_angle;

    if (this->get_parameter("debug").as_bool())
    {
        geometry_msgs::msg::PointStamped target_waypoint_msg;
        target_waypoint_msg.header = scan_msg->header;
        target_waypoint_msg.point.x = target_distance * std::cos(target_angle);
        target_waypoint_msg.point.y = target_distance * std::sin(target_angle);
        target_publisher->publish(target_waypoint_msg);
        this->publish_debug_markers(
            scan_msg->header, target_angle, target_distance, collision_distance,
            path_guidance, current_steering_angle, vehicle_speed,
            commanded_vehicle_speed, collision_ttc, braking_distance,
            turning_speed_cap, trajectory_speed_cap,
            emergency, braking_for_risk,
            preview_tracked, using_steering_avoidance,
            target_angle > 0.0F ? 1 : (target_angle < 0.0F ? -1 : 0));
    }

    if (using_steering_avoidance)
    {
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "Avoided emergency crawl with steering %.1f deg",
            target_angle * 180.0F / static_cast<float>(M_PI));
    }
};

float ReactiveGapFollow::set_speed_from_distance(
    float distance, float steering_angle, float vehicle_speed)
{
    float speed = distance * this->get_parameter("speed_factor").as_double();
    speed *= 1.0
        + (this->get_parameter("speed_increase_factor").as_double() - 1.0)
        * get_speed_increase_ratio(distance);

    // Use the measured speed-dependent curvature for both footprint prediction
    // and the lateral-acceleration speed cap. Evaluate at the larger of current
    // and requested speed so acceleration cannot assume a tighter low-speed arc.
    const float curvature_speed = std::max(vehicle_speed, speed);
    const float turning_speed = this->get_turning_speed_limit(
        steering_angle, curvature_speed);
    float desired_speed = std::min(
        std::min(speed, turning_speed),
        static_cast<float>(this->get_parameter("max_speed").as_double()));
    // Keep normal obstacle avoidance above a distinct non-emergency floor.
    // limit_speed_change() is the only place allowed to command the lower crawl
    // speed, and only when the selected trajectory is actually in collision.
    desired_speed = std::max(
        desired_speed,
        static_cast<float>(
            this->get_parameter("minimum_non_emergency_speed").as_double()));
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
    float steering_angle,
    bool emergency,
    float minimum_command_speed,
    float maximum_command_speed,
    uint64_t current_time,
    float& acceleration)
{
    float elapsed_time = this->get_parameter("initial_update_time").as_double();
    if (last_update_time != 0 && current_time > last_update_time)
    {
        elapsed_time = static_cast<float>(current_time - last_update_time) * 1.0e-9;
    }
    last_update_time = current_time;

    const float previous_speed = current_vehicle_speed;
    const float checked_maximum_speed = std::max(0.0F, maximum_command_speed);
    const float checked_minimum_speed = std::max(
        0.0F, std::min(minimum_command_speed, checked_maximum_speed));
    desired_speed = std::max(
        checked_minimum_speed,
        std::min(desired_speed, checked_maximum_speed));
    const float minimum_crawl_speed = static_cast<float>(
        this->get_parameter("minimum_crawl_speed").as_double());
    const float max_deceleration = static_cast<float>(
        this->get_parameter("max_deceleration").as_double());
    const float normal_deceleration = std::max(
        0.1F,
        static_cast<float>(
            this->get_parameter("normal_deceleration").as_double()));

    // Decelerate hard toward a crawl speed instead of snapping to a full stop.
    // At speed 0 the Ackermann model cannot turn in place, so a hard stop next
    // to a wall would leave the car unable to steer itself back out.
    if (emergency)
    {
        current_vehicle_speed = std::max(
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
        current_vehicle_speed = std::min(
            desired_speed, previous_speed + acceleration_limit * elapsed_time);
    }
    else
    {
        current_vehicle_speed = std::max(
            desired_speed, previous_speed - normal_deceleration * elapsed_time);
    }

    // The non-emergency minimum is a floor, not a fixed target: normal path
    // and gap logic can command any higher value. BRAKE/STOP passes the crawl
    // floor instead. Clamp the upper side as well so acceleration history can
    // never carry a command above the maximum encoded by the locked path.
    current_vehicle_speed = std::max(
        checked_minimum_speed,
        std::min(current_vehicle_speed, checked_maximum_speed));

    acceleration = (current_vehicle_speed - previous_speed) / elapsed_time;
    return current_vehicle_speed;
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
