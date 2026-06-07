#include "mpc_controller.h"

#include <algorithm>

namespace mpc_follower
{
namespace
{
double clampValue(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}
}

MpcController::MpcController(const rclcpp::NodeOptions & options)
    : Node("mpc_controller", options),
      global_optimizer_(nlopt::GN_CRS2_LM, 2 * 30),
      local_optimizer_(nlopt::LD_SLSQP, 2 * 30),
      rng_(std::random_device{}())
{
    loadParameters();
    global_optimizer_ = nlopt::opt(nlopt::GN_CRS2_LM, 2 * params_.N);
    local_optimizer_ = nlopt::opt(nlopt::LD_SLSQP, 2 * params_.N);
    configureGlobalOptimizer();
    configureLocalOptimizer();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(params_.Ts * 1000)),
        std::bind(&MpcController::controlLoop, this));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/sPath", 10, std::bind(&MpcController::onReferencePath, this, std::placeholders::_1));
    // 直接订阅 costmap，全量图和增量更新共同维护最新障碍物代价。
    obstacle_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap/costmap", 10, std::bind(&MpcController::detectObstacles, this, std::placeholders::_1));
    obstacle_update_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        "/costmap/costmap_updates", 10,
        std::bind(&MpcController::detectObstacleUpdates, this, std::placeholders::_1));
    speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/setFollowSpeed", 5, std::bind(&MpcController::setSpeedCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ly/navi/reached", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    previous_control_ = {0.0, 0.0};
}

void MpcController::loadParameters()
{
    params_.Ts = declare_parameter("Ts", Ts);
    params_.N = declare_parameter("N", N);
    params_.MinR = declare_parameter("MinR", MinR);
    params_.MaxLinear = declare_parameter("MaxLinear", MaxLinear);
    params_.MinLinear = declare_parameter("MinLinear", MinLinear);
    params_.MaxAngular = declare_parameter("MaxAngular", MaxAngular);
    params_.MinAngular = declare_parameter("MinAngular", MinAngular);
    params_.MaxAccelLinear = declare_parameter("MaxAccelLinear", MaxAccelLinear);
    params_.MinAccelLinear = declare_parameter("MinAccelLinear", MinAccelLinear);
    params_.MaxAccelAngular = declare_parameter("MaxAccelAngular", MaxAccelAngular);
    params_.MinAccelAngular = declare_parameter("MinAccelAngular", MinAccelAngular);
    params_.Wx = declare_parameter("Wx", Wx);
    params_.Wy = declare_parameter("Wy", Wy);
    params_.Wtheta = declare_parameter("Wtheta", Wtheta);
    params_.Wv = declare_parameter("Wv", Wv);
    params_.Womega = declare_parameter("Womega", Womega);
    params_.WRadius = declare_parameter("WRadius", WRadius);
    params_.Rv = declare_parameter("Rv", Rv);
    params_.Romega = declare_parameter("Romega", Romega);
    params_.TargetSpeed = declare_parameter("target_speed", 1.8);
    params_.ReachSlowDistance = declare_parameter("reach_slow_distance", 1.0);
    params_.ReachStopDistance = declare_parameter("reach_stop_distance", 0.3);
    params_.OptTolerance = declare_parameter("OptTolerance", OptTolerance);
    params_.GlobalMaxEval = declare_parameter("GlobalMaxEval", GlobalMaxEval);
    params_.LocalMaxEval = declare_parameter("LocalMaxEval", LocalMaxEval);
    params_.GlobalInitNoise = declare_parameter("GlobalInitNoise", GlobalInitNoise);
    params_.ObstacleInflation = declare_parameter("ObstacleInflation", ObstacleInflation);
    params_.Wobs = declare_parameter("Wobs", Wobs);
    params_.GlobalFrame = declare_parameter("global_frame", std::string("map"));
    params_.BaseFrame = declare_parameter("base_frame", std::string("base_link"));
}

void MpcController::configureGlobalOptimizer()
{
    global_optimizer_.set_min_objective(MpcController::objectiveFunction, this);

    std::vector<double> lb(2 * params_.N);
    std::vector<double> ub(2 * params_.N);
    for (int i = 0; i < params_.N; ++i)
    {
        lb[2 * i] = params_.MinLinear;
        ub[2 * i] = params_.MaxLinear;
        lb[2 * i + 1] = params_.MinLinear;
        ub[2 * i + 1] = params_.MaxLinear;
    }

    global_optimizer_.set_lower_bounds(lb);
    global_optimizer_.set_upper_bounds(ub);
    global_optimizer_.set_xtol_rel(1e-2);
    global_optimizer_.set_maxeval(params_.GlobalMaxEval);
}

void MpcController::configureLocalOptimizer()
{
    local_optimizer_.set_min_objective(MpcController::objectiveFunction, this);
    local_optimizer_.add_inequality_constraint(
        MpcController::accelerationConstraints, this, params_.OptTolerance);

    std::vector<double> lb(2 * params_.N);
    std::vector<double> ub(2 * params_.N);
    for (int i = 0; i < params_.N; ++i)
    {
        lb[2 * i] = params_.MinLinear;
        ub[2 * i] = params_.MaxLinear;
        lb[2 * i + 1] = params_.MinLinear;
        ub[2 * i + 1] = params_.MaxLinear;
    }

    local_optimizer_.set_lower_bounds(lb);
    local_optimizer_.set_upper_bounds(ub);
    local_optimizer_.set_xtol_rel(params_.OptTolerance);
    local_optimizer_.set_maxeval(params_.LocalMaxEval);
    local_optimizer_.set_param("initial_step", 0.01);
}

int MpcController::findNearestWaypoint(const RobotState &state, size_t last_index)
{
    if (reference_path_.empty())
        return 0;

    double min_dist = INFINITY;
    int nearest_idx = static_cast<int>(last_index);
    const int path_size = static_cast<int>(reference_path_.size());
    const int search_count = std::min(10, path_size);

    for (int i = 0; i < search_count; ++i)
    {
        const int idx = (static_cast<int>(last_index) + i) % path_size;
        const double dx = state.x - reference_path_[idx].x;
        const double dy = state.y - reference_path_[idx].y;
        const double dist = dx * dx + dy * dy;

        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_idx = idx;
        }
    }

    last_index_ = static_cast<size_t>(nearest_idx);
    return nearest_idx;
}

double MpcController::objectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    MpcController *mpc = static_cast<MpcController *>(data);
    const auto &params = mpc->params_;
    if (mpc->reference_path_.empty())
        return 0.0;

    double cost = 0.0;
    RobotState current = mpc->current_state_;
    const int nearest_idx = mpc->findNearestWaypoint(current, mpc->last_index_);
    mpc->last_index_ = static_cast<size_t>(nearest_idx);

    for (int i = 0; i < params.N; ++i)
    {
        Control u{x[2 * i], x[2 * i + 1]};
        current = mpc->predictState(current, u, params.Ts, mpc);

        const int ref_idx = (nearest_idx + i) % static_cast<int>(mpc->reference_path_.size());
        const PathPoint &ref = mpc->reference_path_[ref_idx];

        cost += params.Wx * std::pow(current.x - ref.x, 2);
        cost += params.Wy * std::pow(current.y - ref.y, 2);

        // 障碍物代价直接查询 costmap，不再依赖点云聚类结果。
        cost += mpc->costmapObstacleCost(current.x, current.y);

        const double speed = std::hypot(current.vx, current.vy);
        const double speed_error = std::hypot(u.vx, u.vy) - params.TargetSpeed;
        cost += params.Wv * std::exp(-speed);
        cost += params.Wv * speed_error * speed_error;

        if (i == 0)
        {
            cost += params.Rv * std::abs(u.vx - mpc->previous_control_.vx) / params.Ts;
            cost += params.Rv * std::abs(u.vy - mpc->previous_control_.vy) / params.Ts;
        }
        else
        {
            cost += params.Rv * std::abs(u.vx - x[2 * (i - 1)]) / params.Ts;
            cost += params.Rv * std::abs(u.vy - x[2 * (i - 1) + 1]) / params.Ts;
        }
    }

    if (!grad.empty() && mpc->local_optimizer_.get_algorithm() == nlopt::LD_SLSQP)
    {
        const double eps = 1e-3;
        std::vector<double> x_eps = x;
        std::vector<double> tmp;

        for (size_t i = 0; i < x.size(); ++i)
        {
            x_eps[i] += eps;
            const double f_plus = objectiveFunction(x_eps, tmp, data);
            x_eps[i] -= 2 * eps;
            const double f_minus = objectiveFunction(x_eps, tmp, data);
            grad[i] = (f_plus - f_minus) / (2 * eps);
            x_eps[i] = x[i];
        }
    }

    return cost;
}

double MpcController::accelerationConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    MpcController *mpc = static_cast<MpcController *>(data);
    const auto &params = mpc->params_;
    double max_violation = 0.0;

    const double dvx = x[0] - mpc->previous_control_.vx;
    const double dvy = x[1] - mpc->previous_control_.vy;
    const double accel_linear = std::hypot(dvx, dvy) / params.Ts;
    max_violation = std::max(max_violation, accel_linear - params.MaxAccelLinear);
    max_violation = std::max(max_violation, params.MinAccelLinear - accel_linear);

    for (int i = 1; i < params.N; ++i)
    {
        const double dvx_i = x[2 * i] - x[2 * (i - 1)];
        const double dvy_i = x[2 * i + 1] - x[2 * (i - 1) + 1];
        const double accel_linear_i = std::hypot(dvx_i, dvy_i) / params.Ts;
        max_violation = std::max(max_violation, accel_linear_i - params.MaxAccelLinear);
        max_violation = std::max(max_violation, params.MinAccelLinear - accel_linear_i);
    }

    if (!grad.empty() && mpc->local_optimizer_.get_algorithm() == nlopt::LD_SLSQP)
        std::fill(grad.begin(), grad.end(), 0.0);

    return max_violation;
}

double MpcController::turningRadiusConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    (void)x;
    (void)grad;
    (void)data;
    return 0.0;
}

RobotState MpcController::predictState(const RobotState &current, const Control &u, double dt, MpcController *mpc)
{
    (void)mpc;
    RobotState next;

    const double c = std::cos(current.theta);
    const double s = std::sin(current.theta);

    next.x = current.x + (u.vx * c - u.vy * s) * dt;
    next.y = current.y + (u.vx * s + u.vy * c) * dt;
    next.theta = current.theta;
    next.vx = u.vx;
    next.vy = u.vy;
    next.omega = 0.0;
    next.stamp = current.stamp;

    return next;
}

Control MpcController::computeControl()
{
    if (reference_path_.empty())
        return {0.0, 0.0};

    std::vector<double> global_x0(2 * params_.N);
    std::uniform_real_distribution<double> dist_v(-params_.GlobalInitNoise, params_.GlobalInitNoise);

    for (int i = 0; i < params_.N; ++i)
    {
        global_x0[2 * i] = clampValue(
            previous_control_.vx + dist_v(rng_), params_.MinLinear, params_.MaxLinear);
        global_x0[2 * i + 1] = clampValue(
            previous_control_.vy + dist_v(rng_), params_.MinLinear, params_.MaxLinear);
    }

    double global_fval = 0.0;
    try
    {
        const nlopt::result global_result = global_optimizer_.optimize(global_x0, global_fval);
        RCLCPP_DEBUG_STREAM(get_logger(), "Global Optimization: result=" << global_result << ", cost=" << global_fval);
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN_STREAM(get_logger(), "Global Optimization failed: " << e.what() << " (fallback to previous control)");
        for (int i = 0; i < params_.N; ++i)
        {
            global_x0[2 * i] = previous_control_.vx;
            global_x0[2 * i + 1] = previous_control_.vy;
        }
    }

    std::vector<double> local_x0 = global_x0;
    double local_fval = 0.0;
    nlopt::result local_result = nlopt::FAILURE;
    try
    {
        local_result = local_optimizer_.optimize(local_x0, local_fval);
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN_STREAM(get_logger(), "Local Optimization failed: " << e.what() << " (fallback to previous control)");
        return previous_control_;
    }

    if (local_result < 0 || std::isnan(local_fval) || std::isinf(local_fval))
    {
        RCLCPP_WARN(get_logger(), "Local Optimization produced invalid result (fallback to previous control)");
        return previous_control_;
    }

    opt_result_ = local_x0;
    opt_fval_ = local_fval;

    Control u;
    u.vx = clampValue(local_x0[0], params_.MinLinear, params_.MaxLinear);
    u.vy = clampValue(local_x0[1], params_.MinLinear, params_.MaxLinear);
    return u;
}

bool MpcController::updateRobotState()
{
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer_->lookupTransform(
            params_.GlobalFrame, params_.BaseFrame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "TF lookup failed (%s <- %s): %s",
            params_.GlobalFrame.c_str(), params_.BaseFrame.c_str(), ex.what());
        return false;
    }

    rclcpp::Time stamp(transform.header.stamp);
    if (stamp.nanoseconds() == 0)
    {
        stamp = this->now();
    }

    const double x = transform.transform.translation.x;
    const double y = transform.transform.translation.y;

    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    const double theta = tf2::getYaw(q);

    double vx_body = 0.0;
    double vy_body = 0.0;
    if (has_prev_tf_state_)
    {
        const double dt = (stamp - prev_tf_stamp_).seconds();
        if (dt > 1e-4)
        {
            // 先差分 map 系位置，再转到机器人 body 系。
            const double vx_map = (x - prev_tf_x_) / dt;
            const double vy_map = (y - prev_tf_y_) / dt;
            const double c = std::cos(theta);
            const double s = std::sin(theta);
            vx_body = c * vx_map + s * vy_map;
            vy_body = -s * vx_map + c * vy_map;
        }
    }

    current_state_.x = x;
    current_state_.y = y;
    current_state_.theta = theta;
    current_state_.vx = vx_body;
    current_state_.vy = vy_body;
    // omega 不是优化变量，这里不做 yaw 差分。
    current_state_.omega = 0.0;
    current_state_.stamp = stamp;

    prev_tf_x_ = x;
    prev_tf_y_ = y;
    prev_tf_stamp_ = stamp;
    has_prev_tf_state_ = true;
    has_state_ = true;
    return true;
}

void MpcController::onReferencePath(const nav_msgs::msg::Path::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received new reference path with %zu points", msg->poses.size());
    reference_path_.clear();

    for (const auto &pose_stamped : msg->poses)
    {
        PathPoint wp;
        wp.x = pose_stamped.pose.position.x;
        wp.y = pose_stamped.pose.position.y;
        tf2::Quaternion q(
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w);
        wp.theta = tf2::getYaw(q);
        reference_path_.push_back(wp);
    }

    last_index_ = 0;
    reached_flag_ = false;
}

int MpcController::normalizeCostmapCellToOcc100(int8_t raw_cell) const
{
    // 兼容标准 OccupancyGrid 和 Nav2 costmap 的 0..254/255 编码。
    if (raw_cell >= 0 && raw_cell <= 100)
    {
        return static_cast<int>(raw_cell);
    }

    const int cell_u8 = static_cast<int>(static_cast<uint8_t>(raw_cell));
    if (cell_u8 == 255)
    {
        return -1;
    }
    return static_cast<int>(std::round(static_cast<double>(cell_u8) * 100.0 / 254.0));
}

double MpcController::costmapObstacleCost(double x, double y) const
{
    // 在 ObstacleInflation 半径内取最大 cost，避免代价受分辨率影响。
    if (!has_costmap_ || latest_costmap_data_.empty() || latest_costmap_info_.resolution <= 0.0)
    {
        return 0.0;
    }

    const int width = static_cast<int>(latest_costmap_info_.width);
    const int height = static_cast<int>(latest_costmap_info_.height);
    if (width <= 0 || height <= 0 ||
        latest_costmap_data_.size() != static_cast<size_t>(width * height))
    {
        return 0.0;
    }

    const double resolution = static_cast<double>(latest_costmap_info_.resolution);
    const double origin_x = latest_costmap_info_.origin.position.x;
    const double origin_y = latest_costmap_info_.origin.position.y;
    const int center_col = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int center_row = static_cast<int>(std::floor((y - origin_y) / resolution));
    const int radius_cells =
        std::max(0, static_cast<int>(std::ceil(params_.ObstacleInflation / resolution)));

    int max_occ = 0;
    for (int dr = -radius_cells; dr <= radius_cells; ++dr)
    {
        for (int dc = -radius_cells; dc <= radius_cells; ++dc)
        {
            const double dist = std::hypot(static_cast<double>(dr), static_cast<double>(dc)) * resolution;
            if (dist > params_.ObstacleInflation)
            {
                continue;
            }

            const int row = center_row + dr;
            const int col = center_col + dc;
            // unknown 或越界区域按障碍处理，代价值为 100。
            int occ = 100;
            if (row >= 0 && row < height && col >= 0 && col < width)
            {
                occ = normalizeCostmapCellToOcc100(latest_costmap_data_[row * width + col]);
                if (occ < 0)
                {
                    occ = 100;
                }
            }
            max_occ = std::max(max_occ, occ);
        }
    }

    const double normalized_cost = static_cast<double>(max_occ) / 100.0;
    return params_.Wobs * normalized_cost * normalized_cost;
}

void MpcController::detectObstacles(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // 收到全量 costmap 时整体刷新本地缓存。
    const int width = static_cast<int>(msg->info.width);
    const int height = static_cast<int>(msg->info.height);
    if (width <= 0 || height <= 0 ||
        msg->data.size() != static_cast<size_t>(width * height))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Invalid costmap message: size=%dx%d data=%zu",
            width, height, msg->data.size());
        return;
    }

    latest_costmap_info_ = msg->info;
    latest_costmap_data_ = msg->data;
    has_costmap_ = true;

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received costmap: frame=%s size=%dx%d resolution=%.3f origin=(%.2f, %.2f)",
        msg->header.frame_id.c_str(), width, height,
        static_cast<double>(msg->info.resolution),
        msg->info.origin.position.x, msg->info.origin.position.y);
}

void MpcController::detectObstacleUpdates(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
{
    if (!has_costmap_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Skip costmap update: waiting for full /costmap/costmap message.");
        return;
    }

    const int map_width = static_cast<int>(latest_costmap_info_.width);
    const int map_height = static_cast<int>(latest_costmap_info_.height);
    const int update_x = static_cast<int>(msg->x);
    const int update_y = static_cast<int>(msg->y);
    const int update_width = static_cast<int>(msg->width);
    const int update_height = static_cast<int>(msg->height);
    const int expected_size = update_width * update_height;

    if (map_width <= 0 || map_height <= 0 ||
        latest_costmap_data_.size() != static_cast<size_t>(map_width * map_height) ||
        update_x < 0 || update_y < 0 || update_width <= 0 || update_height <= 0 ||
        update_x + update_width > map_width || update_y + update_height > map_height ||
        msg->data.size() != static_cast<size_t>(expected_size))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Invalid costmap update: region=(x%d,y%d,w%d,h%d) map=%dx%d data=%zu",
            update_x, update_y, update_width, update_height,
            map_width, map_height, msg->data.size());
        return;
    }

    for (int local_row = 0; local_row < update_height; ++local_row)
    {
        const int map_row = update_y + local_row;
        for (int local_col = 0; local_col < update_width; ++local_col)
        {
            // 将 costmap_updates 的局部窗口覆盖到本地全量缓存。
            const int map_col = update_x + local_col;
            latest_costmap_data_[map_row * map_width + map_col] =
                msg->data[local_row * update_width + local_col];
        }
    }
}

void MpcController::cmdVelPublish(const Control &u)
{
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = u.vx;
    cmd_vel_msg.linear.y = u.vy;
    cmd_vel_pub_->publish(cmd_vel_msg);
}

void MpcController::setSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    params_.TargetSpeed = clampValue(msg->data, 0.0, params_.MaxLinear);
}

Control MpcController::applySpeedLimit(const Control &u) const
{
    double speed_limit = params_.TargetSpeed;
    if (!reference_path_.empty() && has_state_ && params_.ReachSlowDistance > params_.ReachStopDistance)
    {
        const auto &goal = reference_path_.back();
        const double distance = std::hypot(current_state_.x - goal.x, current_state_.y - goal.y);
        if (distance < params_.ReachSlowDistance)
        {
            const double ratio = clampValue(
                (distance - params_.ReachStopDistance) /
                (params_.ReachSlowDistance - params_.ReachStopDistance),
                0.0,
                1.0);
            speed_limit *= ratio;
        }
    }

    const double speed = std::hypot(u.vx, u.vy);
    if (speed <= speed_limit || speed < 1e-6)
        return u;

    const double scale = speed_limit / speed;
    return {u.vx * scale, u.vy * scale};
}

void MpcController::updateReachedState()
{
    std_msgs::msg::Bool reached_msg;
    if (reference_path_.empty() || !has_state_)
    {
        reached_flag_ = false;
        reached_msg.data = reached_flag_;
        reached_pub_->publish(reached_msg);
        return;
    }

    const auto &goal = reference_path_.back();
    const double distance = std::hypot(current_state_.x - goal.x, current_state_.y - goal.y);
    reached_flag_ = distance < params_.ReachStopDistance;
    reached_msg.data = reached_flag_;
    reached_pub_->publish(reached_msg);
}

void MpcController::publishVisualization()
{
    visualization_msgs::msg::Marker predict_marker;
    predict_marker.header.frame_id = "map";
    predict_marker.header.stamp = this->now();
    predict_marker.ns = "mpc_predict";
    predict_marker.id = 0;
    predict_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
}

void MpcController::controlLoop()
{
    static rclcpp::Time last_state_stamp;

    if (!updateRobotState())
    {
        Control stop_cmd{0.0, 0.0};
        cmdVelPublish(stop_cmd);
        previous_control_ = stop_cmd;
        return;
    }

    updateReachedState();

    if (!has_state_ || reference_path_.empty() || reached_flag_)
    {
        Control stop_cmd{0.0, 0.0};
        cmdVelPublish(stop_cmd);
        previous_control_ = stop_cmd;
        return;
    }

    if (current_state_.stamp == last_state_stamp)
    {
        RCLCPP_WARN(get_logger(), "No new tf state data!");
        Control failed_cmd{0.0, 0.0};
        cmdVelPublish(failed_cmd);
        return;
    }
    last_state_stamp = current_state_.stamp;

    Control u = computeControl();
    u = applySpeedLimit(u);
    cmdVelPublish(u);
    previous_control_ = u;
}

}  // namespace mpc_follower
