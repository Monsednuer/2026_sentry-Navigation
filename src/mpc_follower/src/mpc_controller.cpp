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
    obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/dynamic_obstacles", 10, std::bind(&MpcController::detectObstacles, this, std::placeholders::_1));
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
    constexpr double kEps = 1e-6;

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

        for (const auto &obs : mpc->dynamic_obstacles_)
        {
            const double dx_obs = current.x - obs.x;
            const double dy_obs = current.y - obs.y;
            const double d_obs = std::hypot(dx_obs, dy_obs);
            const double safe_dist = params.ObstacleInflation + obs.radius;
            if (d_obs < safe_dist && d_obs > kEps)
            {
                cost += params.Wobs * std::exp(1.0 / std::abs(d_obs - safe_dist));
            }
        }

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

void MpcController::detectObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
    dynamic_obstacles_.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *raw);

    double L = 2.0;
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(raw);
    crop.setMin(Eigen::Vector4f(-L, -L, -1.0, 1.0));
    crop.setMax(Eigen::Vector4f(L, L, 2.0, 1.0));
    crop.setTranslation(Eigen::Vector3f(current_state_.x, current_state_.y, 0));
    crop.setRotation(Eigen::Vector3f(0, 0, 0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr windowed(new pcl::PointCloud<pcl::PointXYZ>());
    crop.filter(*windowed);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(windowed);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1, 2.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pass.filter(*filtered);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered);
    ec.extract(cluster_indices);

    for (const auto &indices : cluster_indices)
    {
        DynamicObstacle obs;
        double cx = 0.0;
        double cy = 0.0;
        double r = 0.0;

        for (int idx : indices.indices)
        {
            const auto &p = filtered->points[idx];
            cx += p.x;
            cy += p.y;
        }

        cx /= indices.indices.size();
        cy /= indices.indices.size();

        for (int idx : indices.indices)
        {
            const auto &p = filtered->points[idx];
            r = std::max(r, std::hypot(p.x - cx, p.y - cy));
        }

        obs.x = cx;
        obs.y = cy;
        obs.radius = r + 0.1;
        dynamic_obstacles_.push_back(obs);
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
