#include "follower.hpp"

using namespace path_follower;
using std::placeholders::_1;

Follower::Follower() : Node("follower_node"), 
                       control_dt_(0.01), 
                       speed_dt_(0.1), 
                       speed_ratio_(1.0),


                       
                       current_index_(0),
                       lookahead_distance_(0.0),
                       target_velocity_(Eigen::Vector2d::Zero()),
                       control_velocity_(Eigen::Vector2d::Zero())
{
    // Declare and get parameters
    kp_ = this->declare_parameter("kp", 1.0);
    ki_ = this->declare_parameter("ki", 0.0);
    kd_ = this->declare_parameter("kd", 1.0);
    k_lookahead_ = this->declare_parameter("k_lookahead", 1.0);
    min_lookahead_ = this->declare_parameter("min_lookahead", 0.2);
    target_speed_ = this->declare_parameter("target_speed", 2.0);
    //TODO TEST
    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();

    k_lookahead_ = this->get_parameter("k_lookahead").as_double();
    min_lookahead_ = this->get_parameter("min_lookahead").as_double();
    target_speed_ = this->get_parameter("target_speed").as_double();


    RCLCPP_WARN(this->get_logger(), "Speed PID: %f, %f, %f", kp_, ki_, kd_);
    RCLCPP_WARN(this->get_logger(), "Lookahead: %f, %f", k_lookahead_, min_lookahead_);
    RCLCPP_WARN(this->get_logger(), "Target Speed: %f", target_speed_);

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    sub_global_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "/sPath", 5, std::bind(&Follower::globalPathCallback, this, _1));
    sub_set_speed_ = this->create_subscription<std_msgs::msg::Float64>(
        "/setFollowSpeed", 5, std::bind(&Follower::setSpeedCallback, this, _1));

    // Publishers
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("follower/visualization", 1);
    pub_reached_ = this->create_publisher<std_msgs::msg::Bool>("/ly/navi/reached", 10);

    // Timers
    auto control_ms = std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000.0));
    timer_manager_ = this->create_wall_timer(control_ms, std::bind(&Follower::manager, this));

    auto speed_ms = std::chrono::milliseconds(static_cast<int>(speed_dt_ * 1000.0));
    timer_get_current_velocity_ = this->create_wall_timer(speed_ms, std::bind(&Follower::getCurrentVelocity, this));
}

void Follower::manager()
{
    RCLCPP_DEBUG(this->get_logger(), "Manager");
    if (global_path_.empty())
    {
        // RCLCPP_WARN(this->get_logger(), "No Global Path");
        return;
    }

    if (!getTransform())
    {
        RCLCPP_WARN(this->get_logger(), "No Transform so no following");
        return;
    }
    
    getLookaheadDistance();
    getLocalPath();
    checkReached();
    getCurrentIndex();
    getTargetVelocity();
    follow();
    control();
    publish();
    visualize();
}

void Follower::getLookaheadDistance()
{
    RCLCPP_DEBUG(this->get_logger(), "Get Lookahead Distance");
    lookahead_distance_ = std::max(min_lookahead_, k_lookahead_ * current_velocity_local_.norm());
    RCLCPP_INFO(this->get_logger(), "Lookahead Distance: %f", lookahead_distance_);
}

void Follower::getCurrentVelocity()
{
    // RCLCPP_DEBUG(this->get_logger(), "Get Current Velocity");
    // static Eigen::Vector2d last_position_global = Eigen::Vector2d::Zero();
    // Eigen::Vector2d current_position_global = Eigen::Vector2d::Zero();
    // if (!getTransform())
    // {
    //     RCLCPP_WARN(this->get_logger(), "No Transform so no current velocity");
    //     return;
    // }
    // current_position_global = map_baselink_tf_.block<2, 1>(0, 3);

    // auto current_velocity_global = (current_position_global - last_position_global) / speed_dt_;
    // current_velocity_local_ = map_baselink_tf_.block<2, 2>(0, 0).transpose() * current_velocity_global;

    // last_position_global = current_position_global;
}

bool Follower::getTransform()
{
    RCLCPP_DEBUG(this->get_logger(), "Get Transform");
    geometry_msgs::msg::TransformStamped robot_global_pose;
    try
    {
        // tf2::TimePointZero gives the latest available transform
        robot_global_pose = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

        map_baselink_tf_ = Eigen::MatrixXd::Zero(4, 4);
        map_baselink_tf_(3, 3) = 1;
        map_baselink_tf_(0, 3) = robot_global_pose.transform.translation.x;
        map_baselink_tf_(1, 3) = robot_global_pose.transform.translation.y;
        map_baselink_tf_(2, 3) = 0;

        Eigen::Quaterniond q;
        q.x() = robot_global_pose.transform.rotation.x;
        q.y() = robot_global_pose.transform.rotation.y;
        q.z() = robot_global_pose.transform.rotation.z;
        q.w() = robot_global_pose.transform.rotation.w;

        Eigen::Matrix3d R = q.normalized().toRotationMatrix();

        map_baselink_tf_.block<3, 3>(0, 0) = R;
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return false;
    }
}

void Follower::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Global Path Callback");
    global_path_.clear();
    for (const auto &pose : msg->poses)
    {
        global_path_.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
    }
}

void Follower::setSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    this->target_speed_ = msg->data;
}

void Follower::getLocalPath()
{
    RCLCPP_DEBUG(this->get_logger(), "Get Local Path");
    local_path_.clear();
    for (const auto &point : global_path_)
    {
        Eigen::Vector4d p_temp = {point[0], point[1], 0, 1};
        p_temp = map_baselink_tf_.inverse() * p_temp;
        local_path_.push_back(Eigen::Vector2d(p_temp[0], p_temp[1]));
    }
}

void Follower::getCurrentIndex()
{
    RCLCPP_DEBUG(this->get_logger(), "Get Current Index");
    auto current_position = Eigen::Vector2d(0, 0);
    double min_distance = INFINITY;
    int min_index = 0;
    for (size_t i = 0; i < local_path_.size(); i++)
    {
        double distance = (local_path_[i] - current_position).norm();
        if (distance < min_distance)
        {
            min_distance = distance;
            min_index = i;
        }
    }
    current_index_ = min_index;
    RCLCPP_DEBUG(this->get_logger(), "Current Index: %d", current_index_);
}

void Follower::checkReached()
{
    RCLCPP_DEBUG(this->get_logger(), "Check Reached");
    auto current_position = Eigen::Vector2d(0, 0);
    
    // Printing Eigen Vector directly via stream
    RCLCPP_INFO_STREAM(this->get_logger(), "Target end: [" << local_path_.back()[0] << ", " << local_path_.back()[1] << "]");
    
    auto distance = (local_path_.back() - current_position).norm();
    if(distance < 1.0)
    {
        if (distance < 0.3)
        {
            RCLCPP_INFO(this->get_logger(), "Reached");
            reached_flag_ = true;
            distance = 0;
        }
        speed_ratio_ = distance;
        reached_flag_ = false;
        RCLCPP_INFO(this->get_logger(), "Almost reach, ratio: %f", speed_ratio_);
    }
    else
    {
        speed_ratio_ = 1;
        reached_flag_ = false;
    }
}

void Follower::getTargetVelocity()
{
    RCLCPP_DEBUG(this->get_logger(), "Get Target Velocity");
    auto current_position = Eigen::Vector2d(0, 0);
    int target_index = current_index_;
    
    for (size_t i = current_index_; i < local_path_.size(); i++)
    {
        double distance = (local_path_[i] - current_position).norm();
        if (distance > lookahead_distance_)
        {
            target_index = i;
            break;
        }
    }
    
    target_goal_local_ = local_path_[target_index] - current_position;
    auto target_distance = target_goal_local_.norm();
    
    if (target_distance > 3.0)
    {
        target_velocity_ = Eigen::Vector2d(0, 0);
        RCLCPP_ERROR(this->get_logger(), "Bad distance to path! Stopped.");
        return;
    }
    
    if (target_distance > lookahead_distance_)
    {
        target_velocity_ = target_goal_local_ / target_distance * target_speed_ * speed_ratio_;
    }
    else
    {
        target_velocity_ = target_goal_local_ / lookahead_distance_ * target_speed_ * speed_ratio_;
    }
}

void Follower::follow()
{
    RCLCPP_DEBUG(this->get_logger(), "Follow");

    current_velocity_direction_ = target_velocity_.norm() == 0 ? Eigen::Vector2d(0, 1) : target_velocity_.normalized();
    follow_velocity_ = target_velocity_;
}

void Follower::control()
{
    RCLCPP_DEBUG(this->get_logger(), "Control");

    static double last_error = 0, last_last_error = 0;
    static double last_out = 0;

    double target_speed = target_velocity_.norm();
    double current_speed = current_velocity_local_.norm();

    double error = target_speed - current_speed;

    double d_u = kp_*(error - last_error) + ki_ * error + kd_ * (error - 2 * last_error + last_last_error);

    last_last_error = last_error;
    last_error = error;

    double out = last_out + d_u;
    out = std::min(std::max(-1.2*target_speed, out), 1.2*target_speed);
    last_out = out;

    control_velocity_ = out * (target_velocity_.norm() > 0 ? target_velocity_.normalized() : Eigen::Vector2d(0, 1));
    // Overwriting control velocity according to original script intent
    control_velocity_ = target_velocity_; 
}

void Follower::publish()
{
    RCLCPP_DEBUG(this->get_logger(), "Publish");
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = control_velocity_[0];
    cmd_vel.linear.y = control_velocity_[1];
    // cmd_vel.linear.x = 1.0;
    // // cmd_vel.linear.y = -1.0;
    pub_cmd_vel_->publish(cmd_vel);
    auto reached_msg = std_msgs::msg::Bool();
    reached_msg.data = reached_flag_;
    pub_reached_->publish(reached_msg);
}

void Follower::visualize()
{
    if (local_path_.empty() || current_index_ >= (int)local_path_.size())
        return;

    rclcpp::Time now = this->now();

    // 1. Current Point Marker
    visualization_msgs::msg::Marker current_point_marker;
    current_point_marker.header.frame_id = "base_link";
    current_point_marker.header.stamp = now;
    current_point_marker.ns = "follower";
    current_point_marker.id = 0;
    current_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
    current_point_marker.action = visualization_msgs::msg::Marker::ADD;
    current_point_marker.pose.position.x = local_path_[current_index_][0];
    current_point_marker.pose.position.y = local_path_[current_index_][1];
    current_point_marker.pose.position.z = 0;
    current_point_marker.scale.x = 0.2;
    current_point_marker.scale.y = 0.2;
    current_point_marker.scale.z = 0.2;
    current_point_marker.color.r = 1.0;
    current_point_marker.color.g = 0.0;
    current_point_marker.color.b = 0.0;
    current_point_marker.color.a = 1.0;

    pub_marker_->publish(current_point_marker);
    
    // Target Point Marker
    visualization_msgs::msg::Marker target_point_marker;
    target_point_marker.header.frame_id = "base_link";
    target_point_marker.header.stamp = now;
    target_point_marker.ns = "follower";
    target_point_marker.id = 2;
    target_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_point_marker.action = visualization_msgs::msg::Marker::ADD;
    target_point_marker.pose.position.x = target_goal_local_[0];
    target_point_marker.pose.position.y = target_goal_local_[1];
    target_point_marker.pose.position.z = 0;
    target_point_marker.scale.x = 0.2;
    target_point_marker.scale.y = 0.2;
    target_point_marker.scale.z = 0.2;
    target_point_marker.color.r = 0.0;
    target_point_marker.color.g = 1.0;
    target_point_marker.color.b = 0.0;
    target_point_marker.color.a = 1.0;

    pub_marker_->publish(target_point_marker);

    // 2. Velocity Vector Arrow
    visualization_msgs::msg::Marker velocity_arrow;
    velocity_arrow.header.frame_id = "base_link";
    velocity_arrow.header.stamp = now;
    velocity_arrow.ns = "follower";
    velocity_arrow.id = 1;
    velocity_arrow.type = visualization_msgs::msg::Marker::ARROW;
    velocity_arrow.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;
    start.x = 0;
    start.y = 0;
    start.z = 0;
    end.x = control_velocity_[0];
    end.y = control_velocity_[1];
    end.z = 0;

    velocity_arrow.points.push_back(start);
    velocity_arrow.points.push_back(end);
    velocity_arrow.scale.x = 0.05; 
    velocity_arrow.scale.y = 0.1;  
    velocity_arrow.scale.z = 0.1;  
    velocity_arrow.color.r = 1.0;
    velocity_arrow.color.g = 0.0;
    velocity_arrow.color.b = 0.0;
    velocity_arrow.color.a = 1.0;

    pub_marker_->publish(velocity_arrow);
}

// Optional: Main wrapper if you compile this directly as an executable instead of a component
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_follower::Follower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// #include "follower.hpp"

// using namespace path_follower;
// using std::placeholders::_1;

// Follower::Follower() : Node("follower_node"), 
//                        control_dt_(0.01), 
//                        speed_dt_(0.1), 
//                        speed_ratio_(1.0),
//                        current_index_(0),
//                        lookahead_distance_(0.0),
//                        target_velocity_(Eigen::Vector2d::Zero()),
//                        control_velocity_(Eigen::Vector2d::Zero())
// {
//     kp_ = this->declare_parameter("kp", 1.0);
//     ki_ = this->declare_parameter("ki", 0.0);
//     kd_ = this->declare_parameter("kd", 1.0);
//     k_lookahead_ = this->declare_parameter("k_lookahead", 1.0);
//     min_lookahead_ = this->declare_parameter("min_lookahead", 0.2);
//     target_speed_ = this->declare_parameter("target_speed", 2.0);

//     kp_ = this->get_parameter("kp").as_double();
//     ki_ = this->get_parameter("ki").as_double();
//     kd_ = this->get_parameter("kd").as_double();
//     k_lookahead_ = this->get_parameter("k_lookahead").as_double();
//     min_lookahead_ = this->get_parameter("min_lookahead").as_double();
//     target_speed_ = this->get_parameter("target_speed").as_double();

//     RCLCPP_WARN(this->get_logger(), "Speed PID: %f, %f, %f", kp_, ki_, kd_);
//     RCLCPP_WARN(this->get_logger(), "Lookahead: %f, %f", k_lookahead_, min_lookahead_);
//     RCLCPP_WARN(this->get_logger(), "Target Speed: %f", target_speed_);

//     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//     sub_global_path_ = this->create_subscription<nav_msgs::msg::Path>(
//         "/sPath", 5, std::bind(&Follower::globalPathCallback, this, _1));
//     sub_set_speed_ = this->create_subscription<std_msgs::msg::Float64>(
//         "/setFollowSpeed", 5, std::bind(&Follower::setSpeedCallback, this, _1));

//     pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//     pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("follower/visualization", 1);
//     //TEST
//     pub_vel_dir_map_ = this->create_publisher<geometry_msgs::msg::Vector3>("velocity_dir_map", 10);

//     auto control_ms = std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000.0));
//     timer_manager_ = this->create_wall_timer(control_ms, std::bind(&Follower::manager, this));

//     auto speed_ms = std::chrono::milliseconds(static_cast<int>(speed_dt_ * 1000.0));
//     timer_get_current_velocity_ = this->create_wall_timer(speed_ms, std::bind(&Follower::getCurrentVelocity, this));
// }

// void Follower::manager()
// {
//     RCLCPP_DEBUG(this->get_logger(), "Manager");
//     if (global_path_.empty())
//     {
//         return;
//     }

//     if (!getTransform())
//     {
//         RCLCPP_WARN(this->get_logger(), "No Transform so no following");
//         return;
//     }
    
//     getLookaheadDistance();
//     getCurrentIndex();
//     getTargetVelocity();
//     checkReached();
//     follow();
//     control();
//     publish();
//     visualize();
// }

// void Follower::getLookaheadDistance()
// {
//     RCLCPP_DEBUG(this->get_logger(), "Get Lookahead Distance");
//     lookahead_distance_ = std::max(min_lookahead_, k_lookahead_ * current_velocity_local_.norm());
//     RCLCPP_INFO(this->get_logger(), "Lookahead Distance: %f", lookahead_distance_);
// }

// void Follower::getCurrentVelocity()
// {
//     RCLCPP_DEBUG(this->get_logger(), "Get Current Velocity");
//     static Eigen::Vector2d last_position_global = Eigen::Vector2d::Zero();
//     Eigen::Vector2d current_position_global = Eigen::Vector2d::Zero();
//     if (!getTransform())
//     {
//         RCLCPP_WARN(this->get_logger(), "No Transform so no current velocity");
//         return;
//     }
//     current_position_global = map_baselink_tf_.block<2, 1>(0, 3);

//     auto current_velocity_global = (current_position_global - last_position_global) / speed_dt_;
//     current_velocity_local_ = map_baselink_tf_.block<2, 2>(0, 0).transpose() * current_velocity_global;

//     last_position_global = current_position_global;
// }

// // bool Follower::getTransform()
// // {
// //     RCLCPP_DEBUG(this->get_logger(), "Get Transform");
// //     geometry_msgs::msg::TransformStamped robot_global_pose;
// //     try
// //     {
// //         robot_global_pose = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

// //         map_baselink_tf_ = Eigen::MatrixXd::Zero(4, 4);
// //         map_baselink_tf_(3, 3) = 1;
// //         map_baselink_tf_(0, 3) = robot_global_pose.transform.translation.x;
// //         map_baselink_tf_(1, 3) = robot_global_pose.transform.translation.y;
// //         map_baselink_tf_(2, 3) = 0;

// //         Eigen::Quaterniond q;
// //         q.x() = robot_global_pose.transform.rotation.x;
// //         q.y() = robot_global_pose.transform.rotation.y;
// //         q.z() = robot_global_pose.transform.rotation.z;
// //         q.w() = robot_global_pose.transform.rotation.w;

// //         Eigen::Matrix3d R = q.normalized().toRotationMatrix();
// //         map_baselink_tf_.block<3, 3>(0, 0) = R;
// //         return true;
// //     }
// //     catch (const tf2::TransformException &ex)
// //     {
// //         RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
// //         return false;
// //     }
// // }

// //TEST
// bool Follower::getTransform()
// {
//     RCLCPP_DEBUG(this->get_logger(), "Get Transform");
//     geometry_msgs::msg::TransformStamped robot_global_pose;
    
//     static rclcpp::Time last_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);  // 上一帧时间戳
    
//     try
//     {
//         // 查最新可用变换
//         robot_global_pose = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        
//         rclcpp::Time current_stamp = robot_global_pose.header.stamp;
        
//         // 时间戳检查：拒绝早于上一帧的数据（乱序/回溯）
//         if (current_stamp < last_stamp) {
//             RCLCPP_WARN(this->get_logger(), "TF timestamp rollback: %.3f < %.3f, skip frame", 
//                         current_stamp.seconds(), last_stamp.seconds());
//             return false;
//         }
        
//         // 可选：检查延迟，如果 TF 比当前时间落后超过 0.5s，认为太旧
//         rclcpp::Time now = this->now();
//         if ((now - current_stamp).seconds() > 0.5) {
//             RCLCPP_WARN(this->get_logger(), "TF too old: %.3f s delay, skip frame", 
//                         (now - current_stamp).seconds());
//             return false;
//         }
        
//         last_stamp = current_stamp;

//         // ... 原有矩阵转换逻辑不变
//         map_baselink_tf_ = Eigen::MatrixXd::Zero(4, 4);
//         map_baselink_tf_(3, 3) = 1;
//         map_baselink_tf_(0, 3) = robot_global_pose.transform.translation.x;
//         map_baselink_tf_(1, 3) = robot_global_pose.transform.translation.y;
//         map_baselink_tf_(2, 3) = 0;

//         Eigen::Quaterniond q;
//         q.x() = robot_global_pose.transform.rotation.x;
//         q.y() = robot_global_pose.transform.rotation.y;
//         q.z() = robot_global_pose.transform.rotation.z;
//         q.w() = robot_global_pose.transform.rotation.w;

//         Eigen::Matrix3d R = q.normalized().toRotationMatrix();
//         map_baselink_tf_.block<3, 3>(0, 0) = R;
        
//         return true;
//     }
//     catch (const tf2::TransformException &ex)
//     {
//         RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
//         return false;
//     }
// }

// void Follower::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
// {
//     RCLCPP_DEBUG(this->get_logger(), "Global Path Callback");
//     global_path_.clear();
//     for (const auto &pose : msg->poses)
//     {
//         global_path_.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
//     }
// }

// void Follower::setSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
// {
//     this->target_speed_ = msg->data;
// }

// void Follower::getCurrentIndex()
// {
//     RCLCPP_DEBUG(this->get_logger(), "Get Current Index");
//     auto current_position_global = map_baselink_tf_.block<2, 1>(0, 3);
    
//     double min_distance = INFINITY;
//     int min_index = 0;
//     for (size_t i = 0; i < global_path_.size(); i++)
//     {
//         double distance = (global_path_[i] - current_position_global).norm();
//         if (distance < min_distance)
//         {
//             min_distance = distance;
//             min_index = i;
//         }
//     }
//     current_index_ = min_index;
//     RCLCPP_DEBUG(this->get_logger(), "Current Index: %d", current_index_);
// }

// void Follower::checkReached()
// {
//     RCLCPP_DEBUG(this->get_logger(), "Check Reached");
//     auto current_position_global = map_baselink_tf_.block<2, 1>(0, 3);
    
//     auto distance = (global_path_.back() - current_position_global).norm();
    
//     RCLCPP_INFO_STREAM(this->get_logger(), "Target end global: [" << global_path_.back()[0] << ", " << global_path_.back()[1] << "]");
//     RCLCPP_INFO_STREAM(this->get_logger(), "Current global: [" << current_position_global[0] << ", " << current_position_global[1] << "]");
//     RCLCPP_INFO(this->get_logger(), "Distance to end: %f", distance);
    
//     if(distance < 1.0)
//     {
//         if (distance < 0.3)
//         {
//             RCLCPP_INFO(this->get_logger(), "Reached");
//             distance = 0;
//         }
//         speed_ratio_ = distance;
//         RCLCPP_INFO(this->get_logger(), "Almost reach, ratio: %f", speed_ratio_);
//     }
//     else
//     {
//         speed_ratio_ = 1;
//     }
// }

// void Follower::getTargetVelocity()
// {
//     RCLCPP_DEBUG(this->get_logger(), "Get Target Velocity");
//     auto current_position_global = map_baselink_tf_.block<2, 1>(0, 3);
    
//     int target_index = current_index_;
    
//     for (size_t i = current_index_; i < global_path_.size(); i++)
//     {
//         double distance = (global_path_[i] - current_position_global).norm();
//         if (distance > lookahead_distance_)
//         {
//             target_index = i;
//             break;
//         }
//     }
    
//     Eigen::Vector2d target_global = global_path_[target_index];
    
//     Eigen::Vector4d p_temp = {target_global[0], target_global[1], 0, 1};
//     p_temp = map_baselink_tf_.inverse() * p_temp;
//     target_goal_local_ = Eigen::Vector2d(p_temp[0], p_temp[1]);
    
//     auto target_distance = target_goal_local_.norm();
    
//     if (target_distance > 3.0)
//     {
//         target_velocity_ = Eigen::Vector2d(0, 0);
//         RCLCPP_ERROR(this->get_logger(), "Bad distance to path! Stopped.");
//         return;
//     }
    
//     if (target_distance > lookahead_distance_)
//     {
//         target_velocity_ = target_goal_local_ / target_distance * target_speed_ * speed_ratio_;
//     }
//     else
//     {
//         target_velocity_ = target_goal_local_ / lookahead_distance_ * target_speed_ * speed_ratio_;
//     }
// }

// void Follower::follow()
// {
//     RCLCPP_DEBUG(this->get_logger(), "Follow");
//     current_velocity_direction_ = target_velocity_.norm() == 0 ? Eigen::Vector2d(0, 1) : target_velocity_.normalized();
//     follow_velocity_ = target_velocity_;
// }

// void Follower::control()
// {
//     RCLCPP_DEBUG(this->get_logger(), "Control");

//     static double last_error = 0, last_last_error = 0;
//     static double last_out = 0;

//     double target_speed = target_velocity_.norm();
//     double current_speed = current_velocity_local_.norm();

//     double error = target_speed - current_speed;

//     double d_u = kp_*(error - last_error) + ki_ * error + kd_ * (error - 2 * last_error + last_last_error);

//     last_last_error = last_error;
//     last_error = error;

//     double out = last_out + d_u;
//     out = std::min(std::max(-1.2*target_speed, out), 1.2*target_speed);
//     last_out = out;

//     control_velocity_ = out * (target_velocity_.norm() > 0 ? target_velocity_.normalized() : Eigen::Vector2d(0, 1));
//     // control_velocity_ = target_velocity_; 

//      // 输出速度方向（map系下的单位向量）
//     if (control_velocity_.norm() > 0.01) {
//     Eigen::Vector2d dir_local = control_velocity_.normalized();
//     Eigen::Matrix2d R = map_baselink_tf_.block<2, 2>(0, 0);
//     Eigen::Vector2d dir_map = R * dir_local;
    
//     geometry_msgs::msg::Vector3 msg;
//     msg.x = dir_map[0];
//     msg.y = dir_map[1];
//     msg.z = 0.0;
//     pub_vel_dir_map_->publish(msg);
// }
// }

// void Follower::publish()
// {
//     RCLCPP_DEBUG(this->get_logger(), "Publish");
//     geometry_msgs::msg::Twist cmd_vel;
//     cmd_vel.linear.x = control_velocity_[0];
//     cmd_vel.linear.y = control_velocity_[1];
//     pub_cmd_vel_->publish(cmd_vel);
// }

// void Follower::visualize()
// {
//     if (global_path_.empty() || current_index_ >= (int)global_path_.size())
//         return;

//     rclcpp::Time now = this->now();

//     // Current Point Marker (global 转 baselink 显示)
//     Eigen::Vector2d current_global = global_path_[current_index_];
//     Eigen::Vector4d p_temp = {current_global[0], current_global[1], 0, 1};
//     p_temp = map_baselink_tf_.inverse() * p_temp;
    
//     visualization_msgs::msg::Marker current_point_marker;
//     current_point_marker.header.frame_id = "base_link";
//     current_point_marker.header.stamp = now;
//     current_point_marker.ns = "follower";
//     current_point_marker.id = 0;
//     current_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
//     current_point_marker.action = visualization_msgs::msg::Marker::ADD;
//     current_point_marker.pose.position.x = p_temp[0];
//     current_point_marker.pose.position.y = p_temp[1];
//     current_point_marker.pose.position.z = 0;
//     current_point_marker.scale.x = 0.2;
//     current_point_marker.scale.y = 0.2;
//     current_point_marker.scale.z = 0.2;
//     current_point_marker.color.r = 1.0;
//     current_point_marker.color.g = 0.0;
//     current_point_marker.color.b = 0.0;
//     current_point_marker.color.a = 1.0;
//     pub_marker_->publish(current_point_marker);
    
//     // Target Point Marker
//     visualization_msgs::msg::Marker target_point_marker;
//     target_point_marker.header.frame_id = "base_link";
//     target_point_marker.header.stamp = now;
//     target_point_marker.ns = "follower";
//     target_point_marker.id = 2;
//     target_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
//     target_point_marker.action = visualization_msgs::msg::Marker::ADD;
//     target_point_marker.pose.position.x = target_goal_local_[0];
//     target_point_marker.pose.position.y = target_goal_local_[1];
//     target_point_marker.pose.position.z = 0;
//     target_point_marker.scale.x = 0.2;
//     target_point_marker.scale.y = 0.2;
//     target_point_marker.scale.z = 0.2;
//     target_point_marker.color.r = 0.0;
//     target_point_marker.color.g = 1.0;
//     target_point_marker.color.b = 0.0;
//     target_point_marker.color.a = 1.0;
//     pub_marker_->publish(target_point_marker);

//     // Velocity Vector Arrow
//     visualization_msgs::msg::Marker velocity_arrow;
//     velocity_arrow.header.frame_id = "base_link";
//     velocity_arrow.header.stamp = now;
//     velocity_arrow.ns = "follower";
//     velocity_arrow.id = 1;
//     velocity_arrow.type = visualization_msgs::msg::Marker::ARROW;
//     velocity_arrow.action = visualization_msgs::msg::Marker::ADD;

//     geometry_msgs::msg::Point start, end;
//     start.x = 0;
//     start.y = 0;
//     start.z = 0;
//     end.x = control_velocity_[0];
//     end.y = control_velocity_[1];
//     end.z = 0;

//     velocity_arrow.points.push_back(start);
//     velocity_arrow.points.push_back(end);
//     velocity_arrow.scale.x = 0.05; 
//     velocity_arrow.scale.y = 0.1;  
//     velocity_arrow.scale.z = 0.1;  
//     velocity_arrow.color.r = 1.0;
//     velocity_arrow.color.g = 0.0;
//     velocity_arrow.color.b = 0.0;
//     velocity_arrow.color.a = 1.0;
//     pub_marker_->publish(velocity_arrow);
// }

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<path_follower::Follower>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }