#include "ee4308_turtle/controller.hpp"

namespace ee4308::turtle
{
    void Controller::cleanup() { RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::activate() { RCLCPP_INFO_STREAM(node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::deactivate() { RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { global_plan_ = path; }

    // ====================================== LAB 1, PROJ 1 ====================================================

    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        tf_ = tf;
        plugin_name_ = name;

        // initialize parameters
        initParam(node_, plugin_name_ + ".desired_linear_vel", desired_linear_vel_, 0.18);
        initParam(node_, plugin_name_ + ".desired_lookahead_dist", desired_lookahead_dist_, 0.4);
        initParam(node_, plugin_name_ + ".max_angular_vel", max_angular_vel_, 1.0);
        initParam(node_, plugin_name_ + ".max_linear_vel", max_linear_vel_, 0.22);
        initParam(node_, plugin_name_ + ".xy_goal_thres", xy_goal_thres_, 0.05);
        initParam(node_, plugin_name_ + ".yaw_goal_thres", yaw_goal_thres_, 0.25);
        initParam(node_, plugin_name_ + ".curvature_threshold", curvature_threshold, 1.5);
        initParam(node_, plugin_name_ + ".proximity_threshold", proximity_threshold, 0.8);
        initParam(node_, plugin_name_ + ".lookahead_gain", lookahead_gain, 5.0);

        adjusted_lookahead = desired_lookahead_dist_;

        //initialize topics
        sub_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::scannerCallback, this, std::placeholders::_1));
    }

    void Controller::scannerCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ranges_ = msg->ranges;
    }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        // get goal pose (contains the "clicked" goal rotation and position)
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();
        
        double goal_x = goal_pose.pose.position.x;
        double goal_y = goal_pose.pose.position.y;
        double goal_yaw = ee4308::getYawFromQuaternion(goal_pose.pose.orientation);
        //std::cout << "goal coordinates: " << goal_x << " " << goal_y << std::endl;

        double robot_x = pose.pose.position.x; // position x of the robot
        double robot_y = pose.pose.position.y; // position y of the robot
        double phi = ee4308::getYawFromQuaternion(pose.pose.orientation); // orientation of the robot

        // check if robot is close to the goal (both dist and yaw)
        double dist_to_goal = std::hypot(goal_x - robot_x, goal_y - robot_y);
        double diff_yaw = std::abs(ee4308::limitAngle(goal_yaw - phi));

        // if close to goal, then stop
        if (dist_to_goal < xy_goal_thres_)
        {
            // if robot's heading is far from goal orientation, then
            if (diff_yaw > yaw_goal_thres_)
            {
                return writeCmdVel(0, max_angular_vel_); 
            }
            return writeCmdVel(0, 0);
        }

        // get closest point
        size_t closest_index = 0;
        double shortest_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < global_plan_.poses.size(); i++)
        {
            double point_pose_x = global_plan_.poses[i].pose.position.x; 
            double point_pose_y = global_plan_.poses[i].pose.position.y;
            double dist = std::hypot((point_pose_x - robot_x),(point_pose_y - robot_y));
            //std::cout << "calculated distance" << dist << std::endl;
            if (dist < shortest_dist) 
            {
                shortest_dist = dist;
                closest_index = i;
            }
        }

        // get lookahead
        geometry_msgs::msg::PoseStamped lookahead_pose = goal_pose;

        // Determine lookahead point
        //std::cout << "current lookahead distance: " << current_lookahead_dist_ << std::endl;
        for (size_t j = closest_index + 1; j < global_plan_.poses.size(); j++)
        {
            double point_pose_x = global_plan_.poses[j].pose.position.x; 
            double point_pose_y = global_plan_.poses[j].pose.position.y;
            double dist_from_robot = std::hypot((point_pose_x - robot_x),(point_pose_y - robot_y));
            //std::cout << "dist_from_robot: " << dist_from_robot << std::endl;

            if (dist_from_robot >= adjusted_lookahead) 
            {
                lookahead_pose = global_plan_.poses[j];
                break;
            }
        }

        // Calculate Lookahead Point in Robot Frame
        double delta_x = lookahead_pose.pose.position.x - robot_x;
        double delta_y = lookahead_pose.pose.position.y - robot_y;

        double x_r = delta_x * std::cos(phi) + delta_y * std::sin(phi); 
        double y_r = delta_y * std::cos(phi) - delta_x * std::sin(phi);

        // calculate curvature
        double d_squared = std::pow(x_r, 2) + std::pow(y_r, 2);
        double curvature = 2 * y_r / d_squared;
        //std::cout << "curvature: " << curvature << std::endl;

        double angular_vel = desired_linear_vel_ * curvature;

        // Curvature Heuristic
        double regulated_linear_vel = desired_linear_vel_;

        if (curvature_threshold < std::abs(curvature))
        {
            regulated_linear_vel = desired_linear_vel_ * (curvature_threshold / std::abs(curvature));
        }

        // Find distance to the closest obstacle
        double min_obstacle_dist = std::numeric_limits<double>::max();

        for (const auto &range: scan_ranges_)
        {
            if (range < min_obstacle_dist && std::isfinite(range))
            {
                min_obstacle_dist = range;
            }
        }

        // Obstacle Heuristic
        if (min_obstacle_dist < proximity_threshold)
        {
            //std::cout << "shortest_dist: " << shortest_dist << std::endl;
            regulated_linear_vel = regulated_linear_vel * (min_obstacle_dist / proximity_threshold);
        }

        // Vary Lookahead
        adjusted_lookahead = regulated_linear_vel * lookahead_gain;

        regulated_linear_vel = std::clamp(regulated_linear_vel, -max_linear_vel_, max_linear_vel_);
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

        return writeCmdVel(regulated_linear_vel, angular_vel);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)