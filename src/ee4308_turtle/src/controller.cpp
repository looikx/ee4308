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
        initParam(node_, plugin_name_ + ".desired_linear_vel", desired_linear_vel_, 0.12);
        initParam(node_, plugin_name_ + ".desired_lookahead_dist", desired_lookahead_dist_, 0.3);
        initParam(node_, plugin_name_ + ".max_angular_vel", max_angular_vel_, 1.0);
        initParam(node_, plugin_name_ + ".max_linear_vel", max_linear_vel_, 0.22);
        initParam(node_, plugin_name_ + ".xy_goal_thres", xy_goal_thres_, 0.05);
        initParam(node_, plugin_name_ + ".yaw_goal_thres", yaw_goal_thres_, 0.25);
        initParam(node_, plugin_name_ + ".curvature_threshold", curvature_threshold, 0.1);
        initParam(node_, plugin_name_ + ".proximity_threshold", proximity_threshold, 0.3);
        initParam(node_, plugin_name_ + ".lookahead_gain", lookahead_gain, 0.5);

        // initialize topics
        // sub_scan_ = node_->create_subscription<some msg type>(
        //     "some topic", rclcpp::SensorDataQoS(),
        //     std::bind(&Controller::some_callback, this, std::placeholders::_1));
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
        //std::cout << "goal coordinates: " << goal_x << " " << goal_y << std::endl;


        double robot_x = pose.pose.position.x; // position x of the robot
        double robot_y = pose.pose.position.y; // position y of the robot
        double phi = ee4308::getYawFromQuaternion(pose.pose.orientation); // orientation of the robot

        // check if robot is close to the goal
        double dist_to_goal = std::hypot(goal_x - robot_x, goal_y - robot_y);
        if (dist_to_goal < xy_goal_thres_ && phi )
        {
            if (phi < yaw_goal_thres_)
            {
                return writeCmdVel(0, w0) // 
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

        // coordinates of the closest point
        //double closest_point_x = global_plan_.poses[closest_index].pose.position.x;
        //double closest_point_y = global_plan_.poses[closest_index].pose.position.y;

        // get lookahead?
        geometry_msgs::msg::PoseStamped lookahead_pose = goal_pose;

        // Determine lookahead point
        double lookahead_dist = (lookahead_dist > 0) ? 0 : desired_lookahead_dist_;
        for (size_t j = closest_index + 1; j < global_plan_.poses.size(); j++)
        {
            double point_pose_x = global_plan_.poses[j].pose.position.x; 
            double point_pose_y = global_plan_.poses[j].pose.position.y;
            double dist_from_robot = std::hypot((point_pose_x - robot_x),(point_pose_y - robot_y));

            if (dist_from_robot >= lookahead_dist) 
            {
                lookahead_pose = global_plan_.poses[j];
                break;
            }
        }


        double delta_x = lookahead_pose.pose.position.x - robot_x;
        double delta_y = lookahead_pose.pose.position.y - robot_y;

        //std::cout<<"phi: "<<phi<< std::endl;

        double x_r = delta_x * std::cos(phi) + delta_y * std::sin(phi); 
        double y_r = delta_y * std::cos(phi) - delta_x * std::sin(phi);

        // calculate curvature

        double d_hypot = std::hypot(x_r, y_r);
        double curvature = 2 * y_r / std::pow(d_hypot,2);
    
        //double linear_vel = 0 * (lookahead_pose.pose.position.x - pose.pose.position.x);
        //double angular_vel = 0 * getYawFromQuaternion(goal_pose.pose.orientation);
        
        double linear_vel =  desired_linear_vel_;
        double angular_vel = linear_vel * curvature;

        // Curvature Heuristic

        double intermediate_linear_vel = desired_linear_vel_;

        if (curvature_threshold < curvature)
        {
            intermediate_linear_vel = desired_linear_vel_ * (curvature_threshold / curvature);
        }

        // Proximity Heuristic

        double regulated_linear_vel = intermediate_linear_vel;
        if (shortest_dist < proximity_threshold)
        {
            regulated_linear_vel = intermediate_linear_vel * (shortest_dist / proximity_threshold);
        }

        // Vary Lookahead

        double adjusted_lookahead_dist = regulated_linear_vel * lookahead_gain;
        lookahead_dist = adjusted_lookahead_dist;       


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