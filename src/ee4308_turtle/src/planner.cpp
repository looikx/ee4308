#include "ee4308_turtle/planner.hpp"

namespace ee4308::turtle
{

    // ======================== Nav2 Planner Plugin ===============================
    void Planner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        // initialize states / variables
        node_ = parent.lock(); // this class is not a node. It is instantiated as part of a node `parent`.
        tf_ = tf;
        plugin_name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros->getGlobalFrameID();

        // initialize parameters
        initParam(node_, plugin_name_ + ".max_access_cost", max_access_cost_, 255);
        initParam(node_, plugin_name_ + ".interpolation_distance", interpolation_distance_, 0.05);
        initParam(node_, plugin_name_ + ".sg_half_window", sg_half_window_, 5);
        initParam(node_, plugin_name_ + ".sg_order", sg_order_, 3);
    }

    nav_msgs::msg::Path Planner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        // initializations
        PlannerNodes nodes(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
        OpenList open_list;
        // RayTracer ray_tracer;

        int start_mx, start_my, goal_mx, goal_my;
        costmap_->worldToMapEnforceBounds(
            start.pose.position.x, start.pose.position.y,
            start_mx, start_my);
        costmap_->worldToMapEnforceBounds(
            goal.pose.position.x, goal.pose.position.y,
            goal_mx, goal_my);
        nodes.getNode(start_mx, start_my)->g = 0;
        nodes.getNode(start_mx, start_my)->h = std::hypot(goal_mx - start_mx, goal_my - start_my);
        open_list.queue(nodes.getNode(start_mx, start_my));

        // ray_tracer.init(goal_mx, goal_my, start_mx, start_my);
        std::vector<std::array<int, 2>> coords;
        while (!open_list.empty())
        {
            PlannerNode *curr_node = open_list.pop();
            if (curr_node->expanded)
            {
                continue;
            }
            else if (curr_node->mx == goal_mx && curr_node->my == goal_my)
            {
                /* code */
                while (curr_node != nullptr)
                {
                    std::array<int, 2> coord = {curr_node->mx, curr_node->my};
                    coords.push_back(coord);
                    curr_node = curr_node->parent;
                }
                std::reverse(coords.begin(), coords.end());
                break;
            }
            curr_node->expanded = true;
            for (int i = -1; i < 2; i++)
            {
                for (int j = -1; j < 2; j++)
                {
                    if (i == 0 && j == 0)
                    {
                        continue;
                    }
                    PlannerNode *neighbor_node = nodes.getNode(curr_node->mx + i, curr_node->my + j);
                    if (neighbor_node == nullptr)
                    {
                        continue;
                    }
                    double cost = costmap_->getCost(neighbor_node->mx, neighbor_node->my) + 1;
                    cost = (cost < max_access_cost_) ? cost : INFINITY;
                    double temp_g = curr_node->g + std::hypot(i, j) + cost;
                    if (temp_g < neighbor_node->g)
                    {
                        neighbor_node->g = temp_g;
                        neighbor_node->parent = curr_node;
                        neighbor_node->h = hypot(neighbor_node->mx - goal_mx, neighbor_node->my - goal_my);
                        neighbor_node->f = neighbor_node->h + temp_g;
                        open_list.queue(neighbor_node);
                    }
                }
            }
        }

        nav_msgs::msg::Path generated_path = writeToPath(coords, goal);
        int r = 2 * sg_half_window_ + 1;
        int c = sg_order_ + 1;
        Eigen::MatrixXd J(r, c);
        Eigen::MatrixXd a(r, c);
        for (int i = 0; i < r; i++)
        {
            for (int j = 0; j < c; j++)
            {
                J(i, j) = std::pow((-1 * sg_half_window_ + i), j);
            }
        }
        a = (J.transpose() * J).inverse() * J.transpose();
        Eigen::RowVectorXd alpha = a.row(0);
        int path_size = static_cast<int>(generated_path.poses.size());
        for (size_t i = 0 ; i < generated_path.poses.size(); i++)
        {   
            // RCLCPP_WARN_STREAM(node_->get_logger(), "helep");
            double resx = 0;
            double resy = 0;
            for (int j = 0; j < r; j++)
            {
                int k = i + 1 + j - sg_half_window_;
                if (k < 1)
                {
                    k = 1;
                }
                else if (k > path_size - 1)
                {
                    k = path_size - 1;
                }
                // RCLCPP_WARN_STREAM(node_->get_logger(), "flag1");
                // k = std::clamp(k,1,sg_half_window_);
                resx += generated_path.poses[k].pose.position.x * alpha(j- sg_half_window_ +sg_half_window_);
                resy += generated_path.poses[k].pose.position.y * alpha(j- sg_half_window_ +sg_half_window_);
                // RCLCPP_WARN_STREAM(node_->get_logger(), "flag2");

            }
            // RCLCPP_WARN_STREAM(node_->get_logger(), "pos x: " << generated_path.poses[i].pose.position.x);
            // RCLCPP_WARN_STREAM(node_->get_logger(), "smooth x: " << resx);
            generated_path.poses[i].pose.position.x = resx;
            generated_path.poses[i].pose.position.y = resy;
            RCLCPP_WARN_STREAM(node_->get_logger(), "flag3");

        }


        // draws a straight line from goal to start on the grid
        // mimics how a vector is typically filled when iterating from the goal node to start node.

        // while (rclcpp::ok())
        // {
        //     std::array<int, 2> coord = ray_tracer.frontCell();
        //     coords.push_back(coord);
        //     if (ray_tracer.reached())
        //         break; // check reached here so
        //     ray_tracer.next();
        // }

        // reverse the coordinates because the convention for filling nav_msgs::msg::Path is from start to goal.
        // std::reverse(coords.begin(), coords.end());

        return generated_path;
    }

    nav_msgs::msg::Path Planner::writeToPath(
        std::vector<std::array<int, 2>> coords,
        geometry_msgs::msg::PoseStamped goal)
    {
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = global_frame_id_;
        path.header.stamp = node_->now();

        for (const auto &coord : coords)
        {
            // convert map coordinates to world coordiantes
            double wx, wy;
            costmap_->mapToWorld(coord[0], coord[1], wx, wy);
            // implement smoothing here
            // push the pose into the messages.

            geometry_msgs::msg::PoseStamped pose; // do not fill the header with timestamp or frame information.
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1; // normalized quaternion
            path.poses.push_back(pose);
        }

        // push the goal
        goal.header.frame_id = "";          // remove frame id to prevent incorrect transformations.
        goal.header.stamp = rclcpp::Time(); // remove timestamp from header, otherwise there will be time extrapolation issues.
        path.poses.push_back(goal);

        // return path;
        return path;
    }

    void Planner::cleanup()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }

    void Planner::activate()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }

    void Planner::deactivate()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }

    // ====================== Planner Node ===================
    PlannerNode::PlannerNode(int mx, int my) : mx(mx), my(my) {}

    // ======================= Open List Implemetations ===========
    bool OpenListComparator::operator()(PlannerNode *l, PlannerNode *r) const { return l->f > r->f; }

    void OpenList::queue(PlannerNode *node) { pq.push(node); }

    PlannerNode *OpenList::pop()
    {
        if (pq.empty())
            return nullptr;
        PlannerNode *cheapest_node = pq.top();
        pq.pop();
        return cheapest_node;
    }

    bool OpenList::empty() const { return pq.empty(); }

    // ======================== Nodes ===============================
    PlannerNodes::PlannerNodes(int num_cells_x, int num_cells_y)
    {
        size_mx = num_cells_x;
        size_my = num_cells_y;

        nodes.reserve(num_cells_x * num_cells_y);
        for (int mx = 0; mx < size_mx; ++mx)
            for (int my = 0; my < size_my; ++my)
                nodes[mx * size_my + my] = PlannerNode(mx, my);
    }

    PlannerNode *PlannerNodes::getNode(int mx, int my)
    {
        if (mx < 0 || my < 0 || mx >= size_mx || my >= size_my)
            return nullptr;
        return &nodes[mx * size_my + my];
    }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Planner, nav2_core::GlobalPlanner)