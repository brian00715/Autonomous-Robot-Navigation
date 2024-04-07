#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

  RRTPlanner::RRTPlanner(ros::NodeHandle *node)
      : nh_(node),
        private_nh_("~"),
        map_received_(false),
        init_pose_received_(false),
        goal_received_(false)
  {
    // Get map and path topics from parameter server
    std::string map_topic, path_topic;
    private_nh_.param<std::string>("map_topic", map_topic, "/map");
    private_nh_.param<std::string>("path_topic", path_topic, "/path");
    private_nh_.param<int>("max_iter", max_iter_, 10000);
    private_nh_.param<int>("forward_max_step_size", forward_max_step_size_, 20);
    private_nh_.param<int>("goal_reach_th", goal_reach_th_, 10);
    private_nh_.param<int>("obs_path_discre_coef", obs_path_discre_coef_, 5);
    private_nh_.param<int>("radius", radius_, 10);
    private_nh_.param<std::string>("mode", mode_, "std");
    ROS_INFO("RRT Mode: %s", mode_.c_str());
    ROS_INFO("RRT Params:\r\n\tmax_iter: %d, forward_max_step_size: %d, \
    goal_reach_th: %d, obs_path_discre_coef: %d radius: %d",
             max_iter_, forward_max_step_size_, goal_reach_th_, obs_path_discre_coef_, radius_);

    // Subscribe to map topic
    map_sub_ = nh_->subscribe<const nav_msgs::OccupancyGrid::Ptr &>(
        map_topic, 1, &RRTPlanner::mapCallback, this);

    // Subscribe to initial pose topic that is published by RViz
    init_pose_sub_ = nh_->subscribe<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>(
        "/initialpose", 1, &RRTPlanner::initPoseCallback, this);

    // Subscribe to goal topic that is published by RViz
    goal_sub_ = nh_->subscribe<const geometry_msgs::PoseStamped::ConstPtr &>(
        "/move_base_simple/goal", 1, &RRTPlanner::goalCallback, this);

    // Advertise topic where calculated path is going to be published
    path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1, true);

    // This loops until the node is running, will exit when the node is killed
    while (ros::ok())
    {
      // if map, initial pose, and goal have been received
      // build the map image, draw initial pose and goal, and plan
      if (map_received_ && init_pose_received_ && goal_received_)
      {
        buildMapImage();
        drawGoalInitPose();
        if (mode_ == "std")
        {
          plan();
        }
        else if (mode_ == "star")
        {
          planStar();
        }
        else
        {
          ROS_ERROR("Invalid mode: %s", mode_.c_str());
        }

        // test isAnyObstacleInPath
        if (0)
        {
          init_pose_received_ = false;
          goal_received_ = false;
          bool res = isAnyObstacleInPath(init_pose_, goal_);
          std::cout << "obs on the path: " << (res ? "true" : "false") << std::endl;
          drawLine(init_pose_, goal_, cv::Scalar(0, 0, 255), 2);
        }
      }
      else
      {
        if (map_received_)
        {
          displayMapImage();
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
      }
    }
  }

  void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr &msg)
  {
    map_grid_ = msg;

    // Build and display the map image
    buildMapImage();
    displayMapImage();

    // Reset these values for a new planning iteration
    map_received_ = true;
    init_pose_received_ = false;
    goal_received_ = false;

    ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
  }

  void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    if (init_pose_received_)
    {
      buildMapImage();
    }

    // Convert mas to Point2D
    poseToPoint(init_pose_, msg->pose.pose);
    // std::cout << "x: " << msg->pose.pose.position.x << " y: " << msg->pose.pose.position.y << std::endl;
    // int index = toIndex(init_pose_.x(), init_pose_.y());
    // std::cout << "index: " << index << std::endl;
    // std::cout << "point_x: " << init_pose_.x() << " point_y: " << init_pose_.y() << std::endl;
    // printf("data: %d\n", map_grid_->data[index]);

    // Reject the initial pose if the given point is occupied in the map
    if (!isPointUnoccupied(init_pose_))
    {
      init_pose_received_ = false;
      ROS_WARN(
          "The initial pose specified is on or too close to an obstacle please specify another point");
    }
    else
    {
      init_pose_received_ = true;
      drawGoalInitPose();
      ROS_INFO("Initial pose obtained successfully.");
    }

    displayMapImage();
  }

  void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (goal_received_)
    {
      buildMapImage();
    }

    // Convert msg to Point2D
    poseToPoint(goal_, msg->pose);

    // Reject the goal pose if the given point is occupied in the map
    if (!isPointUnoccupied(goal_))
    {
      goal_received_ = false;
      ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
    }
    else
    {
      goal_received_ = true;
      drawGoalInitPose();
      ROS_INFO("Goal obtained successfully.");
    }

    displayMapImage();
  }

  void RRTPlanner::drawGoalInitPose()
  {
    if (goal_received_)
    {
      drawCircle(goal_, 3, cv::Scalar(12, 255, 43));
    }
    if (init_pose_received_)
    {
      drawCircle(init_pose_, 3, cv::Scalar(255, 200, 0));
    }
  }

  void RRTPlanner::publishPath()
  {
    // Create new Path msg
    nav_msgs::Path path;
    path.header.frame_id = map_grid_->header.frame_id;
    path.header.stamp = ros::Time::now();

    // TODO: Fill nav_msgs::Path msg with the path calculated by RRT
    std::vector<geometry_msgs::PoseStamped> tempPathPoses;
    const RRTNode *currNode = &tree_->back();
    while (currNode != nullptr)
    {
      geometry_msgs::PoseStamped pose;
      pose = pointToPose(currNode->point);
      // std::cout << "x: " << pose.pose.position.x << " y: " << pose.pose.position.y << std::endl;
      tempPathPoses.push_back(pose);

      currNode = currNode->parent;
      // std::cout << "currNode: " << currNode << std::endl;
    }
    std::reverse(tempPathPoses.begin(), tempPathPoses.end());
    path.poses = tempPathPoses;

    // Publish the calculated path
    path_pub_.publish(path);

    // Optional: visualize the path
    if (1)
    {
      for (size_t i = 1; i < path.poses.size(); ++i)
      {
        Point2D p1, p2;
        poseToPoint(p1, path.poses[i - 1].pose);
        poseToPoint(p2, path.poses[i].pose);
        drawLine(p1, p2, cv::Scalar(0, 255, 0), 2);
      }
    }
    displayMapImage();
  }

  bool RRTPlanner::isPointUnoccupied(const Point2D &p)
  {
    // TODO: Fill out this function to check if a given point is occupied/free in the map
    if (p.x() < 0 || p.x() >= map_grid_->info.height || p.y() < 0 || p.y() >= map_grid_->info.width)
    {
      return false;
    }

    if (map_grid_->data[toIndex(p.x(), p.y())] < OCCU_TH)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void RRTPlanner::buildMapImage()
  {
    // Create a new opencv matrix with the same height and width as the received map
    map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                                map_grid_->info.width,
                                                CV_8UC3,
                                                cv::Scalar::all(255)));

    // Fill the opencv matrix pixels with the map points
    for (int i = 0; i < map_grid_->info.height; i++)
    {
      for (int j = 0; j < map_grid_->info.width; j++)
      {
        if (map_grid_->data[toIndex(i, j)])
        {
          map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
        }
        else
        {
          map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
        }
      }
    }
  }

  void RRTPlanner::displayMapImage(int delay)
  {
    cv::imshow("Output", *map_);
    cv::waitKey(delay);
  }

  void RRTPlanner::drawCircle(Point2D &p, int radius, const cv::Scalar &color)
  {
    cv::circle(
        *map_,
        cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
        radius,
        color,
        -1);
  }

  void RRTPlanner::drawLine(const Point2D &p1, const Point2D &p2, const cv::Scalar &color, int thickness)
  {
    cv::line(
        *map_,
        cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
        cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
        color,
        thickness);
  }

  inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D &p)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = p.y() * map_grid_->info.resolution;
    pose.pose.position.y = p.x() * map_grid_->info.resolution;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = map_grid_->header.frame_id;
    return pose;
  }

  inline void RRTPlanner::poseToPoint(Point2D &p, const geometry_msgs::Pose &pose)
  {
    p.x(pose.position.y / map_grid_->info.resolution);
    p.y(pose.position.x / map_grid_->info.resolution);
  }

  inline int RRTPlanner::toIndex(int x, int y)
  {
    return x * map_grid_->info.width + y;
  }

  Point2D RRTPlanner::generateRandomPoint()
  {
    int width = map_grid_->info.width;
    int height = map_grid_->info.height;
    int x = rand() % width;
    int y = rand() % height;
    return Point2D(x, y);
  }

  RRTNode *RRTPlanner::findNearestNode(std::list<RRTNode> &tree, const Point2D &point)
  {
    RRTNode *nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    for (auto &node : tree)
    {
      double dist = sqrt(pow(node.point.x() - point.x(), 2) + pow(node.point.y() - point.y(), 2));
      if (dist < min_dist)
      {
        nearest = &node;
        min_dist = dist;
      }
    }
    return nearest;
  }

  Point2D RRTPlanner::steerTowards(const RRTNode &nearest, const Point2D &random_pt)
  {
    Point2D direction(random_pt.x() - nearest.point.x(), random_pt.y() - nearest.point.y());
    double length = sqrt(direction.x() * direction.x() + direction.y() * direction.y());
    if (length <= forward_max_step_size_)
      return random_pt;

    // Scale the direction to the max step size
    Point2D new_pt(nearest.point.x() + (direction.x() / length) * forward_max_step_size_,
                   nearest.point.y() + (direction.y() / length) * forward_max_step_size_);
    return new_pt;
  }

  bool RRTPlanner::isGoalReached(const Point2D &point, const Point2D &goal)
  {
    double distance = sqrt(pow(point.x() - goal.x(), 2) + pow(point.y() - goal.y(), 2));
    return distance <= goal_reach_th_;
  }

  bool RRTPlanner::isAnyObstacleInPath(const Point2D &p1, const Point2D &p2)
  {
    double theta = atan2(p2.y() - p1.y(), p2.x() - p1.x());
    double distance = sqrt(pow(p2.x() - p1.x(), 2) + pow(p2.y() - p1.y(), 2));
    // ROS_INFO("distance: %f", distance);
    int num_step = distance * obs_path_discre_coef_;
    // ROS_INFO("num_step: %d", num_step);
    for (int i = 0; i < num_step; ++i)
    {
      float line_x = (float)p1.x() + (float)i * cos(theta);
      float line_y = (float)p1.y() + (float)i * sin(theta);
      if (!isPointUnoccupied(Point2D(line_x, line_y)))
      {
        return true;
      }
    }
    return false;
  }

  void RRTPlanner::chooseParent(RRTNode &new_node)
  {
    for (auto &node : *tree_)
    {
      if (isAnyObstacleInPath(node.point, new_node.point))
      {
        continue;
      }

      double dist = sqrt(pow(node.point.x() - new_node.point.x(), 2) + pow(node.point.y() - new_node.point.y(), 2));
      if (dist < radius_) // candidate parent
      {
        float cost = node.cost + dist;
        if (new_node.cost > cost) // optimize the new node's cost
        {
          new_node.cost = cost;
          new_node.parent = &node;
        }
        else // optimize other candidate parent's cost, i.e, rewire
        {
          if (node.cost > new_node.cost + dist)
          {
            node.cost = new_node.cost + dist;
            node.parent = &new_node;
          }
        }
      }
    }
  }

  void RRTPlanner::plan()
  {
    // Reset these values so planning only happens once for a
    // given pair of initial pose and goal points
    goal_received_ = false;
    init_pose_received_ = false;

    // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
    //       path through the map starting from the initial pose and ending at the goal pose

    ROS_INFO("Start to plan path using RRT...");

    tree_ = new std::list<RRTNode>;
    RRTNode init_node = RRTNode(init_pose_, nullptr);
    tree_->push_back(init_node);
    bool path_found = false;
    srand(time(nullptr)); // Seed for random number generation

    auto start_time = std::chrono::high_resolution_clock::now();

    int i = 0;
    while (i < max_iter_ && !path_found)
    {
      Point2D random_pt = generateRandomPoint();
      RRTNode *nearest_node = findNearestNode(*tree_, random_pt);
      if (isAnyObstacleInPath((*nearest_node).point, random_pt))
      {
        continue;
      }

      Point2D new_point = steerTowards(*nearest_node, random_pt);
      // Optional: draw the expansion for visualization
      if (1)
      {
        drawLine(nearest_node->point, new_point, cv::Scalar(255, 0, 0), 1);
        displayMapImage(1);
      }
      RRTNode new_node = RRTNode(new_point, nearest_node);
      chooseParent(new_node);
      tree_->push_back(new_node);

      if (isGoalReached(new_point, goal_))
      {
        path_found = true;
        tree_->push_back(RRTNode(goal_, &(*tree_).back()));
      }

      i++;
    }

    if (!path_found)
    {
      ROS_WARN("Failed to find a path to the goal");
      return;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    ROS_INFO("Path found! Time taken to find path: %f ms after %d iters.", elapsed.count() * 1000.0f, i);

    publishPath();
  }

  void RRTPlanner::planStar()
  {
    goal_received_ = false;
    init_pose_received_ = false;

    ROS_INFO("Start to plan path using RRT*...");

    tree_ = new std::list<RRTNode>;
    RRTNode init_node = RRTNode(init_pose_, nullptr);
    tree_->push_back(init_node);
    bool path_found = false;
    srand(time(nullptr)); // Seed for random number generation

    auto start_time = std::chrono::high_resolution_clock::now();

    int i = 0;
    while (i < max_iter_ && !path_found)
    {
      Point2D random_pt = generateRandomPoint();
      RRTNode *nearest_node = findNearestNode(*tree_, random_pt);
      if (isAnyObstacleInPath((*nearest_node).point, random_pt))
      {
        continue;
      }

      Point2D new_point = steerTowards(*nearest_node, random_pt);
      // Optional: draw the expansion for visualization
      if (1)
      {
        drawLine(nearest_node->point, new_point, cv::Scalar(255, 0, 0), 1);
        displayMapImage(1);
      }
      RRTNode new_node = RRTNode(new_point, nearest_node);
      tree_->push_back(new_node);

      if (isGoalReached(new_point, goal_))
      {
        path_found = true;
        tree_->push_back(RRTNode(goal_, &(*tree_).back()));
      }

      i++;
    }

    if (!path_found)
    {
      ROS_WARN("Failed to find a path to the goal");
      return;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    ROS_INFO("Path found! Time taken to find path: %f ms after %d iters.", elapsed.count() * 1000.0f, i);

    publishPath();
  }
} // namespace rrt_planner
