#ifndef RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
#define RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_

#include <random>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>

namespace rrt_planner
{

  /**
   * A utility class to represent a 2D point
   */
  class Point2D
  {
  public:
    Point2D() : x_(0), y_(0) {}
    Point2D(int x, int y) : x_(x), y_(y) {}

    int x() const
    {
      return x_;
    }

    int y() const
    {
      return y_;
    }

    void x(int x)
    {
      x_ = x;
    }

    void y(int y)
    {
      y_ = y;
    }

  private:
    int x_;
    int y_;
  };

  class RRTNode
  {
  public:
    Point2D point;
    RRTNode *parent;
    float cost;

    RRTNode(Point2D p, RRTNode *parent = nullptr, float cost = 0) : point(p), parent(parent), cost(cost) {}
  };

  /**
   * Main class which implements the RRT algorithm
   */
  class RRTPlanner
  {
  public:
    explicit RRTPlanner(ros::NodeHandle *);

    ~RRTPlanner() = default;

    /**
     * Given a map, the initial pose, and the goal, this function will plan
     * a collision-free path through the map from the initial pose to the goal
     * using the RRT algorithm
     *
     * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
     */
    void plan();

    /**
     * @brief Plan using RRT*
     *
     */
    void planStar();

    /**
     * @brief Plan using Informed RRT*
     *
     */
    void planInformedStar();

    /**
     * Callback for map subscriber
     */
    void mapCallback(const nav_msgs::OccupancyGrid::Ptr &);

    /**
     * Callback for initial pose subscriber
     */
    void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);

    /**
     * Callback for goal subscriber
     */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);

    /**
     * Utility function to steer the tree towards a given point
     * @param nearest: the nearest node in the tree to the given point
     * @param randomPoint: the point to which the tree is to be steered
     * @return the new point to which the tree has been steered
     */
    Point2D steerTowards(const RRTNode &nearest, const Point2D &randomPoint);

    /**
     * Utility function to check if a given point is within the bounds of the map
     * @param point: the point to be checked
     * @return boolean true if point is within bounds, false if not
     */
    bool isGoalReached(const Point2D &point, const Point2D &goal);

    /**
     * Utility function to check if there is any obstacle in the path between two points
     * @param point: the point to be checked
     * @return boolean true if point is within bounds, false if not
     */
    bool isAnyObstacleInPath(const Point2D &p1, const Point2D &p2);

    /**
     * Publishes the path calculated by RRT as a nav_msgs::Path msg
     *
     * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
     */
    void publishPath();

    /**
     * Utility function to check if a given point is free/occupied in the map
     * @param p: point in the map
     * @return boolean true if point is unoccupied, false if occupied
     *
     * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
     */
    bool isPointUnoccupied(const Point2D &p);

    /**
     * Utility function to build a CV::Mat from a nav_msgs::OccupancyGrid for display
     */
    void buildMapImage();

    /**
     * Utility function to display the CV::Mat map image
     * @param delay
     */
    void displayMapImage(int delay = 1);

    /**
     * Utility function to draw initial pose and goal pose on the map image
     */
    void drawGoalInitPose();

    /**
     * Utility function to draw a circle on the map
     * @param p: center point of the circle
     * @param radius: radius of the circle
     * @param color: color of the circle
     */
    void drawCircle(Point2D &p, int radius, const cv::Scalar &color);

    /**
     * Utility function to draw a line on the map
     * @param p1: starting point of the line
     * @param p2: end point of the line
     * @param color: color of the line
     * @param thickness: thickness of the line
     */
    void drawLine(const Point2D &p1, const Point2D &p2, const cv::Scalar &color, int thickness = 1);

    /**
     * Utility function to convert a Point2D object to a geometry_msgs::PoseStamped object
     * @return corresponding geometry_msgs::PoseStamped object
     */
    inline geometry_msgs::PoseStamped pointToPose(const Point2D &);

    /**
     * Utility function to convert a geometry_msgs::PoseStamped object to a Point2D object
     */
    inline void poseToPoint(Point2D &, const geometry_msgs::Pose &);

    /**
     * Utility function to convert (x, y) matrix coordinate to corresponding vector coordinate
     */
    inline int toIndex(int, int);

    /**
     * Utility function to generate a random point in the map
     * @return random point
     */
    Point2D generateRandomPoint();

  private:
    /**
     * Utility function to find the nearest node in the tree to a given point
     * @param tree: the tree of RRTNodes
     * @param point: the point to which the nearest node is to be found
     * @return the nearest node in the tree to the given point
     */
    RRTNode *findNearestNode(std::list<RRTNode> &tree, const Point2D &point);

    /**
     * Utility function to find the nearest node in the tree to a given point, using the RRT* algorithm
     * @param tree: the tree of RRTNodes
     * @param point: the point to which the nearest node is to be found
     * @return the nearest node in the tree to the given point
     */
    RRTNode *findNearestNodeStar(std::list<RRTNode> &tree, const Point2D &point);

    /**
     * @brief choose the parent for the new node and rewire the tree
     *
     * @param new_node
     */
    void chooseParent(RRTNode &new_node);

    ros::NodeHandle *nh_;
    ros::NodeHandle private_nh_;

    bool map_received_;
    std::unique_ptr<cv::Mat> map_;
    nav_msgs::OccupancyGrid::Ptr map_grid_;

    bool init_pose_received_;
    Point2D init_pose_;

    bool goal_received_;
    Point2D goal_;

    ros::Subscriber map_sub_;
    ros::Subscriber init_pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;

    const uint8_t OCCU_TH = 50;
    std::list<RRTNode> *tree_;
    int max_iter_;
    int forward_max_step_size_;
    int goal_reach_th_;
    int obs_path_discre_coef_;
    int radius_;
    std::string mode_;
  };

}

#endif // RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
