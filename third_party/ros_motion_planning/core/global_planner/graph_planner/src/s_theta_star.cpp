/**
* *********************************************************
*
* @file: s_theta_star.cpp
* @brief: Contains the S-Theta* planner class
* @author: Wu Maojia
* @date: 2024-3-9
* @version: 1.0
*
* Copyright (c) 2024, Wu Maojia, Yang Haodong.
* All rights reserved.
*
* --------------------------------------------------------
*
* ********************************************************
*/
#include "s_theta_star.h"

namespace global_planner
{
/**
* @brief Construct a new SThetaStar object
* @param nx          pixel number in costmap x direction
* @param ny          pixel number in costmap y direction
* @param resolution  costmap resolution
*/
SThetaStar::SThetaStar(int nx, int ny, double resolution) : ThetaStar(nx, ny, resolution)
{
 factor_ = 0.35;
};

/**
* @brief S-Theta* implementation
* @param global_costmap global costmap
* @param start         start node
* @param goal          goal node
* @param path          optimal path consists of Node
* @param expand        containing the node been search during the process
* @return  true if path found, else false
*/
bool SThetaStar::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                     std::vector<Node>& expand)
{
 // initialize
 costs_ = global_costmap;
 path.clear();
 expand.clear();

 // closed list

 // open list and closed list
 std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
 std::unordered_map<int, Node> closed_list;

 open_list.push(start);

 // get all possible motions
 const std::vector<Node> motion = Node::getMotion();

 // main process
 while (!open_list.empty())
 {
   // pop current node from open list
   Node current = open_list.top();
   open_list.pop();

   // current node does not exist in closed list
   if (closed_list.find(current.id_) != closed_list.end())
     continue;

   closed_list.insert(std::make_pair(current.id_, current));
   expand.push_back(current);

   // goal found
   if (current == goal)
   {
     path = _convertClosedListToPath(closed_list, start, goal);
     return true;
   }

   // explore neighbor of current node
   for (const auto& m : motion)
   {
     // explore a new node
     // path 1

     Node node_new = current + m;  // add the x_, y_, g_
     node_new.h_ = _getDistance(node_new, goal);
     node_new.id_ = grid2Index(node_new.x_, node_new.y_);
     node_new.pid_ = current.id_;

     // current node do not exist in closed list
     if (closed_list.find(node_new.id_) != closed_list.end())
       continue;

     double alpha = 0.0;

     // get the coordinate of parent node
     Node parent;
     parent.id_ = current.pid_;
     index2Grid(parent.id_, parent.x_, parent.y_);

     // update g value
     auto find_parent = closed_list.find(parent.id_);
     if (find_parent != closed_list.end())
     {
       parent = find_parent->second;
       alpha = _alpha(parent, node_new, goal);
       // update g
       node_new.g_ = current.g_ + _getDistance(parent, node_new) + alpha;
     }

     // next node hit the boundary or obstacle
     if ((node_new.id_ < 0) || (node_new.id_ >= ns_)
         || (costs_[node_new.id_] >= lethal_cost_ * factor_ && costs_[node_new.id_] >= costs_[current.id_]))
       continue;

     if (find_parent != closed_list.end())
     {
       _updateVertex(parent, node_new, alpha);
     }

     // add node to open list
     open_list.push(node_new);
   }
 }

 return false;
}

/**
* @brief update the g value of child node
* @param parent
* @param child
* @param alpha
*/
void SThetaStar::_updateVertex(const Node& parent, Node& child, const double alpha){
// if (alpha == 0 || _lineOfSight(parent, child)){  // "alpha == 0" will cause penetration of obstacles
  if (_lineOfSight(parent, child)){
   // path 2
   double new_g = parent.g_ + _getDistance(parent, child) + alpha;
   if (new_g < child.g_){
     child.g_ = new_g;
     child.pid_ = parent.id_;
   }
 }
}

/**
* @brief Get the deviation cost
* @param parent
* @param child
* @param goal
* @return deviation cost
*/
double SThetaStar::_alpha(const Node& parent, const Node& child, const Node& goal)
{
 double d_qt = _getDistance(parent, child);
 double d_qg = _getDistance(parent, goal);
 double d_tg = _getDistance(child, goal);
 double cost;
 double value = (d_qt * d_qt + d_qg * d_qg - d_tg * d_tg) / (2 * d_qt * d_qg);
 if (value <= -1.0) cost = pi_;
 else if (value >= 1.0) cost = 0.0;
 else cost = std::acos(value);
 return cost;
}

/**
* @brief Get the Euclidean distance between two nodes
* @param node  current node
* @param goal  goal node
* @return  Euclidean distance
*/
double SThetaStar::_getDistance(const Node& node, const Node& goal)
{
 return std::hypot(node.x_ - goal.x_, node.y_ - goal.y_);
}
}  // namespace global_planner
