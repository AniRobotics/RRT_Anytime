/* 
 * File:   rrt_anytime.h
 * Author: simbot
 *
 * Created on August 5, 2020, 8:56 PM
 */

#ifndef RRT_ANYTIME_H
#define RRT_ANYTIME_H

#include "helper.h"

class MyRRT {
 public:
  Node GenRandNode() const;
  Node GetClosestNode(Node& new_node);
  Node GetClosestNodeModified(Node& new_node);
  bool CheckCollision(const Node& n_1, const Node& n_2) const;
  void Rewiring(const Node& new_node);  
  bool ReachedGoal(const Node& new_node);
  void StoreObstacles(std::vector<std::vector<int>>& grid_in);
  std::vector<Node> improved_path(std::vector<std::vector<int>>& grid_in, int max_iter_multiplier, double threshold_in);
  std::vector<Node> rrt_non_optimal(
      std::vector<std::vector<int>>& grid_in, const Node& start_in,
      const Node& goal_in, const int max_iter_multiplier,
      const double threshold_in);
 private:
  std::vector<Node> point_list_;
  std::vector<Node> obstacle_list_;
  std::vector<Node> near_nodes_;
  std::vector<double> near_nodes_dist_;
  Node start_, goal_;
  double threshold_;
  bool found_goal_ = false;
  int n = 0;
};

#endif /* RRT_ANYTIME_H */

