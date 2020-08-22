#include <random>
#include <vector>
#include <algorithm>
#include <cmath>
#include "rrt_anytime.h"

// Define constants parameters
constexpr double half_cell_sz = 0.5;
constexpr double tol= 0.000001;

// Finds the closest node in the tree from the new_node
Node MyRRT::GetClosestNode(Node& new_node) {
  Node nearest_node(-1, -1, -1, -1, -1, -1);
  std::vector<Node>::const_iterator iter_f;
  std::vector<Node>::const_iterator iter_f_store;
  // NOTE: Use total cost not just distance
  auto dist = static_cast<double>(n * n);
  for (iter_f = point_list_.begin(); iter_f != point_list_.end(); ++iter_f) {
    auto new_dist = static_cast<double>(
        std::sqrt((static_cast<double>(iter_f->x_ - new_node.x_) *
                   static_cast<double>(iter_f->x_ - new_node.x_)) +
                  (static_cast<double>(iter_f->y_ - new_node.y_) *
                   static_cast<double>(iter_f->y_ - new_node.y_))));
    if (new_dist > threshold_) {
      continue;
    }
    new_dist += iter_f->cost_;

    if (CheckCollision(*iter_f, new_node)) {
      continue;
    }
    if (iter_f->id_ == new_node.id_) {
      continue;
    }
    // The nearest nodes are stored while searching for the nearest node to
    // speed up th Rewiring process
    near_nodes_.push_back(*iter_f);
    near_nodes_dist_.push_back(new_dist);
    if (iter_f->pid_ == new_node.id_) {
      continue;
    }
    if (new_dist >= dist) {
      continue;
    }
    dist = new_dist;
    iter_f_store = iter_f;
  }
  if (dist != n * n) {
    nearest_node = *iter_f_store;
    new_node.pid_ = nearest_node.id_;
    new_node.cost_ = dist;
  }
  return nearest_node;
}

// Check for collision: Checks if the line joining n_1 and n_2 passes through the obstacle cell
bool MyRRT::CheckCollision(const Node& n_1, const Node& n_2) const {
  if (n_2.y_ - n_1.y_ == 0) {
    double c = n_2.y_;
    for (const auto& obs_node : obstacle_list_) {
      if (!(((n_1.x_ >= obs_node.x_) && (obs_node.x_ >= n_2.x_)) ||
            ((n_1.x_ <= obs_node.x_) && (obs_node.x_ <= n_2.x_)))) {
        continue;
      }
      if (static_cast<double>(obs_node.y_) == c) {
        return true;
      }
    }
  } else {
    double gradient = static_cast<double>(n_2.x_ - n_1.x_) /
                   static_cast<double>(n_2.y_ - n_1.y_);
    double c =
        static_cast<double>(n_2.x_) - gradient * static_cast<double>(n_2.y_);
    for (const auto& obs_node : obstacle_list_) {
      if (!(((n_1.y_ >= obs_node.y_) && (obs_node.y_ >= n_2.y_)) ||
            ((n_1.y_ <= obs_node.y_) && (obs_node.y_ <= n_2.y_)))) {
        continue;
      }
      if (!(((n_1.x_ >= obs_node.x_) && (obs_node.x_ >= n_2.x_)) ||
            ((n_1.x_ <= obs_node.x_) && (obs_node.x_ <= n_2.x_)))) {
        continue;
      }
      std::vector<double> dist_vrtx(4);      
      dist_vrtx[0] = static_cast<double>(obs_node.x_) + half_cell_sz - gradient * (static_cast<double>(obs_node.y_) + half_cell_sz) - c;
      dist_vrtx[1] = static_cast<double>(obs_node.x_) + half_cell_sz - gradient * (static_cast<double>(obs_node.y_) - half_cell_sz) - c;
      dist_vrtx[2] = static_cast<double>(obs_node.x_) - half_cell_sz - gradient * (static_cast<double>(obs_node.y_) + half_cell_sz) - c;
      dist_vrtx[3] = static_cast<double>(obs_node.x_) - half_cell_sz - gradient * (static_cast<double>(obs_node.y_) - half_cell_sz) - c;
      double count = 0;
      for (auto& val : dist_vrtx) {
        if (std::fabs(val) <= tol) {
          val = 0;
        } else {
          count += val / std::fabs(val);
        }
      }
      if (std::abs(count) < 3) {
        return true;
      }
    }
  }
  return false;
}

// Generates radom node (configuration) 
Node MyRRT::GenRandNode() const {
  std::random_device rd;
  std::uniform_int_distribution<int> distr(0, n - 1);
  int x = distr(rd);
  int y = distr(rd);
  Node new_node(x, y, 0, 0, n * x + y, 0);
  return new_node;
}

// Optimize the node costs
void MyRRT::Rewiring(const Node& new_node) {
  std::vector<Node>::iterator iter_f;
  for (size_t i = 0; i < near_nodes_.size(); i++) {
    if (near_nodes_[i].cost_ > near_nodes_dist_[i] + new_node.cost_) {
      iter_f = std::find_if(point_list_.begin(), point_list_.end(),
                          [&](const Node& node) {
                            return CompareNodeCoordinates(node, near_nodes_[i]);
                          });
      if (iter_f != point_list_.end()) {
        iter_f->pid_ = new_node.id_;
        iter_f->cost_ = near_nodes_dist_[i] + new_node.cost_;
      }
    }
  }
  near_nodes_.clear();
  near_nodes_dist_.clear();
}

// Check if goal is reached
bool MyRRT::ReachedGoal(const Node& new_node) {
  if (!CheckCollision(new_node, goal_)) {
    auto new_dist = static_cast<double>(
        std::sqrt(static_cast<double>((goal_.x_ - new_node.x_) *
                                      (goal_.x_ - new_node.x_)) +
                  static_cast<double>((goal_.y_ - new_node.y_) *
                                      (goal_.y_ - new_node.y_))));
    if (new_dist > threshold_) {
      return false;
    }
    new_dist += new_node.cost_;
    goal_.pid_ = new_node.id_;
    goal_.cost_ = new_dist;
    std::vector<Node>::iterator iter_f;
    iter_f = std::find_if(
        point_list_.begin(), point_list_.end(),
        [&](const Node& node) { return CompareNodeCoordinates(node, new_node); });
    if (iter_f != point_list_.end() && goal_.cost_ < iter_f->cost_) {
      point_list_.erase(iter_f);
      point_list_.push_back(goal_);
    } else if (iter_f == point_list_.end()) {
      point_list_.push_back(goal_);
    }
    return true;
  }
  return false;
}

// Lists obstacles as occupied nodes
void MyRRT::StoreObstacles(std::vector<std::vector<int>>& grid_in) {
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (grid_in[i][j] == 1) {
        Node obstacle(i, j, 0, 0, j+ i * n, 0);
        obstacle_list_.push_back(obstacle);
      }
    }
  }
}

// Given an non-optimal path, finds an optimal path
std::vector<Node> MyRRT::improved_path(std::vector<std::vector<int>>& grid_in, 
                                  int max_iter_multiplier, double threshold_in) {
    threshold_ = threshold_in;
    int max_iter = max_iter_multiplier * n * n;
    int iter = 0;
    Node new_node = start_;
    
    while (true) {
    iter++;
    if (iter > max_iter) {
      return point_list_;
    }
    new_node = GenRandNode();
    if (grid_in[new_node.x_][new_node.y_] == 1) {
      continue;
    }
    
    Node nearest_node = GetClosestNode(new_node);
    if (nearest_node.id_ == -1) {
      continue;
    }
    
    grid_in[new_node.x_][new_node.y_] = 4;
    
    auto iter_f = std::find_if(
        point_list_.begin(), point_list_.end(),
        [&](const Node& node) { return CompareNodeCoordinates(node, new_node); });
    if (iter_f != point_list_.end() && new_node.cost_ < iter_f->cost_) {
      point_list_.erase(iter_f);
      point_list_.push_back(new_node);
    } else if (iter_f == point_list_.end()) {
      point_list_.push_back(new_node);
    }
     
    Rewiring(new_node);
  }
}

// RRT to compute non-optimal path
std::vector<Node> MyRRT::rrt_non_optimal(std::vector<std::vector<int>>& grid_in,
                                    const Node& start_in, const Node& goal_in,
                                    int max_iter_multiplier,
                                    double threshold_in) {
  start_ = start_in;
  goal_ = goal_in;
  n = grid_in.size();
  threshold_ = threshold_in;
  int max_iter = max_iter_multiplier * n * n;
  StoreObstacles(grid_in);
  point_list_.push_back(start_);
  grid_in[start_.x_][start_.y_] = 2;
  int iter = 0;
  Node new_node = start_;
  if (ReachedGoal(new_node)) {
    found_goal_ = true;
  }
  while (true) {
    iter++;
    if (iter > max_iter) {
      if (!found_goal_) {
        Node no_path_node(-1, -1, -1, -1, -1, -1);
        point_list_.clear();
        point_list_.push_back(no_path_node);
      }
      return point_list_;
    }
    new_node = GenRandNode();
    if (grid_in[new_node.x_][new_node.y_] == 1) {
      continue;
    }
   
    Node nearest_node = GetClosestNodeModified(new_node);
    if (nearest_node.id_ == -1) {
      continue;
    }
    
    grid_in[new_node.x_][new_node.y_] = 2;
    
    auto iter_f = std::find_if(
        point_list_.begin(), point_list_.end(),
        [&](const Node& node) { return CompareNodeCoordinates(node, new_node); });
    if (iter_f != point_list_.end() && new_node.cost_ < iter_f->cost_) {
      point_list_.erase(iter_f);
      point_list_.push_back(new_node);
    } else if (iter_f == point_list_.end()) {
      point_list_.push_back(new_node);
    }
    
    if (ReachedGoal(new_node)) {
      found_goal_ = true;
    }
  }
}

// Find closest node based on relaxed heuristic
Node MyRRT::GetClosestNodeModified(Node& new_node) {
  Node nearest_node(-1, -1, -1, -1, -1, -1);
  std::vector<Node>::const_iterator iter_f;
  std::vector<Node>::const_iterator iter_f_store;
  // NOTE: Use total cost not just distance
  auto dist = static_cast<double>(n * n);
  for (iter_f = point_list_.begin(); iter_f != point_list_.end(); ++iter_f) {
    auto new_dist = static_cast<double>(
        std::sqrt((static_cast<double>(iter_f->x_ - new_node.x_) *
                   static_cast<double>(iter_f->x_ - new_node.x_)) +
                  (static_cast<double>(iter_f->y_ - new_node.y_) *
                   static_cast<double>(iter_f->y_ - new_node.y_))));
    if (new_dist > threshold_) {
      continue;
    }
    new_dist += iter_f->cost_;

    if (CheckCollision(*iter_f, new_node)) {
      continue;
    }
    if (iter_f->id_ == new_node.id_) {
      continue;
    }
    
    near_nodes_.push_back(*iter_f);
    near_nodes_dist_.push_back(new_dist);
    
    if (iter_f->pid_ == new_node.id_) {
      continue;
    }
    
    dist = new_dist;
    iter_f_store = iter_f;
  }
  
  if (dist != n * n) {
    nearest_node = *iter_f_store;
    new_node.pid_ = nearest_node.id_;
    new_node.cost_ = dist;
  }
  
  return nearest_node;
}

