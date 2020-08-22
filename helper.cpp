/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <iostream>
#include <random>
#include "helper.h"

Node::Node(const int node_x, const int node_y, const double node_cost, const double node_h_cost,
           const int node_id, const int node_pid) {
  x_ = node_x;
  y_ = node_y;
  cost_ = node_cost;
  h_cost_ = node_h_cost;
  id_ = node_id;
  pid_ = node_pid;
}

bool CompareNodeCoordinates(const Node& n1, const Node& n2) {
  return n1.x_ == n2.x_ && n1.y_ == n2.y_;
}

void BuildGridEnv(std::vector<std::vector<int>>& grid_in) {
  int n = grid_in.size();
  std::random_device rd;   
  std::uniform_int_distribution<int> distr(0, n);  
  for (int i = 0; i < n; i++) {
    for (int ii = 0; ii < n; ii++) {
      grid_in[i][ii] = distr(rd) / ((n - 1));
    }
  }
}

void ShowGridState(const std::vector<std::vector<int>>& grid_in) {
  int n = grid_in.size();
  std::cout << "Grid color code: " << std::endl;
  std::cout << "O[RED]: Obstacles" << std::endl;
  std::cout << "O[WHITE]: Free cells" << std::endl;
  std::cout << "*[CYAN]: Explored nodes in vanilla RRT" << std::endl;
  std::cout << "+[BLUE]: Explored nodes in anytime RRT" << std::endl;
  std::cout << "Nodes on path: green numbers" << std::endl;

  std::cout << "--------------------------------------" << std::endl;

  int path_node_num = 0;
  for (const auto& row : grid_in) {
    for (const auto& ele : row) {
      if (ele == 100) {
        std::cout << G << path_node_num++ << NEUTRAL << "   ";
      } else if (ele == 1) {
        std::cout << R << "O" << NEUTRAL << "   ";
      } else if (ele == 2) {
        std::cout << BLU << "*" << NEUTRAL << "   ";
      } else if (ele == 4) {
        std::cout << C << "+" << NEUTRAL << "   ";
      } else {
        std::cout << ele << "   ";
      }
    }
    std::cout << std::endl << std::endl;
  }
  std::cout << "--------------------------------------" << std::endl;
}

void PlotPathNodes(std::vector<Node>& path_vector, const Node& start,
               const Node& goal, std::vector<std::vector<int>>& grid_env) {
  if (path_vector[0].id_ == -1) {
    std::cout << "No path exists" << std::endl;
    ShowGridState(grid_env);
    return;
  }
  std::cout << "Path (goal to start):" << std::endl;
  for (size_t i = 0; i < path_vector.size(); i++) {
    if (CompareNodeCoordinates(goal, path_vector[i])) {
      // path_vector[i].NodeInfo();
      grid_env[path_vector[i].x_][path_vector[i].y_] = 100;
      while (path_vector[i].id_ != start.id_) {
        if (path_vector[i].id_ == path_vector[i].pid_) {
          break;
        }
        for (size_t j = 0; j < path_vector.size(); j++) {
          if (path_vector[i].pid_ == path_vector[j].id_) {
            i = j;
            // path_vector[j].NodeInfo();
            grid_env[path_vector[j].x_][path_vector[j].y_] = 100;
          }
        }
      }
      break;
    }
  }
  grid_env[start.x_][start.y_] = 100;
  ShowGridState(grid_env);
}
