/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: simbot
 *
 * Created on August 5, 2020, 8:55 PM
 */

#include <iostream>
#include <random>
#include "rrt_anytime.h"

int main() {
  // Set the grid size
  constexpr int square_n = 21;
  
  // Set the start and goal coordinates
  int start_x = 1; int start_y = 7;
  int goal_x = 10; int goal_y = 10;
  
  // Set parameters for RRT and RRT*
  constexpr double threshold_viable_path = 1;
  constexpr double threshold_improved_path = 1;
  constexpr int max_iter_viable_path = 20;
  constexpr int max_iter_improved_path = 20;
  
  std::cout << "start coordinate =(" << start_x << "," << start_y << ")" << std::endl;
  std::cout << "goal coordinate =(" << goal_x << "," << goal_y << ")" << std::endl;
  std::cout << "----------------------------------------------------" << std::endl;
  
  // Build the grid environment  
  std::vector<std::vector<int>> grid_env(square_n, std::vector<int>(square_n, 0));
  BuildGridEnv(grid_env);
  
  // Create the start node
  int start_id = start_x * square_n + start_y;
  int start_pid = start_x * square_n + start_y;
  int start_cost = 0;
  int start_h_cost = abs(start_x - goal_x) + abs(start_y - goal_y);
  Node start(start_x, start_y, start_cost, start_h_cost, start_id, start_pid);
  
  // Create goal node
  int goal_id = goal_x * square_n + goal_y;
  Node goal(goal_x, goal_y, 0, 0, goal_id, 0);
 
  grid_env[start.x_][start.y_] = 0;
  grid_env[goal.x_][goal.y_] = 0;
  std::cout << "----------------------------------------------------" << std::endl;
  std::cout << "Printing the initial state of the grid" << std::endl;
  std::cout << "----------------------------------------------------" << std::endl;
  ShowGridState(grid_env);

  // Compute a viable path
  MyRRT rrt_obj;
  std::vector<Node> path =
      rrt_obj.rrt_non_optimal(grid_env, start, goal, max_iter_viable_path, threshold_viable_path);
  PlotPathNodes(path, start, goal, grid_env);
  
  // Improve on the viable path
  std::vector<Node> improved_path = rrt_obj.improved_path(grid_env, max_iter_improved_path, threshold_improved_path);
  PlotPathNodes(improved_path, start, goal, grid_env);
  
  return 0;
}
