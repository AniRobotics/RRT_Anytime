/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   helper.h
 * Author: simbot
 *
 * Created on August 2, 2020, 8:55 PM
 */

#ifndef HELPER_H
#define HELPER_H

#include <vector>
#include "color_spec.h"


class Node {  
 public:
  Node(const int x = 0, const int y = 0, const double cost = 0,
       const double h_cost = 0, const int id = 0, const int pid = 0);
  int x_, y_, id_, pid_;
  double cost_, h_cost_;
};

void BuildGridEnv(std::vector<std::vector<int>>& grid);
void ShowGridState(const std::vector<std::vector<int>>& grid);
void PlotPathNodes(std::vector<Node>& path_vector, const Node& start_,
               const Node& goal_, std::vector<std::vector<int>>& grid);
bool CompareNodeCoordinates(const Node& p1, const Node& p2);

#endif /* HELPER_H */

