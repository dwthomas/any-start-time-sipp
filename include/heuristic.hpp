#pragma once
#include <cmath>
#include <algorithm>
#include "constants.hpp"
#include "structs.hpp"

inline double manhattanDistance(State node, State goal, double speed){
    return std::abs(goal.x - node.x) + std::abs(goal.y - node.y) / speed;
}
inline  double euclideanDistance(State node, State goal, double speed){
    return std::hypot(goal.x - node.x, goal.y - node.y) / speed;
}

inline double eightWayDistance(State node, State goal, double speed){
    int dx = std::abs(goal.x - node.x); 
    int dy = std::abs(goal.y - node.y);
    long diag = std::min(dx, dy);
    long flat = dy + dx - 2*diag;
    return (double)(flat +  sqrt2()*diag)/speed;
}