#include <cmath>
#include <algorithm>
#include "constants.hpp"

template <typename NodeT, typename GoalT>
constexpr double manhattanDistance(NodeT node, GoalT goal, double speed){
    return std::abs(goal.x - node.x) + std::abs(goal.y - node.y) / speed;
}

template <typename NodeT, typename GoalT>
constexpr double euclideanDistance(NodeT node, GoalT goal, double speed){
    return std::hypot(goal.x - node.x, goal.y - node.y) / speed;
}

template <typename NodeT, typename GoalT>
constexpr double eightWayDistance(NodeT node, GoalT goal, double speed){
    int dx = std::abs(goal.x - node.x); 
    int dy = std::abs(goal.y - node.y);
    long diag = std::min(dx, dy);
    long flat = dy + dx - 2*diag;
    return (double)(flat +  sqrt2()*diag)/speed;
}