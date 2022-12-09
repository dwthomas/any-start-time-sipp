#pragma once
#include <cmath>
#include <algorithm>
#include "constants.hpp"
#include "structs.hpp"

inline double manhattanDistance(const Configuration& c1, const Configuration& c2, double speed){
    return std::abs(c1.x - c2.x) + std::abs(c1.y - c2.y) / speed;
}
inline  double euclideanDistance(const Configuration& c1, const Configuration& c2, double speed){
    return std::hypot(c1.x - c2.x, c1.y - c2.y) / speed;
}

inline double eightWayDistance(const Configuration& c1, const Configuration& c2, double speed){
    int dx = std::abs(c1.x - c2.x); 
    int dy = std::abs(c1.y - c2.y);
    long diag = std::min(dx, dy);
    long flat = dy + dx - 2*diag;
    return (double)(flat +  sqrt2()*diag)/speed;
}