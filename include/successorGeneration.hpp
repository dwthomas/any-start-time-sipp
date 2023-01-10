#pragma once
#include <vector>
#include "structs.hpp"

constexpr std::array<Location,4> fourWayMovements{
        Location(0,1),
        Location(0,-1),
        Location(1,0),
        Location(-1,0)
    };

constexpr std::array<Location,8> eightWayMovements{
        Location(0,1),
        Location(0,-1),
        Location(1,0),
        Location(-1,0),
        Location(1,1),
        Location(1,-1),
        Location(-1,1),
        Location(-1,-1)
    };