#pragma once

#include <vector>
#include "structs.hpp"

class DynamicObstacle{
    public:
        virtual std::vector<State> path(double starting, double ending) = 0;
        virtual ~DynamicObstacle() = default;
};