#pragma once

#include "structs.hpp"

struct sippNode{
    int x;
    int y;
    double intervalStart;
    double t;
    double f;
    std::shared_ptr<Node> parent;
};