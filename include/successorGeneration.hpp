#pragma once
#include <vector>
#include "structs.hpp"

enum Movement {fourWay, eightWay};

inline  std::vector<Location> fourWayMovements(){
    return {
        Location(0,1),
        Location(0,-1),
        Location(1,0),
        Location(-1,0)
    };
}

inline std::vector<Location> eightWayMovements(){
    return {
        Location(0,1),
        Location(0,-1),
        Location(1,0),
        Location(-1,0),
        Location(1,1),
        Location(1,-1),
        Location(-1,1),
        Location(-1,-1)
    };
}

inline std::vector<Location> moves(std::string m){
    if (m == "four"){
        return fourWayMovements();
    } 
    if (m == "eight"){
        return eightWayMovements();
    } 
    return {};
}