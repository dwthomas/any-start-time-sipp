#include <boost/random/uniform_real_distribution.hpp>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "randomDynamicObstacle.hpp"
#include "dynamicObstacle.hpp"
#include "constants.hpp"

std::vector<std::shared_ptr<DynamicObstacle>> RandomDynamicObstacle::read_obstacles(const std::string & filename, const Map& map, float min_wait, float max_wait){
    auto obs = std::vector<std::shared_ptr<DynamicObstacle>>();
    int x;
    int y;
    double speed;
    std::ifstream obsfile;
    std::string line;
    std::stringstream linestream;
    obsfile.open(filename);
    while(std::getline(obsfile, line)){
        linestream = std::stringstream(line);
        linestream >> x;
        linestream >> y;
        linestream >> speed;
        obs.emplace_back(std::make_shared<RandomDynamicObstacle>(x, y, map, speed, min_wait, max_wait));
    }
    obsfile.close();
    return obs;
}

RandomDynamicObstacle::RandomDynamicObstacle(int x, int y, const Map& _map, double _speed, float min_wait, float max_wait):good_until(0.0),speed(_speed),map(_map){
    std::uint64_t seed = x;
    // No duplicate starting locations allowed!
    seed = seed ^ __builtin_bswap64((std::uint64_t)y);
    generator = boost::random::mt11213b(seed);
    specified_path.emplace_hint(specified_path.end(), 0.0, State(x, y, 0.0));
    movement_direction = boost::random::uniform_smallint<unsigned int>(0,8);
    wait_duration = boost::random::uniform_real_distribution<float>(min_wait, max_wait);
}

unsigned int RandomDynamicObstacle::get_max(int x, int y, int dx, int dy) const{
    assert(dx != 0 || dy != 0);
    int steps = 0;
    while(map.inBounds(Configuration(x + (steps + 1)*dx, y + (steps + 1)*dy)) &&  map.isSafe(Configuration(x + (steps + 1)*dx, y + (steps + 1)*dy))){
        ++steps;
    }
    return steps;
}

void RandomDynamicObstacle::generate(double until){
    int dx = -999;
    int dy = -999;
    double dt = std::numeric_limits<double>::signaling_NaN();
    if (until <= good_until){
        return;
    }
    while(good_until < until){
        unsigned int direction = movement_direction(generator);
        switch (direction){
            case 0:
                dx = 0;
                dy = 0;
                dt = wait_duration(generator);
                break;
            case 1:
                dx = 1;
                dy = 0;
                dt = 1.0*speed;
                break;
            case 2:
                dx = -1;
                dy = 0;
                dt = 1.0*speed;
                break;
            case 3:
                dx = 1;
                dy = 1;
                dt = sqrt2()*speed;
                break;
            case 4:
                dx = -1;
                dy = 1;
                dt = sqrt2()*speed;
                break;
            case 5:
                dx = 0;
                dy = 1;
                dt = 1.0*speed;
                break;
            case 6:
                dx = 0;
                dy = -1;
                dt = 1.0*speed;
                break;
            case 7:
                dx = -1;
                dy = -1;
                dt = sqrt2()*speed;
                break;
            case 8:
                dx = 1;
                dy = -1;
                dt = sqrt2()*speed;
                break;
        }
        if (dx == 0 && dy == 0){
            auto current_state = specified_path.rbegin();
            specified_path.emplace_hint(specified_path.end(), current_state->second.time + dt, State(current_state->second.x.x, current_state->second.x.y, current_state->second.time + dt));
            good_until += dt;
            continue;
        }
        unsigned int max_in_direction = get_max(specified_path.rbegin()->second.x.x,specified_path.rbegin()->second.x.y, dx, dy); 
        unsigned int macro_move_steps = boost::random::uniform_smallint<unsigned int>(0, max_in_direction)(generator);
        good_until += macro_move_steps*dt;
        for (unsigned int step=0; step < macro_move_steps; step++){
            //get state accounting for speed
            auto current_state = specified_path.rbegin();
            specified_path.emplace_hint(specified_path.end(), 
                                        current_state->second.time + dt,
                                        State(current_state->second.x.x + dx, current_state->second.x.y + dy, current_state->second.time + dt));
        } 
    }
}

std::vector<State> RandomDynamicObstacle::path(double starting, double ending){
    std::vector<State> retval;
    if (ending > good_until){
        generate(ending);
    }
    retval.reserve(specified_path.size());
    auto begin = specified_path.lower_bound(starting);
    if (begin != specified_path.begin()){
        --begin;
    }
    auto last = specified_path.upper_bound(ending);
    if (last != specified_path.end()){
        ++last;
    }
    for (auto it = begin; it != last; it++){
        retval.emplace_back(it->second);
    }
    return retval;
}