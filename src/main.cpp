#include <memory>
#include <vector>
#include "metadata.hpp"
#include "map.hpp"
#include "dynamicObstacle.hpp"
#include "node.hpp"
#include "randomDynamicObstacle.hpp"
#include "safeIntervals.hpp"
#include "search.hpp"
  
std::vector<sippNode>  sippNode::nodes = std::vector<sippNode>();
std::vector<pdapNode>  pdapNode::nodes = std::vector<pdapNode>();

inline bool check_path(const std::vector<State>& path, double start_time, const Map& map, const SafeIntervals& safe_intervals){
    State original(path[0].x, path[0].y, 0.0);
    Action act(original, path[0]);
    act.destination.time += start_time;
    if (!safe_intervals.isSafe(act, map)){
        return false;
    }
    for (std::size_t i = 1; i < path.size();i++){
        act.source = path[i-1];
        act.destination = path[i];
        act.destination.time += start_time;
        if (!safe_intervals.isSafe(act, map)){
            return false;
        }
    }
    return true;
}

int main(int argc, char *argv[]){
    //std::cout << "state, sippnode, pdap: " << sizeof(State) << "," << sizeof(sippNode) << "," << sizeof(pdapNode) << "\n" << sizeof(short) << "\n";
    Metadata metadata = Metadata(argc, argv);
    Map map = Map(metadata.mapfile());
    std::vector<std::shared_ptr<DynamicObstacle>> obs = RandomDynamicObstacle::read_obstacles(metadata.args()["obstacles"].as<std::string>(), map, 
                                                                                            metadata.args()["minwait"].as<double>(),
                                                                                            metadata.args()["maxwait"].as<double>());
    double unsafe_time = metadata.args()["allendt"].as<double>();
    SafeIntervals safe_intervals = SafeIntervals(obs, map, unsafe_time);
    State start_state(metadata.args()["startx"].as<int>(), metadata.args()["starty"].as<int>(), 0.0);
    double start_time = metadata.args()["startt"].as<double>();
    assert(map.isSafe(start_state.x, start_state.y));
    safe_intervals.always_safe_until(start_state, metadata.args()["startendt"].as<double>(), map);
    State goal(metadata.args()["goalx"].as<int>(), metadata.args()["goaly"].as<int>(), 0.0);
    assert(map.isSafe(goal.x, goal.y));
    double agent_speed = metadata.args()["aspeed"].as<double>();
    if(metadata.args()["search"].as<std::string>() == "sipp"){
        auto path = sippAStar(start_state, goal, agent_speed,safe_intervals, map, metadata);
        std::cout << "path_check: " <<  check_path(path, start_time, map, safe_intervals) << "\n";
        std::cout << metadata.runtime.format() << "\n";
        std::cout << "SIPP\nExpansions:" << metadata.expansions << "\n"; 
    }
    else if (metadata.args()["search"].as<std::string>() == "pdap"){
        pdapAStar(start_state, goal, agent_speed,safe_intervals, map, metadata);
        std::cout << metadata.runtime.format() << "\n";
        std::cout << "PDAP\nExpansions:" << metadata.expansions << "\n";
    }
    else{
        std::cout << "Incorrect search algorithm specified: " << metadata.args()["search"].as<std::string>() << "\n";
    }
}