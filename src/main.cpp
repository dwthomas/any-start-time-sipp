#include <memory>
#include <vector>
#include "metadata.hpp"
#include "map.hpp"
#include "dynamicObstacle.hpp"
#include "node.hpp"
#include "randomDynamicObstacle.hpp"
#include "safeIntervals.hpp"
#include "search.hpp"
#include "successorGeneration.hpp"
  
std::vector<sippNode>  sippNode::nodes = std::vector<sippNode>();
std::vector<pdapNode>  pdapNode::nodes = std::vector<pdapNode>();
std::vector<partialPdapNode>  partialPdapNode::nodes = std::vector<partialPdapNode>();

inline bool check_path(const std::vector<State>& path, double start_time, const SafeIntervals& safe_intervals, double agent_speed){
    if (path.size() == 0){
        return false;
    }
    State original(path[0].x.x, path[0].x.y, 0.0);
    Action act(original, path[0]);
    act.destination.time += start_time;
    if (!safe_intervals.valid(act, agent_speed)){
        return false;
    }
    for (std::size_t i = 1; i < path.size();i++){
        act.source = path[i-1];
        act.source.time += start_time;
        act.destination = path[i];
        act.destination.time += start_time;
        if (!safe_intervals.valid(act, agent_speed)){
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
    double agent_speed = metadata.args()["aspeed"].as<double>();
    std::vector<Location> movement = moves(metadata.args()["movement"].as<std::string>());
    State start_state(metadata.args()["startx"].as<int>(), metadata.args()["starty"].as<int>(), 0.0);
    SafeIntervals safe_intervals = SafeIntervals(obs, map, unsafe_time, agent_speed, start_state, metadata.args()["startendt"].as<double>());
    double start_time = metadata.args()["startt"].as<double>();
    assert(map.isSafe(start_state.x));
    Configuration goal(metadata.args()["goalx"].as<int>(), metadata.args()["goaly"].as<int>());
    assert(map.isSafe(goal));
    if(metadata.args()["search"].as<std::string>() == "sipp"){
        auto path = sippAStar(start_state, goal, agent_speed,safe_intervals, map, movement, metadata);
        double rts = start_time;
        while(false && !check_path(path, start_time+rts, safe_intervals, agent_speed) &&
                         start_time + rts <= metadata.args()["startendt"].as<double>()){
            if (path.size() == 0){
                break;
            }
            std::cout << "path failed trying start: ";
            std::cout << "rts: " << rts << " " << path.size() <<"\n";
            start_state.time = start_time + rts;
            start_state.debug();
            sippNode::nodes = std::vector<sippNode>();
            safe_intervals.zero_visits();
            path = sippAStar(start_state, goal, agent_speed,safe_intervals, map, movement, metadata, true);
            rts = metadata.runtime.elapsed().user*0.000000001;// conver to second from nanosecond
        }
        std::cout << metadata.runtime.format() << "\n";
        std::cout << "SIPP\nExpansions:" << metadata.expansions << " Nodes Generated:" << metadata.generated << "\n";
        double until = 0;
        double guess = 0.001;
        //assert(check_path(path, start_time, safe_intervals, agent_speed));
        for (int i = 0; i < 100;i++){
            if (check_path(path, start_time+guess, safe_intervals, agent_speed)){
                until = guess;
                guess = 2*guess; 
            }
            else{
                guess = (guess + until)*0.5;
            }
            if (guess - until < 0.0001){
                break;
            }
        }
        std::cout << "Valid until: " << until << "\n"; 
    }
    else if (metadata.args()["search"].as<std::string>() == "pdap"){
        start_state.time = start_time;
        pdapAStar(start_state, goal, agent_speed,safe_intervals, map, metadata);
        std::cout << metadata.runtime.format() << "\n";
        std::cout << "PDAP\nExpansions:" << metadata.expansions << " Nodes Generated:" << metadata.generated << "\n";
    }
    else if (metadata.args()["search"].as<std::string>() == "partialpdap"){
        //auto functional = partialPdapAStar(start_state, goal, agent_speed,safe_intervals, map, metadata);
        //std::cout << metadata.runtime.format() << "\n";
        //std::cout << "partialPDAP\nExpansions:" << metadata.expansions << " Nodes Generated:" << metadata.generated << "\n";
        //functional.debug();
    }
    else{
        std::cout << "Incorrect search algorithm specified: " << metadata.args()["search"].as<std::string>() << "\n";
    }
}