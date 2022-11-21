#pragma once
#include <iostream>
#include <unordered_set>

#include "metadata.hpp"
#include "node.hpp"
#include "safeIntervals.hpp"
#include "map.hpp"
#include "constants.hpp"
#include "heuristic.hpp"
#include "structs.hpp"


inline void debug_open(const NodeOpen<sippNode>& open){
    NodeOpen<sippNode> open_copy;
    for (auto element :open){
        open_copy.emplace(element);
    }
    std::cout << "open:\n";
    while(!open_copy.empty()){
        sippNode::nodes[open_copy.top()].debug();
        open_copy.pop();
    }
    std::cout << "\n";
}

inline void debug_closed(const std::unordered_set<sippNode, NodeHash<sippNode>, NodeEquals<sippNode>>& closed){
    std::cout << "closed:\n";
    for (auto elem: closed){
        elem.debug();
    }
    std::cout << "\n";
}

template<typename NodeT>
inline bool isGoal(const NodeT& node, const State& goal ){
    return node.s.x == goal.x && node.s.y == goal.y;
}

template <typename NodeT>
inline std::vector<State> backtrack_path(std::size_t node){
    const NodeT& n = NodeT::getNode(node);
    std::vector<State> path = {n.s};
    if (n.parent != std::numeric_limits<std::size_t>::max()){
        NodeT node = NodeT::getNode(n.parent);
        path.emplace_back(node.s);
        while(node.parent != std::numeric_limits<std::size_t>::max()){
            node = NodeT::getNode(node.parent);
            path.emplace_back(node.s);        
        }
    }
    for (auto it = path.rbegin(); it != path.rend(); it++){
        it->debug();
    }
    return std::vector<State>(path.rbegin(), path.rend());
}

inline void sippGenerateSuccessors(std::size_t cnode, const State& goal, double agent_speed,SafeIntervals& safe_intervals, const Map& map,
                                    NodeOpen<sippNode>& open, std::vector<double>& waits_res, std::vector<std::size_t>& waits_res_ind){
    double dt;
    const sippNode& current_node = sippNode::getNode(cnode);
    Action act(current_node.s, State(0, 0, 0));
    //auto interval_starts = std::vector<std::pair<double, double>>();
    for (int m_x = -1; m_x <=1; m_x++){
        act.destination.x = current_node.s.x + m_x;
        for (int m_y = -1; m_y <=1; m_y++){
            if (m_x == 0 && m_y == 0){
                continue;
            }
            else if (m_x == 0 || m_y == 0) {
                dt = agent_speed;
            }
            else{
                dt = sqrt2()*agent_speed;
            }
            act.destination.y = current_node.s.y + m_y;
            if (!map.inBounds(act.destination.x, act.destination.y)){
                continue;
            }
            act.destination.time = current_node.s.time + dt;
            std::size_t loc_ind = safe_intervals.waits(map, act, waits_res, waits_res_ind);
            int wrs = waits_res.size();
            for (int i = 0; i < wrs; i++){
                act.destination.time = std::max(act.destination.time, waits_res[i]); 
                if (safe_intervals.isSafe(act, map)){
                    double f = act.destination.time + eightWayDistance(act.destination, goal, agent_speed);
                    std::size_t node_ind = safe_intervals.visited(loc_ind, i);
                    double intervalStart = safe_intervals.get_intervals(loc_ind, waits_res_ind[i])->first;
                    if (node_ind != std::numeric_limits<std::size_t>::max()){
                        if (sippNode::getNode(node_ind).s.time > act.destination.time){
                            sippNode::set_arrival(node_ind, act.destination.time, f, cnode);
                            open.emplace(node_ind);
                        }
                        continue;
                    }
                    auto j = sippNode::newNode(act.destination.x, act.destination.y, intervalStart, act.destination.time, f, cnode);
                    open.emplace(j);
                    safe_intervals.markvisited(loc_ind, i, j);
                }
            }
        }
    }
}

inline void pdapGenerateSuccessors(std::size_t cnode, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map,
                                           NodeOpen<pdapNode>& open, std::vector<double>& waits_res, std::vector<std::size_t>& waits_res_ind){
    double dt;
    const pdapNode& current_node = pdapNode::getNode(cnode);
    Action act(current_node.s, State(0, 0, 0));
    //auto interval_starts = std::vector<std::pair<double, double>>();
    for (int m_x = -1; m_x <=1; m_x++){
        act.destination.x = current_node.s.x + m_x;
        for (int m_y = -1; m_y <=1; m_y++){
            if (m_x == 0 && m_y == 0){
                continue;
            }
            else if (m_x == 0 || m_y == 0) {
                dt = agent_speed;
            }
            else{
                dt = sqrt2()*agent_speed;
            }
            act.destination.y = current_node.s.y + m_y;
            if (!map.inBounds(act.destination.x, act.destination.y)){
                continue;
            }
            act.destination.time = current_node.s.time + dt;
            std::size_t loc_ind = safe_intervals.waits(map, act, waits_res, waits_res_ind);
            int wrs = waits_res.size();
            for (int i = 0; i < wrs; i++){
                double delta_prior = current_node.s.time - current_node.alpha;
                act.destination.time = std::max(act.destination.time, waits_res[i]);
                if (safe_intervals.isSafe(act, map)){ //&& !safe_intervals.markvisited(loc_ind, i)){
                    double intervalStart = safe_intervals.get_intervals(loc_ind,waits_res_ind[i])->first;
                    double intervalEnd = safe_intervals.get_intervals(loc_ind, waits_res_ind[i])->second;
                    double alpha = std::max(current_node.alpha, intervalStart - delta_prior);
                    double beta = std::min(current_node.beta, intervalEnd - delta_prior);
                    double f = act.destination.time + eightWayDistance(act.destination, goal, agent_speed);
                    std::size_t node_ind = safe_intervals.visited(loc_ind, i);
                    if (node_ind != std::numeric_limits<std::size_t>::max()){
                        if (pdapNode::getNode(node_ind).s.time > act.destination.time){
                            pdapNode::set_arrival(node_ind, act.destination.time, alpha, beta, f, cnode);
                            open.emplace(node_ind);
                        }
                        continue;
                    }
                    auto j = pdapNode::newNode(act.destination.x, act.destination.y, intervalStart, act.destination.time, alpha, beta, f, cnode);
                    open.emplace(j);
                    safe_intervals.markvisited(loc_ind, i, j);
                }
            }
        }
    }
    //std::cout << "\n";
}


inline std::vector<State> sippAStar(const State& start_state, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map, Metadata& metadata, bool resume = false){
    ++(metadata.plan_attempts);
    if (!resume){
        metadata.runtime.start();
    }
    else{
        metadata.runtime.resume();
    }
    std::vector<double> waits_res;
    std::vector<std::size_t> waits_res_ind;
    NodeOpen<sippNode> open;
    double f = start_state.time + eightWayDistance(start_state, goal, agent_speed);
    auto current_node = sippNode::newNode(start_state.x,start_state.y, 0.0, start_state.time, f, std::numeric_limits<std::size_t>::max());
    safe_intervals.markvisited(map.get_safe_interval_ind(start_state), 0, current_node);
    open.emplace(current_node);
    while(!open.empty()){
        current_node = open.top();
        open.pop();
        ++(metadata.expansions);
        if (isGoal(sippNode::getNode(current_node), goal)){
            metadata.runtime.stop();
            return backtrack_path<sippNode>(current_node);
        }
        if (sippNode::getNode(current_node).s.time > 1000.0){
            std::cout << "overtime \n";
        }
        sippGenerateSuccessors(current_node, goal, agent_speed, safe_intervals, map, open, waits_res, waits_res_ind);
    }
    metadata.runtime.stop();
    return {};
}

inline void pdapAStar(const State& start_state, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map, Metadata& metadata){
    metadata.runtime.start();
    std::vector<double> waits_res;
    std::vector<std::size_t> waits_res_ind;
    NodeOpen<pdapNode> open;
    auto start_interval = safe_intervals.get_interval(0.0, map.get_safe_interval_ind(start_state));
    double f = start_state.time + eightWayDistance(start_state, goal, agent_speed);
    auto current_node = pdapNode::newNode(start_state.x, start_state.y, start_interval->first, start_state.time, 0.0, start_interval->second, f, std::numeric_limits<std::size_t>::max());
    safe_intervals.markvisited(map.get_safe_interval_ind(start_state), 0, current_node);
    open.emplace(current_node);
    while(!open.empty()){
        current_node = open.top();
        open.pop();
        ++(metadata.expansions);
        if (isGoal(pdapNode::getNode(current_node), goal)){
            metadata.runtime.stop();
            backtrack_path<pdapNode>(current_node);
            return;
        }
        pdapGenerateSuccessors(current_node, goal, agent_speed, safe_intervals, map, open, waits_res, waits_res_ind);
    }
    std::cout << "No path found\n";
}