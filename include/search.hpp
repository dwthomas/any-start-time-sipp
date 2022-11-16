#pragma once
#include <iostream>
#include <unordered_set>

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
        open_copy.top().debug();
        open_copy.pop();
    }
    std::cout << "\n";
}

template<typename NodeT>
inline bool isGoal(const NodeT& node, const State& goal ){
    return node.s.x == goal.x && node.s.y == goal.y;
}

template <typename NodeT>
inline void backtrack_path(const NodeT& n){
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
}

inline void generateSuccessors(const sippNode& current_node, const State& goal, double agent_speed,SafeIntervals& safe_intervals, const Map& map, std::unordered_set<sippNode, NodeHash<sippNode>, NodeEquals<sippNode>>& closed, NodeOpen<sippNode>& open){
    double dt;
    State s(0, 0, 0.0);
    for (int m_x = -1; m_x <=1; m_x++){
        s.x = current_node.s.x + m_x;
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
            s.y = current_node.s.y + m_y;
            if (!map.inBounds(s.x, s.y)){
                continue;
            }
            s.time = current_node.s.time + dt;
            auto interval_starts = safe_intervals.waits(map, Action(current_node.s, s));
            for (auto intervalStart: interval_starts){
                s.time = std::max(s.time, intervalStart);
                double f = s.time + eightWayDistance(current_node.s, goal, agent_speed);
                auto n = sippNode::newNode(s.x, s.y, intervalStart, s.time, f, current_node.ind); 
                if (safe_intervals.isSafe(Action(current_node.s, s), map)){
                    auto inClosed = closed.find(n);
                    if (inClosed != closed.end() && inClosed->s.time <= n.s.time){
                        continue;
                    }
                    closed.emplace(n);
                    open.emplace(n);
                }
            }
        }
    }
}

template<typename NodeT>
inline void aStar(const State& start_state, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map){
    std::unordered_set<sippNode, NodeHash<sippNode>, NodeEquals<sippNode>> closed;
    NodeOpen<NodeT> open;
    std::vector<State> path = {start_state};
    double f = start_state.time + eightWayDistance(start_state, goal, agent_speed);
    auto current_node = NodeT::newNode(start_state.x,start_state.y, map.get_safe_interval_ind(start_state), start_state.time, f, std::numeric_limits<std::size_t>::max());
    closed.emplace(current_node);
    open.emplace(current_node);
    while(!open.empty()){
        current_node = open.top();
        open.pop();
        //current_node.debug();
        if (isGoal(current_node, goal)){
            backtrack_path(current_node);
            return;
        }
        generateSuccessors(current_node, goal, agent_speed, safe_intervals, map, closed, open);
    }
}