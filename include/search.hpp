#pragma once
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_set>

#include "metadata.hpp"
#include "node.hpp"
#include "safeIntervals.hpp"
#include "map.hpp"
#include "constants.hpp"
#include "heuristic.hpp"
#include "structs.hpp"

template <typename NodeT, typename NodeSort>
inline void debug_open(const NodeOpen<NodeT, NodeSort>& open){
    NodeOpen<NodeT, NodeSort> open_copy;
    for (auto element :open){
        open_copy.emplace(element);
    }
    std::cout << "open:\n";
    while(!open_copy.empty()){
        NodeT::nodes[open_copy.top()].debug();
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

inline std::vector<State> sipp_backtrack_path(std::size_t node){
    const sippNode& n = sippNode::getNode(node);
    std::vector<State> path = {n.s};
    if (n.parent != std::numeric_limits<std::size_t>::max()){
        sippNode node = sippNode::getNode(n.parent);
        path.emplace_back(node.s);
        while(node.parent != std::numeric_limits<std::size_t>::max()){
            node = sippNode::getNode(node.parent);
            path.emplace_back(node.s);        
        }
    }
    for (auto it = path.rbegin(); it != path.rend(); it++){
        it->debug();
    }
    return std::vector<State>(path.rbegin(), path.rend());
}

template <typename NodeT>
inline void pdap_backtrack_path(std::size_t node){
    const NodeT& n = NodeT::getNode(node);
    std::vector<NodeT> path = {n};
    if (n.parent != std::numeric_limits<std::size_t>::max()){
        NodeT node = NodeT::getNode(n.parent);
        path.emplace_back(node);
        while(node.parent != std::numeric_limits<std::size_t>::max()){
            node = NodeT::getNode(node.parent);
            path.emplace_back(node);        
        }
    }
    for (auto it = path.rbegin(); it != path.rend(); it++){
        it->report();
    }
    //return std::vector<State>(path.rbegin(), path.rend());
}


inline void sippGenerateSuccessors(std::size_t cnode, const State& goal, double agent_speed,SafeIntervals& safe_intervals, const Map& map,
                                    NodeOpen<sippNode, NodeGreater<sippNode>>& open, std::vector<NodeOpen<sippNode, NodeGreater<sippNode>>::handle_type>& handles, std::vector<bool>& node_on_open,
                                    std::vector<std::size_t>& destination_ind, std::vector<std::size_t>& edge_ind){
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
            safe_intervals.waits(act, current_node.intervalInd, destination_ind, edge_ind, dt);
            for (std::size_t i = 0; i < edge_ind.size(); i++){
                const auto & edge_interval = safe_intervals.get_edge(act, current_node.intervalInd, destination_ind[i], edge_ind[i]);
                act.destination.time = std::max(current_node.s.time + dt, edge_interval.first + dt); 
                if (edge_interval.second < act.destination.time){
                    continue;
                }
                assert(safe_intervals.valid(act, agent_speed));
                double f = act.destination.time + eightWayDistance(act.destination, goal, agent_speed);
                std::size_t node_ind = safe_intervals.visited(act, current_node.intervalInd, destination_ind[i], edge_ind[i]);
                std::size_t intervalInd = destination_ind[i];
                if (node_ind != std::numeric_limits<std::size_t>::max()){
                    if (sippNode::getNode(node_ind).s.time > act.destination.time){
                        sippNode::set_arrival(node_ind, act.destination.time, f, cnode);
                        if (node_on_open.at(node_ind)){
                            open.increase(handles.at(node_ind));
                        }
                        else{
                            handles.at(node_ind) = open.emplace(node_ind);
                            node_on_open.at(node_ind) = true;
                        }
                    }
                    continue;
                }
                auto j = sippNode::newNode(act.destination.x, act.destination.y, intervalInd, act.destination.time, f, cnode);
                handles.emplace_back(open.emplace(j));
                node_on_open.emplace_back(true);
                assert(handles.size() == sippNode::nodes.size());
                safe_intervals.markvisited(act, current_node.intervalInd, destination_ind[i], edge_ind[i], j);
            }
        }
    }
}


inline std::vector<State> sippAStar(const State& start_state, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map, Metadata& metadata, bool resume = false){
    long prior_open_size;
    ++(metadata.plan_attempts);
    if (!resume){
        metadata.runtime.start();
    }
    else{
        metadata.runtime.resume();
    }
    std::vector<std::size_t> destination_ind;
    std::vector<std::size_t> edge_ind;
    NodeOpen<sippNode, NodeGreater<sippNode>> open;
    NodeClosed<sippNode> closed;
    std::vector<NodeOpen<sippNode, NodeGreater<sippNode>>::handle_type> handles;
    std::vector<bool> node_on_open;
    sippNode::nodes.clear();
    double f = start_state.time + eightWayDistance(start_state, goal, agent_speed);
    auto current_node = sippNode::newNode(start_state.x,start_state.y, 0, start_state.time, f, std::numeric_limits<std::size_t>::max());
    handles.emplace_back(open.emplace(current_node));
    node_on_open.emplace_back(true);
    while(!open.empty()){
        current_node = open.top();
        open.pop();
        node_on_open.at(current_node) = false;
        ++(metadata.expansions);
        if (isGoal(sippNode::getNode(current_node), goal)){
            metadata.runtime.stop();
            return sipp_backtrack_path(current_node);
        }
        prior_open_size = open.size();
        sippGenerateSuccessors(current_node, goal, agent_speed, safe_intervals, map, open, handles, node_on_open, destination_ind, edge_ind);
        metadata.generated += std::max<long>((long)open.size() - prior_open_size, 0);
    }
    metadata.runtime.stop();
    return {};
}

inline void pdapGenerateSuccessors(std::size_t cnode, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map,
                                           NodeOpen<pdapNode, NodeGreater<pdapNode>>& open, std::vector<NodeOpen<pdapNode, NodeGreater<pdapNode>>::handle_type>& handles, std::vector<bool>& node_on_open,
                                            std::vector<std::size_t>& destination_ind, std::vector<std::size_t>& edge_ind){
    double dt;
    const pdapNode& current_node = pdapNode::getNode(cnode);
    double delta_prior = current_node.delta;
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
            double delta = delta_prior + dt;
            safe_intervals.waits(act, current_node.intervalInd, destination_ind, edge_ind, dt);
            for (std::size_t i = 0; i < edge_ind.size(); i++){
                const safe_interval& edge_interval = safe_intervals.get_edge(act, current_node.intervalInd, destination_ind[i], edge_ind[i]);
                double alpha = std::max(current_node.alpha, edge_interval.first - delta_prior);
                act.destination.time = std::max(current_node.s.time + dt, edge_interval.first + dt); 
                if (edge_interval.second < act.destination.time){
                    continue;
                }
                assert(safe_intervals.valid(act, agent_speed));
                double beta = std::min(current_node.beta, edge_interval.second - alpha - delta_prior);
                double f = act.destination.time + eightWayDistance(act.destination, goal, agent_speed);
                std::size_t node_ind = safe_intervals.visited(act, current_node.intervalInd, destination_ind[i], edge_ind[i]);
                std::size_t intervalInd = destination_ind[i];
                if (node_ind != std::numeric_limits<std::size_t>::max()){
                    if (pdapNode::getNode(node_ind).s.time > act.destination.time){
                        pdapNode::set_arrival(node_ind, act.destination.time, alpha, beta, delta, f, cnode);
                        if (node_on_open.at(node_ind)){
                            open.increase(handles.at(node_ind));
                        }
                        else{
                            handles.at(node_ind) = open.emplace(node_ind);
                            node_on_open.at(node_ind) = true;
                        }
                    }
                    continue;
                }
                auto j = pdapNode::newNode(act.destination.x, act.destination.y, intervalInd, act.destination.time, alpha, beta, delta, f, cnode);
                handles.emplace_back(open.emplace(j));
                node_on_open.emplace_back(true);
                assert(handles.size() == pdapNode::nodes.size());
                safe_intervals.markvisited(act, current_node.intervalInd, destination_ind[i], edge_ind[i], j);
            }
        }
    }
}





inline void pdapAStar(const State& start_state, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map, Metadata& metadata){
    metadata.runtime.start();
    long prior_open_size;
    std::vector<std::size_t> destination_ind;
    std::vector<std::size_t> edge_ind;
    NodeOpen<pdapNode, NodeGreater<pdapNode>> open;
    std::vector<NodeOpen<pdapNode, NodeGreater<pdapNode>>::handle_type> handles;
    std::vector<bool> node_on_open;
    pdapNode::nodes.clear();
    //boost::heap::priority_queue<std::size_t, boost::heap::compare<pdapNodeGreater>> open;
    auto start_interval = safe_intervals.get_interval(0.0, map.get_safe_interval_ind(start_state));
    double f = start_state.time + eightWayDistance(start_state, goal, agent_speed);
    auto current_node = pdapNode::newNode(start_state.x, start_state.y, start_interval->first, start_state.time, 0.0, start_interval->second, 0.0, f, std::numeric_limits<std::size_t>::max());
    handles.emplace_back(open.emplace(current_node));
    node_on_open.emplace_back(true);
    assert(handles.size() == pdapNode::nodes.size());
    while(!open.empty()){
        current_node = open.top();
        open.pop();
        node_on_open.at(current_node) = false;
        ++(metadata.expansions);
        if (isGoal(pdapNode::getNode(current_node), goal)){
            metadata.runtime.stop();
            pdap_backtrack_path<pdapNode>(current_node);
            return;
        }
        prior_open_size = open.size();
        pdapGenerateSuccessors(current_node, goal, agent_speed, safe_intervals, map, open, handles, node_on_open, destination_ind, edge_ind);
        metadata.generated += std::max<long>((long)open.size() - prior_open_size, 0);
    }
}





inline void partialPdapGenerateSuccessors(std::size_t cnode, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map,
                                           NodeOpen<partialPdapNode, partialPdapNodeGreater>& open, std::vector<NodeOpen<partialPdapNode, partialPdapNodeGreater>::handle_type>& handles, std::vector<bool>& node_on_open,
                                            std::vector<std::size_t>& destination_ind, std::vector<std::size_t>& edge_ind){
    double dt;
    double min_f = std::numeric_limits<double>::infinity();
    const partialPdapNode& current_node = partialPdapNode::getNode(cnode);
    double delta_prior = current_node.delta;
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
            double delta = delta_prior + dt;
            double pmf = safe_intervals.partialwaits(act, current_node.intervalInd, destination_ind, edge_ind, dt, current_node.expansions);
            double prospective_next_f = std::max(current_node.alpha + delta_prior, pmf) + dt + eightWayDistance(act.destination, goal, agent_speed);
            if (prospective_next_f < min_f){
                min_f = prospective_next_f;
            }
            ++(partialPdapNode::nodes[cnode].expansions);
            for (std::size_t i = 0; i < edge_ind.size(); i++){
                const safe_interval& edge_interval = safe_intervals.get_edge(act, current_node.intervalInd, destination_ind[i], edge_ind[i]);
                double alpha = std::max(current_node.alpha, edge_interval.first - delta_prior);
                act.destination.time = std::max(current_node.s.time + dt, edge_interval.first + dt); 
                if (edge_interval.second < act.destination.time){
                    continue;
                }
                assert(safe_intervals.valid(act, agent_speed));
                double beta = std::min(current_node.beta, edge_interval.second - alpha - delta_prior);
                double f = act.destination.time + eightWayDistance(act.destination, goal, agent_speed);
                std::size_t node_ind = safe_intervals.visited(act, current_node.intervalInd, destination_ind[i], edge_ind[i]);
                std::size_t intervalInd = destination_ind[i];
                if (node_ind != std::numeric_limits<std::size_t>::max()){
                    if (partialPdapNode::getNode(node_ind).s.time > act.destination.time){
                        partialPdapNode::set_arrival(node_ind, act.destination.time, alpha, beta, delta, f, 0,cnode);
                        if (node_on_open.at(node_ind)){
                            open.increase(handles.at(node_ind));
                        }
                        else{
                            handles.at(node_ind) = open.emplace(node_ind);
                            node_on_open.at(node_ind) = true;
                        }
                    }
                    continue;
                }
                auto j = partialPdapNode::newNode(act.destination.x, act.destination.y, intervalInd, act.destination.time, alpha, beta, delta, f, f, 0, cnode);
                handles.emplace_back(open.emplace(j));
                node_on_open.emplace_back(true);
                assert(handles.size() == partialPdapNode::nodes.size());
                safe_intervals.markvisited(act, current_node.intervalInd, destination_ind[i], edge_ind[i], j);
            }
        }
    }
    if (std::isfinite(min_f)){
        partialPdapNode::set_f(cnode, min_f);
        open.emplace(cnode);
    }
}

inline Functional partialPdapAStar(const State& start_state, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map, Metadata& metadata){
    metadata.runtime.start();
    long prior_open_size;
    std::vector<std::size_t> destination_ind;
    std::vector<std::size_t> edge_ind;
    NodeOpen<partialPdapNode,partialPdapNodeGreater> open;
    std::vector<NodeOpen<partialPdapNode, partialPdapNodeGreater>::handle_type> handles;
    std::vector<bool> node_on_open;
    pdapNode::nodes.clear();
    Functional functional;
    double max_query = metadata.args()["maxquery"].as<double>();
    auto start_interval = safe_intervals.get_interval(0.0, map.get_safe_interval_ind(start_state));
    double f = start_state.time + eightWayDistance(start_state, goal, agent_speed);
    auto current_node = partialPdapNode::newNode(start_state.x, start_state.y, start_interval->first, start_state.time, 0, start_interval->second, 0.0, f, f, 0,std::numeric_limits<std::size_t>::max());
    handles.emplace_back(open.emplace(current_node));
    node_on_open.emplace_back(true);
    assert(handles.size() == partialPdapNode::nodes.size());
    while(!open.empty()){
        current_node = open.top();
        auto cn = partialPdapNode::getNode(current_node);
        open.pop();
        node_on_open.at(current_node) = false;
        ++(metadata.expansions);
        if(!functional.check_back(current_node)){
            continue;
        }
        if (isGoal(cn, goal)){
            double alph = functional.emplace_back(cn.alpha, cn.beta, cn.delta, current_node);
            if (alph >= max_query){
                metadata.runtime.stop();
                std::cout << "full functional found\n";
                break;
            }
        }
        prior_open_size = open.size();
        partialPdapGenerateSuccessors(current_node, goal, agent_speed, safe_intervals, map, open, handles, node_on_open, destination_ind, edge_ind);
        metadata.generated += std::max<long>((long)open.size() - prior_open_size, 0);
    }
    if(open.empty()){
        metadata.runtime.stop();
        if (functional.finite_until() > max_query){
            std::cout << "full functional found\n";
        }
        else{
            std::cout << "partial functional found\n";
        }
    }
    return functional.finalize();
}
/*
inline void partialPdapGenerateSuccessors(std::size_t cnode, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map,
                                           boost::heap::priority_queue<std::size_t, boost::heap::compare<partialPdapNodeGreater>>& open, 
                                          std::vector<std::size_t>& destination_ind, std::vector<std::size_t>& edge_ind){
    double dt;
    const partialPdapNode& current_node = partialPdapNode::getNode(cnode);
    double delta_prior = current_node.delta();
    Action act(current_node.s, State(0, 0, 0));
    //auto interval_starts = std::vector<std::pair<double, double>>();
    double min_f = std::numeric_limits<double>::infinity();
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
            std::size_t loc_ind = safe_intervals.waits(map, act, waits_res, waits_res_intervals, edge_ind, current_node.expansions + 2);
            std::size_t i = current_node.expansions;
            double delta_prior = current_node.delta();
            double alpha = std::max(current_node.alpha, waits_res[i]);
            act.destination.time =  current_node.delta() + dt + alpha;
            if (safe_intervals.isSafe(act, map, agent_speed)){ //&& !safe_intervals.markvisited(loc_ind, i)){
                if (waits_res.size() > i){
                    double intervalStart = waits_res_intervals[i].first;
                    double intervalEnd = waits_res_intervals[i].second;
                    double beta = std::min(current_node.beta, intervalEnd - delta_prior);
                    if(beta < alpha){
                        continue;
                    }
                    double f = act.destination.time + eightWayDistance(act.destination, goal, agent_speed);
                    std::size_t node_ind = safe_intervals.visited(loc_ind, edge_ind[i]);
                    if (node_ind != std::numeric_limits<std::size_t>::max()){
                        if (partialPdapNode::getNode(node_ind).s.time > act.destination.time){
                            partialPdapNode::set_arrival(node_ind, act.destination.time, alpha, beta, f, cnode);
                            partialPdapNode::set_f(node_ind, f, 0);
                            open.emplace(node_ind);
                        }
                        continue;
                    }
                    auto j = partialPdapNode::newNode(act.destination.x, act.destination.y, intervalStart, act.destination.time, alpha, beta, f, 0, cnode);
                    open.emplace(j);
                    safe_intervals.markvisited(loc_ind, edge_ind[i], j);
                    if (waits_res.size() > i+1){
                        alpha = std::max(current_node.alpha, waits_res[i+1]);
                        act.destination.time =  current_node.delta() + dt +alpha;
                        f = act.destination.time + eightWayDistance(act.destination, goal, agent_speed);
                        if (f < min_f){
                            min_f = f;
                        }
                    }
                }
            }
            
            
        }
    }
    if (std::isfinite(min_f)){
        partialPdapNode::set_f(cnode, min_f);
        open.emplace(cnode);
    }
}

inline Functional partialPdapAStar(const State& start_state, const State& goal, double agent_speed, SafeIntervals& safe_intervals, const Map& map, Metadata& metadata){
    metadata.runtime.start();
    std::vector<double> waits_res;
    std::vector<safe_interval> waits_res_intervals;
    std::vector<std::size_t> edge_ind;
    boost::heap::priority_queue<std::size_t, boost::heap::compare<partialPdapNodeGreater>> open;
    Functional functional;
    auto start_interval = safe_intervals.get_interval(0.0, map.get_safe_interval_ind(start_state));
    double f = start_state.time + eightWayDistance(start_state, goal, agent_speed);
    auto current_node = partialPdapNode::newNode(start_state.x, start_state.y, start_interval->first, start_state.time, 0.0, start_interval->second, f, 0,std::numeric_limits<std::size_t>::max());
    safe_intervals.markvisited(map.get_safe_interval_ind(start_state), 0, current_node);
    open.emplace(current_node);
    //double lower_bound = 0.0;
    double max_query = metadata.args()["maxquery"].as<double>();
    while(!open.empty()){
        current_node = open.top();
        auto cn = partialPdapNode::getNode(current_node);
        open.pop();
        //if (cn.f < lower_bound){
        //    continue;
        //}
        //lower_bound = cn.f;
        ++(metadata.expansions);
        if (isGoal(cn, goal)){
            pdap_backtrack_path<partialPdapNode>(current_node);
            double alph = functional.emplace_back(cn.alpha, cn.beta, cn.delta(), current_node);
            //std::cout <<"alpha: " << alph << ", second: " << start_interval->second << "\n";
            //debug_open(open);
            if (alph >= max_query){
                std::cout << "full functional found\n";
                break;
            }
        }
        if (functional.check_back(current_node)){
            partialPdapGenerateSuccessors(current_node, goal, agent_speed, safe_intervals, map, open, waits_res, waits_res_intervals, edge_ind);
        }
    }
    metadata.runtime.stop();
    pdap_backtrack_path<partialPdapNode>(functional.domain[0.0].node_ind);
    return functional;
}
*/