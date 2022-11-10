#pragma once

#include "node.hpp"
#include "safeIntervals.hpp"

template <typename NodeT>
void generateSuccessors(const NodeT& current_node, SafeIntervals& safe_intervals, const Map& map, double agent_speed, std::vector<NodeT>& succesors) const{
    double dt;
    succesors.clear();
    State s(0, 0, 0.0);
    for (int m_x = -1; m_x <=1; m_x++){
        s.x = current_node.x + mx;
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
            s.y = current_node.y + m_y;
            s.time = current_node.time + dt;
            auto waits = safe_intervals.start_waits(Action(current_node.state(), s), map, _n);
            for (auto wait: waits){
                next.dynamic_g += wait;
                if (safe_intervals.isSafe(Action(cur_state, next.toState()), map) && !expansion.in_closed(next)){
                    expansion.add_open(next, cur_node.index);
                }
                next.dynamic_g -= wait;
            }
        }
    }
}