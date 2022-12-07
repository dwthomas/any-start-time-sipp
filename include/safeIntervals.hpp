//put all safeinterval related stuff here
#pragma once


#include "heuristic.hpp"
#define SOURCE 0
#define DESTINATION 1
#define EDGE 2
#include <limits>
#include <memory>
#include <cstddef>
#include <ios>
#include <utility>
#include <vector>
#include <unordered_map>
#include <boost/container/flat_map.hpp>
#include <cmath>
#include "structs.hpp"
#include "dynamicObstacle.hpp"
#include "map.hpp"


inline void debug_interval(const safe_interval& i){
    std::cout << "<" << i.first << " " << i.second << ">" << "\n"; 
}

inline double intervalBegin(const safe_interval& si){
    return si.second;
}

inline double intervalEnd(const safe_interval& si){
    return si.first;
}

template<typename iteratorType>
inline double& intervalLvalueBegin(iteratorType it){
    return it->second;
}

template<typename iteratorType>
inline double& intervalLvalueEnd(iteratorType it){
    return it->first;
}

template<typename iteratorType>
inline double intervalBegin(iteratorType it){
    return it->second;
}

template<typename iteratorType>
inline double intervalEnd(iteratorType it){
    return it->first;
}



inline auto pick_safe_interval(const SafeIntervalContainer& si, double time){
    return si.lower_bound(time);

}

inline bool in_safe_interval(const SafeIntervalContainer& si, double time){
    auto lb = pick_safe_interval(si, time);
    return lb != si.end() && intervalBegin(lb) <= time && time <= intervalEnd(lb);
} 


inline bool valid(double time, double action_duration, const safe_interval& source, const safe_interval& destination, const safe_interval& edge){
    return intervalBegin(source) <= time && 
            intervalEnd(source) >= time &&
            intervalBegin(edge) <= time &&
            intervalEnd(edge) >= time &&
            intervalBegin(destination) - action_duration <= time && 
            intervalEnd(destination) - action_duration >= time ;
}

class SafeIntervals{
    private:
        double valid_until;
        std::vector<SafeIntervalContainer> _safe_intervals;
        EdgeClosed _visited;
        EdgeIntervals _edge_safe_intervals;
        std::vector<SafeIntervalContainer> unsafe_intervals;
        const std::vector<std::shared_ptr<DynamicObstacle>>& _obs;
        const Map& _map;

        static void join_intervals(std::vector<SafeIntervalContainer>& intervals){
            for (std::size_t i=0;i<intervals.size();i++){
                auto prev = intervals[i].begin();
                if (prev == intervals[i].end()){
                    continue;
                }
                auto cur = std::next(prev);
                while (cur != intervals[i].end()){
                    if (intervalLvalueBegin(cur) <= intervalEnd(prev)){
                        intervalLvalueEnd(prev) = intervalEnd(cur);
                        cur = intervals[i].erase(cur);
                        prev = std::prev(cur);
                        continue;
                    }
                    ++prev;
                    ++cur;
                }
            }
        }

        void generate_from_unsafe(const std::vector<SafeIntervalContainer>& intervals){
            for (std::size_t i=0;i<intervals.size();i++){
                double begin = 0.0;
                if (intervals[i].empty()){
                    _safe_intervals[i].emplace(std::numeric_limits<double>::infinity(), begin);
                    continue;
                }
                auto cur = intervals[i].begin(); 
                if (intervalBegin(cur) == 0.0){
                    begin = intervalEnd(cur);
                    ++cur;
                }
                while(cur != intervals[i].end()){
                    _safe_intervals[i].emplace_hint(_safe_intervals[i].end(), intervalBegin(cur), begin);
                    begin = cur->second;
                    ++cur; 
                }
                if (std::isfinite(begin)){
                    _safe_intervals[i].emplace_hint(_safe_intervals[i].end(), std::numeric_limits<double>::infinity(), begin);
                }
            }
        }
    public:
        SafeIntervals(const std::vector<std::shared_ptr<DynamicObstacle>>& obs, const Map& map, double unsafe_time, double agent_speed, const State& start_state, double startendt):valid_until(0.0),_obs(obs),_map(map){
            std::size_t size = map.size;
            //unsafe_intervals.resize(4*size);
            _safe_intervals.resize(4*size);
            generate(unsafe_time, agent_speed, start_state, startendt);
        }

        void debug(const Map& map) const{
            for (uint j = 0; j < map.height; j++){
                for (uint i = 0; i < map.width; i++){
                    std::cout << "<" <<_safe_intervals[4*map.getIndex(i, j)].begin()->first << "," << _safe_intervals[4*map.getIndex(i, j)].begin()->second << ">";
                }
                std::cout << "\n";
            }
        }

        inline void zero_visits(){
            for (auto elem: _visited){
                for (std::size_t j = 0; j < elem.second.size(); j++){
                    _visited[elem.first][j] = std::numeric_limits<std::size_t>::max();
                }
            }
        }

        void always_safe_until(const State& s, double until, const Map& map){
            auto ind = map.get_safe_interval_ind(s);
            _safe_intervals[ind] = SafeIntervalContainer();
            _safe_intervals[ind].emplace(until, 0.0);
        }

        inline auto get_interval(double time, std::size_t index) const{
            return pick_safe_interval(_safe_intervals[index], time);
        }

        inline bool valid(const Action& action, double agent_speed) const{
            EdgeIntervalIndex eii;
            double action_duration = eightWayDistance(action.source, action.destination, agent_speed);
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            auto source_interval = get_interval(action.source.time, eii.source_loc_ind);
            double wait_until = action.destination.time - action_duration;
            safe_interval act_int(wait_until, std::numeric_limits<double>::infinity());
            if (intervalBegin(source_interval) > action.source.time  || wait_until > intervalEnd(source_interval)){
                return false;
            }
            //check move if non zero duration
            if(action_duration > 0.0){
                eii.source_ind = _safe_intervals.at(eii.source_loc_ind).index_of(source_interval);
                eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination);
                auto destination_interval = get_interval(action.destination.time, eii.destination_loc_ind);
                eii.destination_ind = _safe_intervals[eii.destination_loc_ind].index_of(destination_interval);
                if (!_edge_safe_intervals.contains(eii)){
                    return false;
                }
                const auto& inter = _edge_safe_intervals.at(eii);
                auto ub = pick_safe_interval(inter, wait_until);
                return (wait_until >= intervalBegin(ub))  && (intervalEnd(ub) >= wait_until);
            }
            return true;
        }



        void generate(double until, double agent_speed, const State& start_state, double startendt){
            if (valid_until >= until){
                return;
            }
            std::array<std::size_t,3> ind;
            std::vector<std::vector<safe_interval>> _unsafe_intervals;
            _unsafe_intervals.reserve(_safe_intervals.size());
            for (std::size_t i = 0; i<_safe_intervals.size();i++){
                _unsafe_intervals.emplace_back(std::vector<safe_interval>());
            }
            for (auto obstacle: _obs){
                auto path = obstacle->path(valid_until, until);
                for (std::size_t i = 1; i<path.size(); i++){
                    auto action = Action(path[i-1], path[i]);
                    _map.get_safe_interval_ind(action, ind);
                    if (ind[1] == std::numeric_limits<std::size_t>::max()){
                        _unsafe_intervals[ind[0]].emplace_back(action.destination.time, action.source.time);
                    }
                    else{
                        double dt = 0.5*(action.destination.time - action.source.time);
                        _unsafe_intervals[ind[0]].emplace_back(action.source.time + dt, action.source.time);
                        _unsafe_intervals[ind[1]].emplace_back(action.destination.time, action.destination.time - dt);
                        _unsafe_intervals[ind[2]].emplace_back(action.destination.time, action.source.time);
                    }
                }
            }
            for (std::size_t i = 0; i<_safe_intervals.size();i++){
                unsafe_intervals.emplace_back(_unsafe_intervals[i].begin(), _unsafe_intervals[i].end());
                unsafe_intervals.back().emplace(safe_interval(std::numeric_limits<double>::infinity(), until));
            }
            join_intervals(unsafe_intervals);
            generate_from_unsafe(unsafe_intervals);
            always_safe_until(start_state, startendt, _map);
            EdgeIntervalIndex eii;
            Action act(State(0,0,0),State(0,0,0));
            for (act.source.x = 0; act.source.x < (int)_map.width; act.source.x++){
                for (act.source.y = 0; act.source.y < (int)_map.height; act.source.y++){
                    eii.source_loc_ind = _map.get_safe_interval_ind(act.source);
                    for (int dx = -1; dx <= 1; dx++){
                        act.destination.x = act.source.x + dx;
                        for (int dy = -1; dy <= 1; dy++){
                            act.destination.y = act.source.y + dy;
                            if (!_map.inBounds(act.source.x, act.source.y) || !_map.isSafe(act.source.x, act.source.y) || !_map.inBounds(act.destination.x, act.destination.y) || !_map.isSafe(act.destination.x, act.destination.y)){
                                continue;
                            }
                            if (act.source.x != act.destination.x && act.source.y != act.destination.y){
                                if (_map.inBounds(act.source.x, act.destination.y) && !_map.isSafe(act.source.x, act.destination.y)){
                                    continue;
                                }
                                if (_map.inBounds(act.destination.x, act.source.y) && !_map.isSafe(act.destination.x, act.source.y)){
                                    continue;
                                }    
                            }
                            _map.get_safe_interval_ind(act, ind);
                            assert(eii.source_loc_ind == ind[0]);
                            eii.destination_loc_ind = ind[1];
                            const auto& source_intervals = _safe_intervals.at(ind[0]);
                            if (ind[1] == std::numeric_limits<std::size_t>::max()){
                                continue;
                            }
                            else{       
                                double action_duration = eightWayDistance(act.source, act.destination, agent_speed);  
                                double dt = 0.5*action_duration;                       
                                for(auto source_interval = source_intervals.begin(); source_interval != source_intervals.end(); source_interval++){
                                    eii.source_ind = source_intervals.index_of(source_interval);
                                    const auto& destination_intervals = _safe_intervals[eii.destination_loc_ind];
                                    for(auto destination_interval = destination_intervals.begin(); destination_interval != destination_intervals.end(); destination_interval++){
                                        eii.destination_ind = destination_intervals.index_of(destination_interval);
                                        const auto& edge_intervals = _safe_intervals[ind[2]];
                                        for (auto edge_interval = edge_intervals.begin(); edge_interval != edge_intervals.end(); edge_interval++ ){
                                            double alpha = std::max(intervalBegin(source_interval), std::max(intervalBegin(edge_interval), intervalBegin(destination_interval) - dt));
                                            double beta = std::min(intervalEnd(source_interval) - dt, std::min(intervalEnd(edge_interval) - action_duration, intervalEnd(destination_interval) - action_duration));
                                            if(alpha <= beta){
                                                if (!_edge_safe_intervals.contains(eii)){
                                                   _edge_safe_intervals.emplace(std::make_pair(eii, SafeIntervalContainer()));
                                                   _visited.emplace(std::make_pair(eii, std::vector<std::size_t>()));
                                                }
                                                _edge_safe_intervals[eii].emplace(beta, alpha);
                                                _visited[eii].emplace_back(std::numeric_limits<std::size_t>::max());
                                            }
                                        }
                                    }
                                }
                            }  
                        }
                    }
                }
            }
            valid_until = until;
        }

        inline auto get_intervals(std::size_t loc_ind, std::size_t interval_i) const{
            return _safe_intervals[loc_ind].nth(interval_i);
        }

        inline std::size_t visited(const Action & action, std::size_t source_interval_i, std::size_t destination_interval_i, std::size_t edge_i) const{
            EdgeIntervalIndex eii;
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            eii.source_ind = source_interval_i;
            eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination);
            eii.destination_ind = destination_interval_i;
            return _visited.at(eii)[edge_i];
        }

        inline void markvisited(const Action & action, std::size_t source_interval_i, std::size_t destination_interval_i, std::size_t edge_i, std::size_t node_ind){
            EdgeIntervalIndex eii;
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            eii.source_ind = source_interval_i;
            eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination);
            eii.destination_ind = destination_interval_i;
            _visited[eii][edge_i] = node_ind;
        }

        inline safe_interval get_edge(const Action & action, std::size_t source_interval_i, std::size_t destination_interval_i, std::size_t edge_i) const{
            EdgeIntervalIndex eii;
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            eii.source_ind = source_interval_i;
            eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination);
            eii.destination_ind = destination_interval_i;
            return *_edge_safe_intervals.at(eii).nth(edge_i);
        }

        inline void waits(const Action& action, std::size_t source_interval_ind, std::vector<std::size_t>& destination_interval_inds, std::vector<std::size_t>& edge_inds, double action_duration) const{
            destination_interval_inds.clear();
            edge_inds.clear();
            EdgeIntervalIndex eii;
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            eii.source_ind = source_interval_ind;
            eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination); 
            double wait_until = action.destination.time - action_duration;
            const auto& source_interval = get_intervals(eii.source_loc_ind, eii.source_ind);
            std::size_t total_destination_intervals = _safe_intervals.at(eii.destination_loc_ind).size();
            auto first_destination = get_interval(wait_until, eii.destination_loc_ind);
            
            for (eii.destination_ind = _safe_intervals[eii.destination_loc_ind].index_of(first_destination); eii.destination_ind < total_destination_intervals; eii.destination_ind++){
                if (_edge_safe_intervals.contains(eii)){
                    const auto & si = _edge_safe_intervals.at(eii);
                    auto lb = pick_safe_interval(si, wait_until);
                    while(lb != si.end()){
                        if (intervalBegin(lb) <= intervalEnd(source_interval) && action.source.time <= intervalEnd(lb)){
                            destination_interval_inds.emplace_back(eii.destination_ind);
                            edge_inds.emplace_back(si.index_of(lb));
                        }
                        ++lb;
                    }
                }
            }
        }

        inline double partialwaits(const Action& action, std::size_t source_interval_ind, std::vector<std::size_t>& destination_interval_inds, std::vector<std::size_t>& edge_inds, double action_duration, std::size_t partial_i) const{
            double min_arrival_time = std::numeric_limits<double>::infinity();
            destination_interval_inds.clear();
            edge_inds.clear();
            EdgeIntervalIndex eii;
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            eii.source_ind = source_interval_ind;
            eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination);
            
            double wait_until = action.destination.time - action_duration;

            const auto& source_interval = get_intervals(eii.source_loc_ind, eii.source_ind);
            std::size_t total_destination_intervals = _safe_intervals.at(eii.destination_loc_ind).size();
            auto first_destination = get_interval(wait_until, eii.destination_loc_ind);

            for (eii.destination_ind = _safe_intervals.at(eii.destination_loc_ind).index_of(first_destination); eii.destination_ind < total_destination_intervals; eii.destination_ind++){
                if (_edge_safe_intervals.contains(eii)){
                    const auto & si = _edge_safe_intervals.at(eii);
                    if (si.size() > partial_i){
                        auto lb = si.nth(partial_i);
                        if (intervalBegin(lb) <= intervalEnd(source_interval) && action.source.time <= intervalEnd(lb)){
                            destination_interval_inds.emplace_back(eii.destination_ind);
                            edge_inds.emplace_back(si.index_of(lb));
                        }
                        ++lb;
                        if (lb != si.end() && intervalBegin(lb) < min_arrival_time){
                            min_arrival_time = intervalBegin(lb);
                        }
                    }       
                }
            }
            return min_arrival_time;
        }
};