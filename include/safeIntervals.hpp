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
#include <boost/container/flat_set.hpp>
#include <cmath>
#include "structs.hpp"
#include "dynamicObstacle.hpp"
#include "map.hpp"




inline void debug_interval(const safe_interval& i){
    std::cout << "<" << i.first << " " << i.second << ">" << "\n"; 
}

inline bool in_safe_interval(const boost::container::flat_set<safe_interval>& si, const std::pair<double,double>& time){
    auto lb = si.upper_bound(time);
    if (lb != si.begin()){
        --lb;
    }
    return time.first >= lb->first && time.second <= lb->second; 
} 

class SafeIntervals{
    private:
        double valid_until;
        double forget_until;
        std::vector<boost::container::flat_set<safe_interval>> _safe_intervals;
        EdgeClosed _visited;
        EdgeIntervals _edge_safe_intervals;
        std::vector<boost::container::flat_set<safe_interval>> unsafe_intervals;
        const std::vector<std::shared_ptr<DynamicObstacle>>& _obs;
        const Map& _map;

        static void join_intervals(std::vector<boost::container::flat_set<safe_interval>>& intervals){
            for (std::size_t i=0;i<intervals.size();i++){
                auto prev = intervals[i].begin();
                if (prev == intervals[i].end()){
                    continue;
                }
                auto cur = std::next(prev);
                while (cur != intervals[i].end()){
                    if (cur->first <= prev->second){
                        prev->second = cur->second;
                        cur = intervals[i].erase(cur);
                        prev = std::prev(cur);
                        continue;
                    }
                    ++prev;
                    ++cur;
                }
            }
        }
        std::vector<safe_interval> combine_intervals(const std::vector<size_t>& inds, std::size_t n, double time, double action_duration) const{
            std::vector<safe_interval> retval;
            safe_interval si;
            double half_duration = 0.5*action_duration;
            auto source_ints = _safe_intervals[inds[0]];
            auto destination_ints = _safe_intervals[inds[1]];
            auto edge_ints = _safe_intervals[inds[2]];
            if (source_ints.empty() || destination_ints.empty() || edge_ints.empty()){
                return retval;
            }
            si.first = time;
            si.second = time;
            auto source = source_ints.lower_bound(si);
            if (source != source_ints.begin()){
                --source;
            }
            auto edge = edge_ints.lower_bound(si);
            if (edge != edge_ints.begin()){
                --edge;
            }
            si.first += half_duration;
            si.second += half_duration;
            auto destination = destination_ints.lower_bound(si);
            if (destination != destination_ints.begin()){
                --destination;
            }
            while(retval.size() < n){
                double comb_start = std::max(source->first, std::max(destination->first - half_duration, edge->first));
                double comb_end = source->second - half_duration;
                int limitation = SOURCE;
                if (edge->second - action_duration <= comb_end){
                    comb_end = edge->second - action_duration;
                    limitation = EDGE;
                }
                if (destination->second - action_duration <= comb_end){
                    comb_end = destination->second - action_duration;
                    limitation = DESTINATION;
                }
                if (limitation == SOURCE){
                    break;
                }
                if (comb_start < comb_end){
                    retval.emplace_back(comb_start, comb_end);
                }
                if (limitation == EDGE){
                    ++edge;
                }
                else{
                    ++destination;
                }
            }
            return retval;
        }

        void generate_from_unsafe(const std::vector<boost::container::flat_set<safe_interval>>& intervals){
            for (std::size_t i=0;i<intervals.size();i++){
                double begin = 0.0;
                if (intervals[i].empty()){
                    _safe_intervals[i].emplace(begin, std::numeric_limits<double>::infinity());
                    continue;
                }
                auto cur = intervals[i].begin(); 
                if (cur->first == 0.0){
                    begin = cur->second;
                    ++cur;
                }
                while(cur != intervals[i].end()){
                    _safe_intervals[i].emplace_hint(_safe_intervals[i].end(), begin, cur->first);
                    begin = cur->second;
                    ++cur; 
                }
                if (std::isfinite(begin)){
                    _safe_intervals[i].emplace_hint(_safe_intervals[i].end(), begin, std::numeric_limits<double>::infinity());
                }
            }
        }
    public:
        SafeIntervals(const std::vector<std::shared_ptr<DynamicObstacle>>& obs, const Map& map, double unsafe_time, double agent_speed, const State& start_state, double startendt):valid_until(0.0),forget_until(0.0),_obs(obs),_map(map){
            std::size_t size = map.size;
            //unsafe_intervals.resize(4*size);
            _safe_intervals.resize(4*size);
            // need enough for all vertexes and neighboring edges.
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

        inline void forget(double before){
            if (before < 2*forget_until){
                return;
            }
            forget_until = 2*before;
            auto grid_size = unsafe_intervals.size();
            for (std::size_t i = 0;i<grid_size;i++){
                auto it = unsafe_intervals[i].begin();
                while(it != unsafe_intervals[i].end() && (it->second < before)){
                    ++it;
                }
                if (it != unsafe_intervals[i].begin()){
                    unsafe_intervals[i].erase(unsafe_intervals[i].begin(), it);
                }
            }
        }

        inline void zero_visits(){
            for (auto elem: _visited){
                for (std::size_t j = 0; j < elem.second.size(); j++){
                    _visited.at(elem.first).at(j) = std::numeric_limits<std::size_t>::max();
                }
            }
        }

        void always_safe_until(const State& s, double until, const Map& map){
            auto ind = map.get_safe_interval_ind(s);
            _safe_intervals[ind] = boost::container::flat_set<safe_interval>();
            _safe_intervals[ind].emplace(0.0, until);
        }
        inline auto get_interval(double time, std::size_t index, bool debug = false)const{
            auto interval = _safe_intervals[index].upper_bound(safe_interval(time, std::numeric_limits<double>::infinity()));
            if(debug){
                std::cout << "get interval " << time << " " << index << "\n";
            }
            if(debug){
                debug_interval(*interval);
            }
            if (interval != _safe_intervals[index].begin()){
                interval = std::prev(interval);
                if(debug){
                    debug_interval(*interval);
                }
            }

            if(debug){
                std::cout << "destination safe intervals\n";
                for(auto inter:_safe_intervals[index]){
                    debug_interval(inter);
                }
                std::cout << "\n";
            }
            
            //assert(interval->first <= time && time <= interval->second);
            return interval;
        }

        inline bool valid(const Action& action, double agent_speed, bool debug = false) const{
            EdgeIntervalIndex eii;
            double action_duration = eightWayDistance(action.source, action.destination, agent_speed);
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            auto source_interval = get_interval(action.source.time, eii.source_loc_ind);
            if (debug){
                std::cout << "action duration: " << action_duration << "\n";
            }
            //check wait
            double wait_until = action.destination.time - action_duration;
            safe_interval act_int(wait_until, std::numeric_limits<double>::infinity());
            if (source_interval->first - action.source.time > epsilon()  || wait_until - source_interval->second > epsilon()){
                if(debug){
                    std::cout << "invalid wait:" << action.source.time << " " << wait_until << "\n";
                    debug_interval(*source_interval);
                    get_interval(action.source.time, eii.source_loc_ind, true);
                }
                return false;
            }
            //check move if non zero duration
            if(action_duration > 0.0){
                eii.source_ind = _safe_intervals.at(eii.source_loc_ind).index_of(source_interval);
                eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination);
                auto destination_interval = get_interval(action.destination.time, eii.destination_loc_ind, debug);
                eii.destination_ind = _safe_intervals.at(eii.destination_loc_ind).index_of(destination_interval);
                const auto& inter = _edge_safe_intervals.at(eii);
                if(debug){
                    debug_interval(act_int);
                }
                auto ub  = inter.upper_bound(act_int);
                if(debug){
                    debug_interval(*ub);
                }
                if (ub != inter.begin()){
                    --ub;
                    if(debug){
                        debug_interval(*ub);
                    }
                }
                bool retval =  (wait_until - ub->first) >= -epsilon()  && (ub->second - wait_until) >= -epsilon();
                if(debug){
                    std::cout << "invalid edge traversal:" << wait_until << " "<< wait_until - ub->first << " " << ub->second - wait_until << "\n";
                    eii.debug();
                    debug_interval(*ub);
                    for(auto interval: inter){
                        debug_interval(interval);
                    }
                }
                return retval;
            }
            return true;
        }

        inline bool valid(double time, double action_duration, const safe_interval& source, const safe_interval& destination, const safe_interval& edge)const{
            return source.first <= time && 
                   source.second >= time &&
                   edge.first <= time &&
                   edge.second >= time &&
                   destination.first - action_duration <= time && 
                   destination.second - action_duration >= time ;
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
                        _unsafe_intervals[ind[0]].emplace_back(action.source.time, action.destination.time);
                    }
                    else{
                        double dt = 0.5*(action.destination.time - action.source.time);
                        _unsafe_intervals[ind[0]].emplace_back(action.source.time, action.source.time + dt);
                        _unsafe_intervals[ind[1]].emplace_back(action.destination.time - dt, action.destination.time);
                        _unsafe_intervals[ind[2]].emplace_back(action.source.time, action.destination.time);
                    }
                }
            }
            for (std::size_t i = 0; i<_safe_intervals.size();i++){
                unsafe_intervals.emplace_back(_unsafe_intervals[i].begin(), _unsafe_intervals[i].end());
                unsafe_intervals.back().emplace(safe_interval(until, std::numeric_limits<double>::infinity()));
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
                                for(auto source_interval = source_intervals.begin(); source_interval != source_intervals.end(); source_interval++){
                                    eii.source_ind = source_intervals.index_of(source_interval);
                                    const auto& destination_intervals = _safe_intervals.at(eii.destination_loc_ind);
                                    for(auto destination_interval = destination_intervals.begin(); destination_interval != destination_intervals.end(); destination_interval++){
                                        eii.destination_ind = destination_intervals.index_of(destination_interval);
                                        const auto& edge_intervals = _safe_intervals.at(ind[2]);
                                        for (auto edge_interval = edge_intervals.begin(); edge_interval != edge_intervals.end(); edge_interval++ ){
                                            double alpha = std::max(source_interval->first, std::max(edge_interval->first, destination_interval->first - action_duration));
                                            double beta = std::min(source_interval->second, std::min(edge_interval->second, destination_interval->second - action_duration));
                                            if(alpha <= beta){
                                                if (!_edge_safe_intervals.contains(eii)){
                                                   _edge_safe_intervals.emplace(std::make_pair(eii, boost::container::flat_set<safe_interval>()));
                                                   _visited.emplace(std::make_pair(eii, std::vector<std::size_t>()));
                                                }
                                                _edge_safe_intervals.at(eii).emplace(alpha, beta);
                                                _visited.at(eii).emplace_back(std::numeric_limits<std::size_t>::max());
                                            }
                                        }
                                    }
                                }
                            }  
                        }
                    }
                }
            }
            _unsafe_intervals.clear();
            //std::cout << _safe_intervals[0].size() << "\n";
            //zero_visits();
            valid_until = until;
        }
/*
        inline bool isSafe(const Action& action, const Map& map, double agent_speed, bool debug = false) const{
            safe_interval time = safe_interval();
            std::array<std::size_t, 3> ind;
            ind.fill(std::numeric_limits<std::size_t>::max());
            if (debug){
                action.debug();
            }
            if (map.isSafe(action.source.x, action.source.y) && map.isSafe(action.destination.x, action.destination.y)){
                if (action.source.x != action.destination.x && action.source.y != action.destination.y){
                    if (!map.isSafe(action.source.x, action.destination.y)){
                        if (debug){
                            std::cout << "diag1\n";
                        }
                        return false;
                    }
                    if (!map.isSafe(action.destination.x, action.source.y)){
                        if (debug){
                            std::cout << "diag2\n";
                        }
                        return false;
                    }    
                }
                //while (valid_until < action.destination.time){
                //    generate(2*valid_until);
                //}
                
                map.get_safe_interval_ind(action, ind);
                if (debug){
                    std::cout << ind[1] << "\n";
                }
                if (ind[1] == std::numeric_limits<std::size_t>::max()){
                    time.first = action.source.time;
                    time.second = action.destination.time;
                    return in_safe_interval(_safe_intervals[ind[0]], time);
                }
                else{
                    
                    double delta = eightWayDistance(action.source, action.destination, agent_speed);
                    double dt = 0.5*(delta);
                    double wait = action.destination.time - action.source.time - delta;
                    time.first = action.source.time;
                    time.second = action.source.time + wait + dt;
                    if (debug){
                            std::cout << "source at " << time.first << " " << time.second << "\n";
                            for (auto inter:_safe_intervals[ind[0]] ){
                                debug_interval(inter);
                            }
                        }
                    if(!in_safe_interval(_safe_intervals[ind[0]], time)){
                        return false;
                    }
                    time.first = action.destination.time - dt;
                    time.second = action.destination.time;
                    if (debug){
                            std::cout << "destination at " << time.first << "\n";
                            for (auto inter:_safe_intervals[ind[1]] ){
                                debug_interval(inter);
                            }
                        }
                    if(!in_safe_interval(_safe_intervals[ind[1]], time)){
                        
                        return false;
                    }
                    time.first = action.source.time + wait;
                    time.second = action.destination.time;
                     if (debug){
                            std::cout << "edge at " << time.first << "\n";
                            for (auto inter:_safe_intervals[ind[2]] ){
                                debug_interval(inter);
                            }
                        }
                    if(!in_safe_interval(_safe_intervals[ind[2]], time)){
                       
                        return false;
                    }
                    return true;
                }
            }
            return false;
        }
*/
        inline std::pair<std::vector<int>, std::vector<double>> flatten(const Map& map) const{
            std::vector<int> retval_int;
            std::vector<double> retval_double;
            for (std::size_t i = 0; i < _safe_intervals.size();i++){
                std::size_t loci = i / 4; 
                int move = i % 4;
                int y = loci / map.width;
                int x = loci % map.width;
                for (auto interval: _safe_intervals[i]){
                    retval_int.emplace_back(x);
                    retval_int.emplace_back(y);
                    retval_int.emplace_back(move);
                    retval_double.emplace_back(interval.first);
                    retval_double.emplace_back(interval.second);
                }
            }
            return std::pair<std::vector<int>, std::vector<double>>(retval_int, retval_double);
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
            return _visited.at(eii).at(edge_i);
        }

        inline void markvisited(const Action & action, std::size_t source_interval_i, std::size_t destination_interval_i, std::size_t edge_i, std::size_t node_ind){
            EdgeIntervalIndex eii;
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            eii.source_ind = source_interval_i;
            eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination);
            eii.destination_ind = destination_interval_i;
            _visited.at(eii).at(edge_i) = node_ind;
        }

        inline safe_interval get_edge(const Action & action, std::size_t source_interval_i, std::size_t destination_interval_i, std::size_t edge_i) const{
            EdgeIntervalIndex eii;
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            eii.source_ind = source_interval_i;
            eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination);
            eii.destination_ind = destination_interval_i;
            return *_edge_safe_intervals.at(eii).nth(edge_i);
        }

        inline void waits(const Action& action, std::size_t source_interval_ind, std::vector<std::size_t>& destination_interval_inds, std::vector<std::size_t>& edge_inds, double action_duration, bool debug = false) const{
            destination_interval_inds.clear();
            edge_inds.clear();
            EdgeIntervalIndex eii;
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            eii.source_ind = source_interval_ind;
            eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination);
            
            double wait_until = action.destination.time - action_duration;
            std::pair<double, double> time(wait_until, std::numeric_limits<double>::infinity());
            if (debug){
                std::cout << "debug waits\n";
            }
            std::size_t total_destination_intervals = _safe_intervals.at(eii.destination_loc_ind).size();
            auto first_destination = _safe_intervals.at(eii.destination_loc_ind).upper_bound(time);
            if (_safe_intervals.at(eii.destination_loc_ind).begin() != first_destination){
                --first_destination;
            }
            for (eii.destination_ind = _safe_intervals.at(eii.destination_loc_ind).index_of(first_destination); eii.destination_ind < total_destination_intervals; eii.destination_ind++){
                if (_edge_safe_intervals.contains(eii)){
                    const auto & si = _edge_safe_intervals.at(eii);
                    auto lb = si.upper_bound(time);
                    if (lb != si.begin()){
                        --lb;
                    }
                    
                    while(lb != si.end()){
                        if (action.source.time >= lb->first && action.source.time <= lb->second){
                            if (debug){
                                std::cout << action.source.time << "\n";
                                eii.debug();
                                debug_interval(*lb);
                                std::cout << "\n";
                            }
                            destination_interval_inds.emplace_back(eii.destination_ind);
                            edge_inds.emplace_back(si.index_of(lb));
                        }
                        ++lb;
                    }
                }
            }
        }

        inline double partialwaits(const Action& action, std::size_t source_interval_ind, std::vector<std::size_t>& destination_interval_inds, std::vector<std::size_t>& edge_inds, double action_duration, std::size_t partial_i, bool debug = false) const{
            double min_arrival_time = std::numeric_limits<double>::infinity();
            destination_interval_inds.clear();
            edge_inds.clear();
            EdgeIntervalIndex eii;
            eii.source_loc_ind = _map.get_safe_interval_ind(action.source);
            eii.source_ind = source_interval_ind;
            eii.destination_loc_ind = _map.get_safe_interval_ind(action.destination);
            
            double wait_until = action.destination.time - action_duration;
            std::pair<double, double> time(wait_until, std::numeric_limits<double>::infinity());
            if (debug){
                std::cout << "debug waits\n";
            }
            std::size_t total_destination_intervals = _safe_intervals.at(eii.destination_loc_ind).size();
            auto first_destination = _safe_intervals.at(eii.destination_loc_ind).upper_bound(time);
            if (_safe_intervals.at(eii.destination_loc_ind).begin() != first_destination){
                --first_destination;
            }
            for (eii.destination_ind = _safe_intervals.at(eii.destination_loc_ind).index_of(first_destination); eii.destination_ind < total_destination_intervals; eii.destination_ind++){
                if (_edge_safe_intervals.contains(eii)){
                    const auto & si = _edge_safe_intervals.at(eii);
                    if (si.size() > partial_i){
                        auto lb = si.nth(partial_i);
                        if (action.source.time >= lb->first && action.source.time <= lb->second){
                            if (debug){
                                std::cout << action.source.time << "\n";
                                eii.debug();
                                debug_interval(*lb);
                                std::cout << "\n";
                            }
                            destination_interval_inds.emplace_back(eii.destination_ind);
                            edge_inds.emplace_back(si.index_of(lb));
                        }
                        ++lb;
                        if (lb != si.end() && lb->first < min_arrival_time){
                            min_arrival_time = lb->first;
                        }
                    }
                    
                   
                }
            }
            return min_arrival_time;
        }
        /*
        inline void waits(const Map& map, const Action& action, std::vector<std::pair<double, std::pair<double,double>>>& res) const{
            double t = action.source.time;
            double action_duration = action.destination.time - t;
            double wait = 0;
            std::array<std::size_t, 3> inds;
            if (!map.isSafe(action.source.x, action.source.y) || !map.isSafe(action.destination.x, action.destination.y)){
                return;
            }
            if (action.source.x != action.destination.x && action.source.y != action.destination.y){
                if (map.inBounds(action.source.x, action.destination.y) && !map.isSafe(action.source.x, action.destination.y)){
                    return;
                }
                if (map.inBounds(action.destination.x, action.source.y) && !map.isSafe(action.destination.x, action.source.y)){
                    return;
                }    
            }
            map.get_safe_interval_ind(action, inds);
            auto source_interval = get_interval(t, inds[0]);
            while(t + wait + 0.5*action_duration <= source_interval->second){
                auto destination_interval = get_interval(t, inds[1]);
                auto edge_interval = get_interval(t, inds[2]);
                if(valid(t+wait, action_duration, *source_interval, *destination_interval, *edge_interval)){
                    res.emplace_back(t+wait, *destination_interval);
                }
                destination_interval = std::next(destination_interval);
                edge_interval = std::next(edge_interval);
                if (destination_interval == _safe_intervals[inds[1]].end() && edge_interval == _safe_intervals[inds[2]].end()){
                    break;
                }
                else if(destination_interval == _safe_intervals[inds[1]].end()){
                    wait = std::nextafter(edge_interval->first - t, std::numeric_limits<double>::infinity());
                }
                else if(edge_interval == _safe_intervals[inds[2]].end()){
                    wait = std::nextafter(destination_interval->first + 0.5 * action_duration - t, std::numeric_limits<double>::infinity());
                }
                wait = std::min(std::nextafter(destination_interval->first + 0.5 * action_duration - t, std::numeric_limits<double>::infinity()),
                        std::nextafter(edge_interval->first - t, std::numeric_limits<double>::infinity()));
            }
        }
        
        static void debug(const Map& map, const std::vector<boost::container::flat_set<safe_interval>>& intervals){
            for (int i=0; i < intervals.size(); i++){
                map.ind2loc(i).debug();
                for (auto interval: intervals[i]){
                    DEBUG_MSG_NO_LINE_BREAK(interval.first);
                    DEBUG_MSG_NO_LINE_BREAK(" ");
                    DEBUG_MSG_NO_LINE_BREAK(interval.second);
                    DEBUG_MSG_NO_LINE_BREAK(", ");
                    (void)interval;
                }
                DEBUG_MSG("");
            }
        }

        void debug(const Map& map) const{
            DEBUG_MSG("Safe Intervals");
            debug(map, _safe_intervals);
            DEBUG_MSG("END safe intervals");
        }
        */
};