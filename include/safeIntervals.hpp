//put all safeinterval related stuff here
#pragma once

#include <memory>
#define SOURCE 0
#define DESTINATION 1
#define EDGE 2

#include <cstddef>
#include <ios>
#include <utility>
#include <vector>
#include <boost/container/flat_set.hpp>
#include <cmath>
#include "structs.hpp"
#include "dynamicObstacle.hpp"
#include "map.hpp"



typedef std::pair<double, double> safe_interval;

inline void debug_interval(const safe_interval& i){
    std::cout << "<" << i.first << " " << i.second << ">" << "\n"; 
}

inline bool in_safe_interval(const boost::container::flat_set<safe_interval>& si, const std::pair<double,double>& time){
    auto lb = si.lower_bound(time);
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
        SafeIntervals(const std::vector<std::shared_ptr<DynamicObstacle>>& obs, const Map& map):valid_until(0.0),forget_until(0.0),_obs(obs),_map(map){
            std::size_t size = map.size;
            unsafe_intervals.resize(4*size);
            _safe_intervals.resize(4*size);
            // need enough for all vertexes and neighboring edges.
            generate(1000.0);
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

        void generate(double until){
            if (valid_until >= until){
                return;
            }
            for (auto obstacle: _obs){
                auto path = obstacle->path(valid_until, until);
                for (std::size_t i = 1; i<path.size(); i++){
                    auto action = Action(path[i-1], path[i]);

                    std::vector<std::size_t> ind = _map.get_safe_interval_ind(action);
                    if (ind.size() == 1){
                        unsafe_intervals[ind[0]].emplace(action.source.time, action.destination.time);
                    }
                    else{
                        double dt = 0.5*(action.destination.time - action.source.time);
                        unsafe_intervals[ind[0]].emplace(action.source.time, action.source.time + dt);
                        unsafe_intervals[ind[1]].emplace(action.destination.time - dt, action.destination.time);
                        unsafe_intervals[ind[2]].emplace(action.source.time, action.destination.time);
                    }
                }
            }
            join_intervals(unsafe_intervals);
            generate_from_unsafe(unsafe_intervals);
            valid_until = until;
        }

        bool isSafe(const Action& action, const Map& map){
            safe_interval time = safe_interval();
            if (map.isSafe(action.source.x, action.source.y) && map.isSafe(action.destination.x, action.destination.y)){
                if (action.source.x != action.destination.x && action.source.y != action.destination.y){
                    if (!map.isSafe(action.source.x, action.destination.y)){
                        return false;
                    }
                    if (!map.isSafe(action.destination.x, action.source.y)){
                        return false;
                    }    
                }
                while (valid_until < action.destination.time){
                    generate(2*valid_until);
                }
                auto ind = map.get_safe_interval_ind(action);
                if (ind.size() == 1){
                    time.first = action.source.time;
                    time.second = action.destination.time;
                    return in_safe_interval(_safe_intervals[ind[0]], time);
                    }
                else{
                    double dt = 0.5*(action.destination.time - action.source.time);
                    time.first = action.source.time;
                    time.second = action.source.time + dt;
                    if(!in_safe_interval(_safe_intervals[ind[0]], time)){
                        return false;
                    }
                    time.first = action.destination.time - dt;
                    time.second = action.destination.time;
                    if(!in_safe_interval(_safe_intervals[ind[1]], time)){
                        return false;
                    }
                    time.first = action.source.time;
                    time.second = action.destination.time;
                    if(!in_safe_interval(_safe_intervals[ind[2]], time)){
                        return false;
                    }
                    return true;
                }
            }
            return false;
        }

        std::vector<double> start_waits(const Action& action, const Map& map, size_t n) const{
            std::vector<double> interval_waits;
            if (map.isSafe(action.source.x, action.source.y) && map.isSafe(action.destination.x, action.destination.y)){
                auto ind = map.get_safe_interval_ind(action);
                if (ind.size() == 3){
                    auto combined_intervals = combine_intervals(ind, n, action.source.time, action.destination.time - action.source.time);
                    for (auto i: combined_intervals){
                        interval_waits.emplace_back(i.first);
                    }
                }
                else{
                    //DEBUG_MSG("Incorrect number of safe intervals.");
                }
            }
            return interval_waits;
        }

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

        inline auto get_interval(double time, std::size_t index)const{
            auto interval = _safe_intervals[index].upper_bound(safe_interval(time, std::numeric_limits<double>::infinity()));
            interval = std::prev(interval);
            assert(interval->first <= time && time <= interval->second);
            return interval;
        }

        inline bool valid(double time, double action_duration, const safe_interval& source, const safe_interval& destination, const safe_interval& edge)const{
            double dt = 0.5*action_duration;
            return source.first <= time && source.second >= time + dt &&
                   edge.first <= time && edge.second >= time + action_duration &&
                   destination.first <= time + dt && destination.second >= time + action_duration;
        }

        inline void waits(const Map& map, const Action& action, std::vector<std::pair<double, double>>& res){
            double t = action.source.time;
            double action_duration = action.destination.time - t;
            double wait = 0;

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
            auto inds = map.get_safe_interval_ind(action);
            auto source_interval = get_interval(t, inds[0]);
            while(t + wait + 0.5*action_duration <= source_interval->second){
                auto destination_interval = get_interval(t, inds[1]);
                auto edge_interval = get_interval(t, inds[2]);
                if(valid(t+wait, action_duration, *source_interval, *destination_interval, *edge_interval)){
                    res.emplace_back(t+wait, destination_interval->first);
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
        inline void waits(const Map& map, const Action& action, std::vector<std::pair<double, std::pair<double,double>>>& res){
            double t = action.source.time;
            double action_duration = action.destination.time - t;
            double wait = 0;

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
            auto inds = map.get_safe_interval_ind(action);
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
        /*
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