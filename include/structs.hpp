#pragma once

#include <iostream>
#include <limits>
#include <vector>
#include <boost/container/flat_map.hpp>
#include <boost/functional/hash.hpp>

typedef std::pair<double, double> safe_interval;

struct State{
    int x;
    int y;
    double time;
    State(int _x, int _y, double _t):x(_x),y(_y),time(_t){}
    constexpr bool operator ==(const State & s) const{
        return x == s.x && y == s.y && time == s.time;
    }
    inline void debug() const{
        std::cout << x << " " << y << " " << time << "\n";
    }
};

struct Action{
    State source;
    State destination;
    Action(State s, State d):source(s),destination(d){}
    inline bool operator ==(const Action & a) const{
        return source == a.source && destination == a.destination;
    }

    inline void debug() const{
        std::cout << source.x << " " << source.y << " " << source.time << " -> ";
        std::cout << destination.x << " " << destination.y << " " << destination.time << "\n";
    }
};

struct EdgeIntervalIndex{
    std::size_t source_loc_ind;
    std::size_t source_ind;
    std::size_t destination_loc_ind;
    std::size_t destination_ind;

    inline void debug() const{
        std::cout << source_loc_ind << " " << source_ind << " " << destination_loc_ind <<  " " << destination_ind << "\n";
    }
};

struct EdgeIntervalIndexHash{
    inline std::size_t operator()(const EdgeIntervalIndex& eii) const{
        std::size_t seed = 0;
        boost::hash_combine(seed, eii.source_loc_ind);
        boost::hash_combine(seed, eii.source_ind);
        boost::hash_combine(seed, eii.destination_loc_ind);
        boost::hash_combine(seed, eii.destination_ind);
        return seed;
    }
};

struct EdgeIntervalIndexEquals{
    inline bool operator()(const EdgeIntervalIndex& lhs, const EdgeIntervalIndex& rhs) const{
        return  lhs.source_loc_ind == rhs.source_loc_ind &&
                lhs.destination_loc_ind == rhs.destination_loc_ind &&
                lhs.source_ind == rhs.source_ind &&
                lhs.destination_ind == rhs.destination_ind;
    }
};


struct EdgeIntervalIndexClosedHash{
    inline std::size_t operator()(const EdgeIntervalIndex& eii) const{
        std::size_t seed = 0;
        boost::hash_combine(seed, eii.destination_loc_ind);
        boost::hash_combine(seed, eii.destination_ind);
        return seed;
    }
};

struct EdgeIntervalIndexClosedEquals{
    inline bool operator()(const EdgeIntervalIndex& lhs, const EdgeIntervalIndex& rhs) const{
        return  lhs.destination_loc_ind == rhs.destination_loc_ind &&
                lhs.destination_ind == rhs.destination_ind;
    }
};

using EdgeIntervals = std::unordered_map<EdgeIntervalIndex, boost::container::flat_set<safe_interval>, EdgeIntervalIndexHash, EdgeIntervalIndexEquals>;
using EdgeClosed = std::unordered_map<EdgeIntervalIndex, std::vector<std::size_t>, EdgeIntervalIndexClosedHash, EdgeIntervalIndexClosedEquals>;

struct Subfunctional{
    double alpha;
    double beta;
    double delta;
    std::size_t node_ind;
    Subfunctional() = default;
    Subfunctional(double _alpha, double _beta, double _delta, std::size_t _node_ind):alpha(_alpha),beta(_beta),delta(_delta),node_ind(_node_ind){};

    inline void debug() const{
        std::cout << alpha << " " << beta << " " << delta << "\n";
    }

    inline double arrival_time(double t) const{
        if (t <= alpha){
            return alpha + delta;
        }
        if (t <= beta){
            return t + delta;
        }
        return std::numeric_limits<double>::infinity();
    }
};




inline double intersection(const Subfunctional& lhs, const Subfunctional& rhs){
    if (rhs.beta > lhs.beta){
        return lhs.beta;
    }
    return rhs.alpha + rhs.delta - lhs.delta;
}


