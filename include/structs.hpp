#pragma once

#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>
#include <boost/container/flat_map.hpp>
#include <boost/functional/hash.hpp>

typedef std::pair<double, double> safe_interval;

struct Location{
    int x;
    int y;
    constexpr Location() = default;
    constexpr Location(int _x, int _y):x(_x),y(_y){}
    constexpr bool operator ==(const Location& l) const{
        return x == l.x && y == l.y;
    }
    void debug() const{
        std::cout << x << " " << y;
    }
};

using Configuration = Location;

struct State{
    Configuration x;
    double time;

    constexpr State(int _x, int _y, double _t):x(_x,_y),time(_t){}
    constexpr bool operator ==(const State & s) const{
        return x == s.x && time == s.time;
    }
    inline void debug() const{
        x.debug();
        std::cout << " " << time << "\n";
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
        std::cout << source.x.x << " " << source.x.y << " " << source.time << " -> ";
        std::cout << destination.x.x << " " << destination.x.y << " " << destination.time << "\n";
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


using SafeIntervalContainer = boost::container::flat_map<double, double>;
// key is END, value is BEGIN, so we can index in to time t with lower_bound()
using EdgeIntervals = std::unordered_map<EdgeIntervalIndex, SafeIntervalContainer, EdgeIntervalIndexHash, EdgeIntervalIndexEquals>;
using EdgeClosed = std::unordered_map<EdgeIntervalIndex, std::vector<std::size_t>, EdgeIntervalIndexClosedHash, EdgeIntervalIndexClosedEquals>;




