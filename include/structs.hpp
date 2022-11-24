#pragma once

#include <iostream>
#include <limits>
#include <vector>
#include <boost/container/flat_map.hpp>

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


inline bool dominates(const Subfunctional& lhs, const Subfunctional& rhs){
    if (rhs.beta > lhs.beta){
        return false;
    }
    if (rhs.arrival_time(rhs.beta) >= lhs.arrival_time(rhs.beta) && rhs.arrival_time(rhs.alpha) >= lhs.arrival_time(rhs.alpha)){
        return true;
    }
    return false;
}

inline double intersection(const Subfunctional& lhs, const Subfunctional& rhs){
    if (rhs.beta > lhs.beta){
        return lhs.beta;
    }
    return rhs.alpha + rhs.delta - lhs.delta;
}


