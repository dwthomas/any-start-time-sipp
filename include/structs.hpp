#pragma once

#include <iostream>

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