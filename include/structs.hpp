#pragma once

struct State{
    int x;
    int y;
    double time;
    State(int _x, int _y, int _t):x(_x),y(_y),time(_t){}
    inline bool operator ==(const State & s) const{
        return x == s.x && y == s.y && time == s.time;
    }
};



struct Action{
    State source;
    State destination;
    Action(State s, State d):source(s),destination(d){}
    inline bool operator ==(const Action & a) const{
        return source == a.source && destination == a.destination;
    }
};