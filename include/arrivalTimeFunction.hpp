#pragma once
#include "node.hpp"
#include <cstddef>
#include <iterator>
#include <limits>

struct Subfunctional{
    double alpha;
    double beta;
    double delta;
    std::size_t node_ind;
    Subfunctional() = default;
    constexpr Subfunctional(double _alpha, double _beta, double _delta, std::size_t _node_ind):alpha(_alpha),beta(_beta),delta(_delta),node_ind(_node_ind){};

    inline void debug() const{
        std::cout << alpha << " " << beta << " " << delta << "\n";
    }

    constexpr double arrival_time(double t) const{
        if (t <= std::min(alpha, beta)){
            return alpha + delta;
        }
        if (t <= beta){
            return t + delta;
        }
        return std::numeric_limits<double>::infinity();
    }
};

struct Segment {
    double start;
    std::size_t subfunctional;
    constexpr Segment(double _start, std::size_t _subfunctional):start(_start),subfunctional(_subfunctional){}
};

struct Functional{
    std::vector<Subfunctional> sfs;
    boost::container::flat_map<double, Segment> dominant;
    Functional(){
        sfs.emplace_back(0.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),std::numeric_limits<std::size_t>::max());
        dominant.emplace(std::numeric_limits<double>::infinity(), 0.0, 0);
    }

    inline double arrival_time(double t) const{
        auto it = dominant.lower_bound(t);
        return sfs[it->second.subfunctional].arrival_time(t);
    }

    std::size_t insert_flat(const Subfunctional& sf, double begin, double end){
        double at = sf.arrival_time(end);
        if (arrival_time(end) > at){
            auto it = dominant.lower_bound(end);
            while(sfs[it->second.subfunctional].arrival_time(it->second.start) > at){
                it = dominant.erase(it);
                if (it == dominant.begin()){
                    break;
                }
                --it;
            }
        }
        return std::numeric_limits<std::size_t>::max(); // not necissary
    }

    inline void insert(const Subfunctional& sf){
        double flat_end = std::min(sf.alpha, sf.beta);
        auto ind = insert_flat(sf, 0.0, flat_end);
        if (flat_end != sf.beta){
            insert_rising(sf, sf.alpha, sf.beta, ind);
        }
    }
};

inline bool dominates(const Subfunctional& lhs, const Subfunctional& rhs, const Functional& functional){
    if (rhs.beta > lhs.beta){
        return false;
    }
    if (rhs.arrival_time(rhs.beta) >= lhs.arrival_time(rhs.beta) && rhs.arrival_time(rhs.alpha) >= lhs.arrival_time(rhs.alpha)){
        return true;
    }
    else if(rhs.arrival_time(rhs.beta) >= functional.arrival_time(rhs.beta) &&((rhs.alpha >= rhs.beta) || rhs.arrival_time(rhs.alpha) >= functional.arrival_time(rhs.alpha))){
        return true;
    }
    return false;
}