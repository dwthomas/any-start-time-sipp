#pragma once

#include <cstddef>
#include <boost/functional/hash.hpp>
#include <boost/heap/priority_queue.hpp>
#include <iterator>
#include "structs.hpp"
#include "node.hpp"

struct sippNode{
    State s;
    double intervalStart;
    double f;
    std::size_t ind;
    std::size_t parent;

    sippNode(int _x, int _y, double _intervalStart, double _t, double _f, std::size_t _ind, std::size_t _parent):s(_x,_y,_t),intervalStart(_intervalStart),f(_f),ind(_ind),parent(_parent){};

    static std::vector<sippNode> nodes;
    
    inline std::size_t hash() const{
        std::size_t seed = 0;
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        boost::hash_combine(seed, intervalStart);
        return seed;
    }

    inline bool equals(const sippNode& rhs) const{
        return s.x == rhs.s.x &&
               s.y == rhs.s.y && 
               intervalStart == rhs.intervalStart;
    }

    static inline std::size_t newNode(int x, int y, double intervalStart, double t, double f, std::size_t parent){
        std::size_t ind = nodes.size();
        nodes.emplace_back(x, y, intervalStart, t, f, ind, parent);
        return ind;
    }

    static inline sippNode getNode(std::size_t ind){
        assert(ind < nodes.size());
        return nodes[ind];
    }
    
    inline void debug() const{
        s.debug();
        std::cout << "g: " << s.time << " is:" << intervalStart<< " f: " << f << "\n";
    }

};

template <typename NodeT>
struct NodeHash{
    inline std::size_t operator()(std::size_t ind) const{
        return NodeT::nodes[ind].hash();
    }
};

template <typename NodeT>
struct NodeEquals{
    inline bool operator()(std::size_t lhs, std::size_t rhs) const{
        return NodeT::nodes[lhs].equals(NodeT::nodes[rhs]);
    }
};

template <typename NodeT>
struct NodeLess{
    inline bool operator()(std::size_t lhs_i, std::size_t rhs_i) const{
        const NodeT& lhs = NodeT::nodes[lhs_i];
        const NodeT& rhs = NodeT::nodes[rhs_i];
        if (lhs.f == rhs.f) {
            return lhs.s.time > rhs.s.time;
        }
        return lhs.f < rhs.f;
    }
};

template <typename NodeT>
struct NodeGreater{
    inline bool operator()(std::size_t lhs_i, std::size_t rhs_i) const{
        const NodeT& lhs = NodeT::nodes[lhs_i];
        const NodeT& rhs = NodeT::nodes[rhs_i];
        if (lhs.f == rhs.f) {
            return lhs.s.time < rhs.s.time;
        }
        return lhs.f > rhs.f;
    }
};

template <typename NodeT>
using NodeOpen = boost::heap::priority_queue<std::size_t, boost::heap::compare<NodeGreater<NodeT>>>;