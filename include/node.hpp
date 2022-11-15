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

    sippNode(int _x, int _y, double _intervalStart, double _t, double _f, std::size_t _ind, std::size_t _parent):s(_x,_y,_t),intervalStart(_intervalStart),f(_f),ind(_ind),parent(_parent){
        std::cout << _t << " " << s.time << "\n";
    };

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

    static inline sippNode newNode(int x, int y, double intervalStart, double t, double f, std::size_t parent){
        std::size_t ind = nodes.size();

        std::cout << "nn sees: " << x << " " << y << " " << t << "\n";
        nodes.emplace_back(x, y, intervalStart, t, f, ind, parent);
        std::cout << "nodes holds: ";
        nodes.back().debug();
        return nodes.back();
    }

    static inline sippNode getNode(std::size_t ind){
        std::cout << "get " << ind << "\n";
        assert(ind < nodes.size());
        return nodes[ind];
    }
    
    inline void debug() const{
        s.debug();
        std::cout << "f: " << f << "\n";
    }

};

template <typename NodeT>
struct NodeHash{
    inline std::size_t operator()(const NodeT& node) const{
        return node.hash();
    }
};

template <typename NodeT>
struct NodeEquals{
    inline bool operator()(const NodeT& lhs, const NodeT& rhs) const{
        return lhs.equals(rhs);
    }
};

template <typename NodeT>
struct NodeLess{
    inline bool operator()(const NodeT& lhs, const NodeT& rhs) const{
        if (lhs.f == rhs.f) {
            return lhs.s.time > rhs.s.time;
        }
        return lhs.f < rhs.f;
    }
};

template <typename NodeT>
struct NodeGreater{
    inline bool operator()(const NodeT& lhs, const NodeT& rhs) const{
        return !NodeLess<NodeT>()(lhs, rhs);
    }
};

template <typename NodeT>
using NodeOpen = boost::heap::priority_queue<NodeT, boost::heap::compare<NodeGreater<NodeT>>>;