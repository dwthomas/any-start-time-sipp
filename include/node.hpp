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
    std::size_t parent;

    sippNode(int _x, int _y, double _intervalStart, double _t, double _f, std::size_t _ind, std::size_t _parent):s(_x,_y,_t),intervalStart(_intervalStart),f(_f),parent(_parent){};

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
    
     static inline void remove(std::size_t ind){
        assert(ind == (nodes.size()-1));
        nodes.pop_back();
    }

    static inline void set_arrival(std::size_t node_ind, double time, std::size_t cnode){
        nodes[node_ind].f = nodes[node_ind].f - nodes[node_ind].s.time + time;
        nodes[node_ind].s.time = time;
        nodes[node_ind].parent = cnode;
    }

    inline void debug() const{
        s.debug();
        std::cout << "g: " << s.time << " is:" << intervalStart<< " f: " << f << "\n";
    }
};

struct pdapNode{
    State s; // s.time = Delta + alpha
    double alpha;
    double beta;
    double intervalStart;
    double f;
    std::size_t parent;

    pdapNode(int _x, int _y, double _intervalStart, double _t, double _alpha, double _beta, double _f, std::size_t _ind, std::size_t _parent):s(_x,_y,_t),alpha(_alpha),beta(_beta),intervalStart(_intervalStart),f(_f),parent(_parent){};

    static std::vector<pdapNode> nodes;
    
    inline std::size_t hash() const{
        std::size_t seed = 0;
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        boost::hash_combine(seed, intervalStart);
        return seed;
    }

    inline bool equals(const pdapNode& rhs) const{
        return s.x == rhs.s.x &&
               s.y == rhs.s.y && 
               intervalStart == rhs.intervalStart;
    }

    static inline std::size_t newNode(int x, int y, double intervalStart, double t, double alpha, double beta, double f, std::size_t parent){
        std::size_t ind = nodes.size();
        nodes.emplace_back(x, y, intervalStart, t, alpha, beta, f, ind, parent);
        return ind;
    }

    static inline pdapNode getNode(std::size_t ind){
        assert(ind < nodes.size());
        return nodes[ind];
    }
    
     static inline void remove(std::size_t ind){
        assert(ind == (nodes.size()-1));
        nodes.pop_back();
    }

    static inline void set_arrival(std::size_t node_ind, double time, double alpha, double beta, std::size_t cnode){
        nodes[node_ind].f = nodes[node_ind].f - nodes[node_ind].s.time + time;
        nodes[node_ind].s.time = time;
        nodes[node_ind].parent = cnode;
        nodes[node_ind].alpha = alpha;
        nodes[node_ind].beta = beta;
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