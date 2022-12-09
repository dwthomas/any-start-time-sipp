#pragma once

#include <cstddef>
#include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <iterator>
#include "structs.hpp"
#include "node.hpp"


struct sippNode{
    State s;
    std::size_t intervalInd;
    double f;
    std::size_t parent;

    sippNode(int _x, int _y, std::size_t _intervalInd, double _t, double _f, std::size_t _parent):s(_x,_y,_t),intervalInd(_intervalInd),f(_f),parent(_parent){};

    static std::vector<sippNode> nodes;
    
    inline std::size_t hash() const{
        std::size_t seed = 0;
        boost::hash_combine(seed, s.x.x);
        boost::hash_combine(seed, s.x.y);
        boost::hash_combine(seed, intervalInd);
        return seed;
    }

    inline bool equals(const sippNode& rhs) const{
        return s.x == rhs.s.x &&
               intervalInd == rhs.intervalInd;
    }

    static inline std::size_t newNode(int x, int y, std::size_t intervalInd, double t, double f, std::size_t parent){
        std::size_t ind = nodes.size();
        nodes.emplace_back(x, y, intervalInd, t, f, parent);
        return ind;
    }

    static inline sippNode getNode(std::size_t ind){
        assert(ind < nodes.size());
        return nodes[ind];
    }
    
     static inline void remove(std::size_t ind){
        (void)ind;
        assert(ind == (nodes.size()-1));
        nodes.pop_back();
    }

    static inline void set_arrival(std::size_t node_ind, double time, double f,std::size_t cnode){
        nodes[node_ind].f = f;
        nodes[node_ind].s.time = time;
        nodes[node_ind].parent = cnode;
    }

    inline void debug() const{
        s.debug();
        std::cout << "g: " << s.time << " is:" << intervalInd<< " f: " << f << "\n";
    }
};

struct pdapNode{
    State s; // s.time = Delta + alpha
    double alpha;
    double beta;
    double delta;
    std::size_t intervalInd;
    double f;
    std::size_t parent;

    pdapNode(int _x, int _y, std::size_t _intervalInd, double _t, double _alpha, double _beta, double _delta,double _f, std::size_t _parent):s(_x,_y,_t),alpha(_alpha),beta(_beta),delta(_delta),intervalInd(_intervalInd),f(_f),parent(_parent){};

    static std::vector<pdapNode> nodes;
    
    inline std::size_t hash() const{
        std::size_t seed = 0;
        boost::hash_combine(seed, s.x.x);
        boost::hash_combine(seed, s.x.y);
        boost::hash_combine(seed, intervalInd);
        return seed;
    }

    inline bool equals(const pdapNode& rhs) const{
        return s.x == rhs.s.x &&
               intervalInd == rhs.intervalInd;
    }

    static inline std::size_t newNode(int x, int y, std::size_t intervalInd, double t, double alpha, double beta, double delta, double f, std::size_t parent){
        std::size_t ind = nodes.size();
        nodes.emplace_back(x, y, intervalInd, t, alpha, beta, delta, f, parent);
        return ind;
    }

    static inline pdapNode getNode(std::size_t ind){
        assert(ind < nodes.size());
        return nodes[ind];
    }
    
     static inline void remove(std::size_t ind){
        (void)ind;
        assert(ind == (nodes.size()-1));
        nodes.pop_back();
    }

    static inline void set_arrival(std::size_t node_ind, double time, double alpha, double beta, double delta,double f,std::size_t cnode){
        nodes[node_ind].f = f;
        nodes[node_ind].s.time = time;
        nodes[node_ind].parent = cnode;
        nodes[node_ind].alpha = alpha;
        nodes[node_ind].beta = beta;
        nodes[node_ind].delta = delta;
    }

    inline void debug() const{
        s.debug();
        std::cout << "g: " << s.time << " ii:" << intervalInd<< " f: " << f << "alpha: " << alpha << " " << "beta: " << beta << "\n";
    }

    inline void report() const{
        std::cout << s.x.x << " " << s.x.y << " " << s.time <<" " << alpha << " " << beta << " " << delta << "\n";
    }
};

struct partialPdapNode{
    State s; // s.time = Delta + alpha
    double alpha;
    double beta;
    double delta;
    std::size_t intervalInd;
    double f;
    double open_f;
    std::size_t expansions;
    std::size_t parent;

    partialPdapNode(int _x, int _y, std::size_t _intervalInd, double _t, double _alpha, double _beta, double _delta, double _f, double _open_f, std::size_t _expansions, std::size_t _parent):s(_x,_y,_t),alpha(_alpha),beta(_beta),delta(_delta),intervalInd(_intervalInd),f(_f),open_f(_open_f),expansions(_expansions),parent(_parent){};

    static std::vector<partialPdapNode> nodes;
    
    inline std::size_t hash() const{
        std::size_t seed = 0;
        boost::hash_combine(seed, s.x.x);
        boost::hash_combine(seed, s.x.y);
        boost::hash_combine(seed, intervalInd);
        return seed;
    }

    inline bool equals(const pdapNode& rhs) const{
        return s.x == rhs.s.x &&
               intervalInd == rhs.intervalInd;
    }


    static inline std::size_t newNode(int x, int y, std::size_t intervalInd, double t, double alpha, double beta, double delta,double f, double open_f, std::size_t expansions, std::size_t parent){
        std::size_t ind = nodes.size();
        nodes.emplace_back(x, y, intervalInd, t, alpha, beta, delta,f, open_f, expansions, parent);
        return ind;
    }

    static inline partialPdapNode getNode(std::size_t ind){
        assert(ind < nodes.size());
        return nodes[ind];
    }
    
     static inline void remove(std::size_t ind){
        (void)ind;
        assert(ind == (nodes.size()-1));
        nodes.pop_back();
    }

    static inline void set_arrival(std::size_t node_ind, double time, double alpha, double beta, double delta,double f, std::size_t expansions,std::size_t cnode){
        nodes[node_ind].f = f;
        nodes[node_ind].s.time = time;
        nodes[node_ind].expansions = expansions;
        nodes[node_ind].parent = cnode;
        nodes[node_ind].alpha = alpha;
        nodes[node_ind].beta = beta;
        nodes[node_ind].delta = delta;
    }

    static inline void set_f(std::size_t ind, double f, std::size_t expansions = std::numeric_limits<std::size_t>::max()){
        nodes[ind].open_f = f;
        if (expansions == std::numeric_limits<std::size_t>::max()){
            nodes[ind].expansions += 1;
        }
        else{
            nodes[ind].expansions = expansions;
        }
    }

    inline void debug() const{
        s.debug();
        std::cout << "g: " << s.time << " is:" << intervalInd<< " f: " << f << " exp: " << expansions << "\n";
    }
    inline void report() const{
        std::cout << s.x.x << " " << s.x.y << " " << alpha << " " << beta << " " << delta << "\n";
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


struct pdapNodeGreater{
    inline bool operator()(std::size_t lhs_i, std::size_t rhs_i) const{
        const pdapNode& lhs = pdapNode::nodes[lhs_i];
        const pdapNode& rhs = pdapNode::nodes[rhs_i];
        if (lhs.f == rhs.f) {
            if(lhs.s.time == rhs.s.time){
                return lhs.alpha < rhs.alpha;
            }
            return lhs.s.time < rhs.s.time;
        }
        return lhs.f > rhs.f;
    }
};

struct partialPdapNodeGreater{
    inline bool operator()(std::size_t lhs_i, std::size_t rhs_i) const{
        const partialPdapNode& lhs = partialPdapNode::nodes[lhs_i];
        const partialPdapNode& rhs = partialPdapNode::nodes[rhs_i];
        if (lhs.open_f == rhs.open_f) {
            if(lhs.s.time == rhs.s.time){
                return lhs.alpha < rhs.alpha;
            }
            return lhs.s.time < rhs.s.time;
        }
        return lhs.open_f > rhs.open_f;
    }
};

template <typename NodeT, typename SortNodeT>
using NodeOpen = boost::heap::d_ary_heap<std::size_t, boost::heap::compare<SortNodeT>, boost::heap::arity<2>,boost::heap::mutable_<true>>;

template <typename NodeT>
using NodeClosed = std::unordered_map<NodeT, std::size_t, NodeHash<NodeT>, NodeEquals<NodeT>>;