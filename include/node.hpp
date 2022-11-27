#pragma once

#include <cstddef>
#include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <iterator>
#include "structs.hpp"
#include "node.hpp"

struct Functional;
inline bool dominates(const Subfunctional& lhs, const Subfunctional& rhs, const Functional& functional);

struct sippNode{
    State s;
    std::size_t intervalInd;
    double f;
    std::size_t parent;

    sippNode(int _x, int _y, std::size_t _intervalInd, double _t, double _f, std::size_t _parent):s(_x,_y,_t),intervalInd(_intervalInd),f(_f),parent(_parent){};

    static std::vector<sippNode> nodes;
    
    inline std::size_t hash() const{
        std::size_t seed = 0;
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        boost::hash_combine(seed, intervalInd);
        return seed;
    }

    inline bool equals(const sippNode& rhs) const{
        return s.x == rhs.s.x &&
               s.y == rhs.s.y && 
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
    std::size_t intervalInd;
    double f;
    std::size_t parent;

    pdapNode(int _x, int _y, std::size_t _intervalInd, double _t, double _alpha, double _beta, double _f, std::size_t _parent):s(_x,_y,_t),alpha(_alpha),beta(_beta),intervalInd(_intervalInd),f(_f),parent(_parent){};

    static std::vector<pdapNode> nodes;
    
    inline std::size_t hash() const{
        std::size_t seed = 0;
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        boost::hash_combine(seed, intervalInd);
        return seed;
    }

    inline bool equals(const pdapNode& rhs) const{
        return s.x == rhs.s.x &&
               s.y == rhs.s.y && 
               intervalInd == rhs.intervalInd;
    }

    inline double delta()const{
        return s.time - alpha;
    }

    static inline std::size_t newNode(int x, int y, std::size_t intervalInd, double t, double alpha, double beta, double f, std::size_t parent){
        std::size_t ind = nodes.size();
        nodes.emplace_back(x, y, intervalInd, t, alpha, beta, f, parent);
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

    static inline void set_arrival(std::size_t node_ind, double time, double alpha, double beta, double f,std::size_t cnode){
        nodes[node_ind].f = f;
        nodes[node_ind].s.time = time;
        nodes[node_ind].parent = cnode;
        nodes[node_ind].alpha = alpha;
        nodes[node_ind].beta = beta;
    }

    inline void debug() const{
        s.debug();
        std::cout << "g: " << s.time << " is:" << intervalInd<< " f: " << f << "\n";
    }

    inline void report() const{
        std::cout << s.x << " " << s.y << " " << alpha << " " << beta << " " << delta() << "\n";
    }
};

struct partialPdapNode{
    State s; // s.time = Delta + alpha
    double alpha;
    double beta;
    std::size_t intervalInd;
    double f;
    double open_f;
    std::size_t expansions;
    std::size_t parent;

    partialPdapNode(int _x, int _y, std::size_t _intervalInd, double _t, double _alpha, double _beta, double _f, std::size_t _expansions, std::size_t _parent):s(_x,_y,_t),alpha(_alpha),beta(_beta),intervalInd(_intervalInd),f(_f),open_f(f),expansions(_expansions),parent(_parent){};

    static std::vector<partialPdapNode> nodes;
    
    inline std::size_t hash() const{
        std::size_t seed = 0;
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        boost::hash_combine(seed, intervalInd);
        return seed;
    }

    inline bool equals(const pdapNode& rhs) const{
        return s.x == rhs.s.x &&
               s.y == rhs.s.y && 
               intervalInd == rhs.intervalInd;
    }

    inline double delta()const{
        return s.time - alpha;
    }

    static inline std::size_t newNode(int x, int y, std::size_t intervalInd, double t, double alpha, double beta, double f, std::size_t expansions, std::size_t parent){
        std::size_t ind = nodes.size();
        nodes.emplace_back(x, y, intervalInd, t, alpha, beta, f, expansions, parent);
        return ind;
    }

    static inline partialPdapNode getNode(std::size_t ind){
        assert(ind < nodes.size());
        return nodes[ind];
    }
    
     static inline void remove(std::size_t ind){
        assert(ind == (nodes.size()-1));
        nodes.pop_back();
    }

    static inline void set_arrival(std::size_t node_ind, double time, double alpha, double beta, double f, std::size_t expansions,std::size_t cnode){
        nodes[node_ind].f = f;
        nodes[node_ind].s.time = time;
        nodes[node_ind].expansions = expansions;
        nodes[node_ind].parent = cnode;
        nodes[node_ind].alpha = alpha;
        nodes[node_ind].beta = beta;
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
        std::cout << s.x << " " << s.y << " " << alpha << " " << beta << " " << delta() << "\n";
    }
};


struct Functional{
    boost::container::flat_map<double, Subfunctional> domain;

    inline double arrival_time(double t) const{
        auto it = domain.upper_bound(t);
        if (it != domain.begin()){
            it = std::prev(it);
        }
        return it->second.arrival_time(t); 
    }

    inline double check_back(std::size_t node_ind){
        const partialPdapNode& cn = partialPdapNode::getNode(node_ind);
        Subfunctional prospect(cn.alpha, cn.beta, cn.f, node_ind);
        if (domain.size()==0){
            return true;
        }
        auto encumbent_it = domain.nth(domain.size()-1);
        auto encumbent = encumbent_it->second;
        if (!dominates(encumbent, prospect, *this)){
            return true;
        }
        return false;
    }

    inline double emplace_back(double alpha, double beta, double delta, std::size_t node_ind){
        Subfunctional prospect(alpha, beta, delta, node_ind);
        //prospect.debug();
        //debug();
        if (domain.size()==0){
            domain[0.0] = prospect;
            return std::max(0.0, prospect.alpha);
        }
        auto encumbent_it = domain.nth(domain.size()-1);
        auto encumbent = encumbent_it->second;
        if (!dominates(encumbent, prospect, *this)){
            double minimization_image = intersection(encumbent, prospect);

            assert(minimization_image > encumbent_it->first);
            domain[minimization_image] = prospect;
            return std::max(minimization_image, prospect.alpha);
        }
        return std::max(encumbent_it->first, encumbent.alpha);
    }

    inline double finite_until() const{
        return domain.nth(domain.size()-1)->second.beta;
    }

    inline void debug() const{
        for (const auto& it: domain){
            std::cout << it.first << ": ";
            it.second.debug();
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
    else if(functional.arrival_time(rhs.beta) >= functional.arrival_time(rhs.beta) && functional.arrival_time(rhs.alpha) >= functional.arrival_time(rhs.alpha)){
        return true;
    }
    return false;
}

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