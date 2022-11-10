#pragma once
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_smallint.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/container/flat_map.hpp>
#include <cstdint>
#include <memory>
#include "dynamicObstacle.hpp"
#include "map.hpp"


class RandomDynamicObstacle: public DynamicObstacle{
    private:
        double good_until;
        double speed;
        boost::random::mt11213b generator;
        boost::container::flat_map<double, State> specified_path;
        boost::random::uniform_smallint<unsigned int> movement_direction;
        boost::random::uniform_real_distribution<float> wait_duration;
        unsigned int get_max(int x, int y, int dx, int dy) const;
        void generate(double until);
        const Map & map;
    public:
        RandomDynamicObstacle(int x, int y, const Map& _map, double _speed, float min_wait, float max_wait);
        std::vector<State> path(double starting, double ending);
        static std::vector<std::shared_ptr<DynamicObstacle>> read_obstacles(const std::string& filename , const Map& map, float min_wait, float max_wait);
};