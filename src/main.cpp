#include "metadata.hpp"
#include "map.hpp"
#include "dynamicObstacle.hpp"
#include "randomDynamicObstacle.hpp"
#include "safeIntervals.hpp"

int main(int argc, char *argv[]){
    Metadata metadata = Metadata(argc, argv);
    Map map = Map(metadata.mapfile());
    std::vector<std::shared_ptr<DynamicObstacle>> obs = RandomDynamicObstacle::read_obstacles(metadata.args()["obs"].as<std::string>(), map, metadata.args()["minwait"].as<float>(), metadata.args()["maxwait"].as<float>());
    SafeIntervals safe_intervals = SafeIntervals(obs, map);
}