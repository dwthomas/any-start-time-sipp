#include "metadata.hpp"
#include "map.hpp"

int main(int argc, char *argv[]){
    Metadata metadata = Metadata(argc, argv);
    Map map = Map(metadata.mapfile());
}