
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <string>
#include <fstream>
#include <vector>

inline bool isOccupied(char c){
    switch (c){
        case '.':
            return false;
        case 'G':
            return false;
        case 'S':
            return false;
        case '@':
            return true;
        case 'O':
            return true;
        case 'T':
            return true;
        case 'W':
            return true;
        default :
            std::cerr << "ERROR: unknown map symbol: '" << c << "'\n"; 
            exit(EXIT_FAILURE);
    }
}



struct Map{
    unsigned int height;
    unsigned int width;
    std::size_t size;
    std::vector<bool> occupancy;

    Map(std::string filename){
        std::ifstream mapfile;
        std::string line;
        mapfile.open(filename);

        mapfile >> line;
        assert(line == "type");
        mapfile >> line;
        assert(line == "octile");


        mapfile >> line;
        assert(line == "height");
        mapfile >> height;

        mapfile >> line;
        assert(line == "width");
        mapfile >> width;

        size = width*height;
        occupancy.resize(size);

        mapfile >> line;
        assert(line == "map");
        char c;
        std::size_t ind = 0; 
        while(mapfile.get(c) && ind < size){
            if (c == '\n'){
                mapfile.get(c);
            }
            occupancy[ind++] = isOccupied(c);
        }
        mapfile.close();
        assert(ind == size);
    }

    constexpr void checkBounds(int x, int y){
        assert (x >= 0 && (unsigned int)x < width);
        assert (y >= 0 && (unsigned int)y < height);
    }

    inline std::size_t getIndex(int x, int y){
        checkBounds(x, y);
        return x + width*y; 
    }

    inline bool isBlocked(int x, int y){
        checkBounds(x, y);
        return occupancy[getIndex(x, y)];
    }
};