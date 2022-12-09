#pragma once

#include <array>
#include <boost/assert.hpp>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <fstream>
#include <vector>
#include "structs.hpp"
#include "constants.hpp"



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

    constexpr void checkBounds(const Location& loc) const{
        (void)loc;
        assert (loc.x >= 0 && (unsigned int)loc.x < width);
        assert (loc.y >= 0 && (unsigned int)loc.y < height);
    }
    constexpr bool inBounds(const Location& loc) const{
        return (loc.x >= 0 && (unsigned int)loc.x < width) && (loc.y >= 0 && (unsigned int)loc.y < height);
    }

    inline std::size_t getIndex(const Location& loc) const{
        checkBounds(loc);
        return loc.x + width*loc.y; 
    }

    inline std::size_t getIndex(State s) const{
        return getIndex(s.x); 
    }

    inline bool isBlocked(const Location& loc) const{
        return occupancy[getIndex(loc)];
    }

    inline bool isSafe(const Location& loc) const{
        return !isBlocked(loc);
    }

    inline std::size_t get_safe_interval_ind(const State& s) const{
        return 4*getIndex(s);
    }

    inline void get_safe_interval_ind(const Action& action, std::array<std::size_t,3>& res) const{
        // return source, destination, edge indexes or source index
        const auto& source = action.source;
        const auto& destination = action.destination;
        res[0] = 4*getIndex(source);
        if (source == destination){
            res[1] = std::numeric_limits<std::size_t>::max();
            res[2] = std::numeric_limits<std::size_t>::max();
            return;
        }
        res[1] = get_safe_interval_ind(destination);
        int shift = (destination.x.x != source.x.x) | ((destination.x.y != source.x.y) << 1);
        res[2] = 4*getIndex(Location(std::min(destination.x.x, source.x.x), std::min(destination.x.y, source.x.y))) + shift;
        return;
    }

    void debug()const{
        for (uint j = 0; j < height; j++){
            for (uint i = 0; i < width; i++){
                std::cout << isSafe(Location(i, j));
            }
            std::cout << "\n";
        }
    }
};