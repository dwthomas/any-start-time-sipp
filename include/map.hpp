#pragma once

#include <boost/assert.hpp>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <iterator>
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

    constexpr void checkBounds(int x, int y) const{
        assert (x >= 0 && (unsigned int)x < width);
        assert (y >= 0 && (unsigned int)y < height);
    }
    constexpr bool inBounds(int x, int y) const{
        return (x >= 0 && (unsigned int)x < width) && (y >= 0 && (unsigned int)y < height);
    }

    inline std::size_t getIndex(int x, int y) const{
        checkBounds(x, y);
        return x + width*y; 
    }

    inline std::size_t getIndex(State s) const{
        return getIndex(s.x, s.y); 
    }

    inline bool isBlocked(int x, int y) const{
        return occupancy[getIndex(x, y)];
    }

    inline bool isSafe(int x, int y) const{
        return !isBlocked(x, y);
    }

    inline std::size_t get_safe_interval_ind(const State& s) const{
        return 4*getIndex(s);
    }

    inline std::vector<std::size_t> get_safe_interval_ind(const Action& action) const{
        // return source, destination, edge indexes or source index
        auto res = std::vector<std::size_t>();
        const auto& source = action.source;
        const auto& destination = action.destination;
        res.reserve(3);
        res.emplace_back(4*getIndex(source));
        if (source == destination){
            return res;
        }
        res.emplace_back(4*getIndex(destination));
        int shift = (destination.x != source.x) | ((destination.y != source.y) << 1);
        res.emplace_back(4*getIndex(std::min(destination.x, source.x), std::min(destination.y, source.y)) + shift);
        return res;
    }

    void debug()const{
        for (uint j = 0; j < height; j++){
            for (uint i = 0; i < width; i++){
                std::cout << isSafe(i, j);
            }
            std::cout << "\n";
        }
    }
};