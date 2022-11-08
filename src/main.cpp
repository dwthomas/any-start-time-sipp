#include <iostream>
#include "boost/program_options.hpp"
namespace po = boost::program_options;

po::variables_map parse_arguments(int argc, char *argv[]){
    po::variables_map vm;
    try{
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help", "produce help message")
            ("obstacles", po::value<int>()->required(), "The number of moving obstacles to generate.")
            ("map", po::value<std::string>()->required(), "The filepath of the static map to use.")
            ("startx", po::value<int>()->required(), "The starting x location")
            ("starty", po::value<int>()->required(), "The starting y location")
            ("goalx", po::value<int>()->required(), "The goal x location")
            ("goaly", po::value<int>()->required(), "The goal y location")
        ;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")){
                std::cout << desc << "\n";
                exit(EXIT_SUCCESS);
            }
        po::notify(vm);
    }    
    catch(std::exception& e){
            std::cerr << "Error: " << e.what() << "\n";
            exit(EXIT_FAILURE);
    }
    catch(...){
        std::cerr << "Unknown error!" << "\n";
        exit(EXIT_FAILURE);
    }
    return vm;
}



int main(int argc, char *argv[]){
    po::variables_map vm = parse_arguments(argc, argv);
}