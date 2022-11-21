#pragma once
#include <iostream>
#include "boost/program_options.hpp"
#include "boost/timer/timer.hpp"
namespace po = boost::program_options;

inline po::variables_map parse_arguments(int argc, char *argv[]){
    po::variables_map vm;
    try{
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help", "produce help message")
            ("obstacles", po::value<std::string>()->required(), "The file specifying obstacles to generate.")
            ("map", po::value<std::string>()->required(), "The filepath of the static map to use.")
            ("search", po::value<std::string>()->required(), "The search to run, sipp, pdap...")
            ("startx", po::value<int>()->required(), "The starting x location")
            ("starty", po::value<int>()->required(), "The starting y location")
            ("goalx", po::value<int>()->required(), "The goal x location")
            ("goaly", po::value<int>()->required(), "The goal y location")
            ("startt", po::value<double>()->default_value(0.0), "The starting time")
            ("startendt", po::value<double>()->default_value(10.0), "The end of the safe interval at start")
            ("allendt", po::value<double>()->default_value(1000.0), "The end of the safe interval everywhere")
            ("minwait", boost::program_options::value<double>()->default_value(0.0), "Minimum obstacle wait")
            ("maxwait", boost::program_options::value<double>()->default_value(2.0), "Maximum obstacle wait")
            ("aspeed", boost::program_options::value<double>()->default_value(1.0))
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

class Metadata{
    private:
        po::variables_map vm;
    public:
        boost::timer::cpu_timer runtime;
        long expansions;
        long plan_attempts;
        Metadata(int argc, char *argv[]):expansions(0),plan_attempts(0){
            vm = parse_arguments(argc, argv);
        }

        inline std::string mapfile(){
            return vm["map"].as<std::string>();
        }

        po::variables_map args(){
            return vm;
        }
};