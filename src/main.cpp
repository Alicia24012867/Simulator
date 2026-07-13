#include <iostream>
#include <fstream>

#include "../include/core/parser.h"
#include "../include/core/circuit.h"

int main(int argc, char* argv[]){
    if(argc != 2 && argc != 3){
        std::cerr << "Usage: " << argv[0] << " <inputfile> " <<
            " <outputfile>[optional] " << std::endl;
        return 1;
    }

    std::ofstream ofs;
    std::ostream* os = &std::cout;
    if(argc == 3){
        ofs.open(argv[2], std::ios::out | std::ios::trunc);
        if(!ofs){
            std::cerr << "Fail to open/create " << argv[2] << std::endl;
            return 1;
        }
        os = &ofs;
    }

    Parser parser(argv[1]);
    Circuit circuit;

    if(!parser.parse(circuit)){
        std::cerr << "Fail to parse file <" << argv[1] << "> " << std::endl;
        return 1;
    }

    if(!circuit.build()){
        std::cerr << "Fail to build circuit <" << argv[1] << "> " << std::endl;
        return 1;
    }

    if(!circuit.solveOperatingPoint()){
        std::cerr << "Fail to solve circuit <" << argv[1] << "> " << std::endl;
        return 1;
    }

    circuit.printOperatingPoint(*os);

    return 0;
}
