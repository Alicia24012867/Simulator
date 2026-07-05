#pragma once
#include <string>
#include <fstream>
#include <vector>

class Circuit;

class Parser{
public:
    Parser(std::string f): file(f), filename(f){}
    ~Parser() = default;

    bool parse(Circuit& circuit);

private:
    bool parseLine(Circuit& circuit, const std::vector<std::string>& tokens);
    bool parseModel(Circuit& circuit, const std::vector<std::string>& tokens);

    std::ifstream file;
    std::string filename;
};
