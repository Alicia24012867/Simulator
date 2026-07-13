#pragma once
#include <fstream>
#include <string>
#include <vector>

#include "analysisPlan.h"

class Circuit;

class Parser{
public:
    Parser(std::string f): file(f), filename(f){}
    ~Parser() = default;

    bool parse(Circuit& circuit);

    const AnalysisPlan& analysisPlan() const{
        return analysisPlan_;
    }

private:
    bool parseLine(Circuit& circuit, const std::vector<std::string>& tokens);
    bool parseModel(Circuit& circuit, const std::vector<std::string>& tokens);
    bool parseAnalysisDirective(const std::vector<std::string>& tokens);

    std::ifstream file;
    std::string filename;
    AnalysisPlan analysisPlan_;
};
