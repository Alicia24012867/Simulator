#pragma once
#include <string>
#include <utility>
#include <vector>

#include "analysisPlan.h"

class Circuit;

class Parser{
public:
    explicit Parser(std::string filename):
        filename_(std::move(filename)) {}

    ~Parser() = default;

    bool parse(Circuit& circuit);

    const AnalysisPlan& analysisPlan() const{
        return analysisPlan_;
    }

    const std::string& title() const{
        return title_;
    }

private:
    bool parseLine(Circuit& circuit, const std::vector<std::string>& tokens);
    bool parseModel(Circuit& circuit, const std::vector<std::string>& tokens);
    bool parseAnalysisDirective(const std::vector<std::string>& tokens);
    void parsePrintDirective(const std::string& line);

    std::string filename_;
    std::string title_;
    AnalysisPlan analysisPlan_;
};
