#pragma once
#include <string>
#include <fstream>
#include <optional>
#include <vector>

class Circuit;

class Parser{
public:
    struct TranAnalysis {
        double timeStep = 0.0;
        double stopTime = 0.0;
        std::optional<double> startTime;
        std::optional<double> maxTimeStep;
        bool useInitialConditions = false;
    };

    Parser(std::string f): file(f), filename(f){}
    ~Parser() = default;

    bool parse(Circuit& circuit);

    const std::optional<TranAnalysis>& tranAnalysis() const{
        return tranAnalysis_;
    }

private:
    bool parseLine(Circuit& circuit, const std::vector<std::string>& tokens);
    bool parseModel(Circuit& circuit, const std::vector<std::string>& tokens);
    bool parseDirective(const std::vector<std::string>& tokens);

    std::ifstream file;
    std::string filename;
    std::optional<TranAnalysis> tranAnalysis_;
};
