#include <iostream>
#include <memory>
#include <stdexcept>

#include "../include/core/parser.h"
#include "../include/core/circuit.h"
#include "../include/devices/bjt.hpp"
#include "../include/devices/capacitor.hpp"
#include "../include/devices/currentSource.hpp"
#include "../include/devices/diode.hpp"
#include "../include/devices/inductor.hpp"
#include "../include/devices/mosfet.hpp"
#include "../include/devices/resistor.hpp"
#include "../include/devices/voltageSource.hpp"
#include "../include/models/model.hpp"

#include "../../utils/string_utils.hpp"

namespace {
ModelType parseModelType(const std::string& token){
    const std::string type = to_lower_copy(token);
    if(type == "d") return ModelType::Diode;
    if(type == "npn") return ModelType::NPN;
    if(type == "pnp") return ModelType::PNP;
    if(type == "nmos" || type == "nch") return ModelType::NMOS;
    if(type == "pmos" || type == "pch") return ModelType::PMOS;
    return ModelType::Unknown;
}
}

bool Parser::parse(Circuit& circuit){
    if (!file.is_open()) {
        std::cerr << "Can not open file <" << filename << ">!" << std::endl;
        return false;
    }

    tranAnalysis_.reset();

    std::string line;
    bool firstLine = true;

    std::vector<std::vector<std::string>> lines;
    while(std::getline(file, line)){
        if(firstLine){
            firstLine = false;
            continue;
        }

        line = trim(strip_spice_comment(line));
        if(line.empty() || line[0] == '*'){
            continue;
        }

        if(line[0] == '+'){
            if(lines.empty()){
                std::cerr << "Continuation line without previous line" << std::endl;
                return false;
            }
            std::vector<std::string> extra = tokenize_spice_line(line.substr(1));
            lines.back().insert(lines.back().end(), extra.begin(), extra.end());
            continue;
        }

        std::vector<std::string> tokens = tokenize_spice_line(line);
        if(tokens.empty()){
            continue;
        }
        if(equal_ignore_case(tokens[0], ".end")){
            break;
        }
        lines.push_back(tokens);
    }

    try {
        for(const auto& tokens: lines){
            if(tokens[0][0] == '.' && !parseDirective(tokens)){
                return false;
            }
        }

        for(const auto& tokens: lines){
            if(equal_ignore_case(tokens[0], ".model") && !parseModel(circuit, tokens)){
                return false;
            }
        }

        for(const auto& tokens: lines){
            if(tokens[0][0] == '.'){
                continue;
            }
            if(!parseLine(circuit, tokens)){
                return false;
            }
        }
    } catch(const std::exception& ex){
        std::cerr << "Parse error: " << ex.what() << std::endl;
        return false;
    }

    return true;
}

bool Parser::parseDirective(const std::vector<std::string>& tokens){
    if(!equal_ignore_case(tokens[0], ".tran")){
        return true;
    }

    if(tranAnalysis_){
        throw std::runtime_error("Multiple .tran directives are not supported");
    }
    if(tokens.size() < 3){
        throw std::runtime_error(".tran requires TSTEP and TSTOP");
    }

    TranAnalysis analysis;
    analysis.timeStep = parse_spice_number(tokens[1]);
    analysis.stopTime = parse_spice_number(tokens[2]);

    if(analysis.timeStep <= 0.0){
        throw std::runtime_error(".tran TSTEP must be positive");
    }
    if(analysis.stopTime <= 0.0){
        throw std::runtime_error(".tran TSTOP must be positive");
    }

    std::vector<double> optionalTimes;
    for(std::size_t i = 3; i < tokens.size(); ++i){
        if(equal_ignore_case(tokens[i], "uic")){
            if(analysis.useInitialConditions){
                throw std::runtime_error(".tran specifies UIC more than once");
            }
            analysis.useInitialConditions = true;
            continue;
        }
        optionalTimes.push_back(parse_spice_number(tokens[i]));
    }

    if(optionalTimes.size() > 2){
        throw std::runtime_error(".tran accepts at most TSTART and TMAX after TSTEP and TSTOP");
    }
    if(!optionalTimes.empty()){
        analysis.startTime = optionalTimes[0];
        if(*analysis.startTime < 0.0 || *analysis.startTime >= analysis.stopTime){
            throw std::runtime_error(".tran TSTART must be non-negative and smaller than TSTOP");
        }
    }
    if(optionalTimes.size() == 2){
        analysis.maxTimeStep = optionalTimes[1];
        if(*analysis.maxTimeStep <= 0.0){
            throw std::runtime_error(".tran TMAX must be positive");
        }
    }

    tranAnalysis_ = analysis;
    return true;
}

bool Parser::parseModel(Circuit& circuit, const std::vector<std::string>& tokens){
    if(tokens.size() < 3){
        std::cerr << ".model requires name and type" << std::endl;
        return false;
    }

    const ModelType type = parseModelType(tokens[2]);
    if(type == ModelType::Unknown){
        std::cerr << "Unsupported model type: " << tokens[2] << std::endl;
        return false;
    }

    auto model = std::make_unique<Model>(to_lower_copy(tokens[1]), type);

    for(std::size_t i = 3; i < tokens.size(); ++i){
        std::string key;
        std::string value;
        if(read_spice_assignment(tokens, i, key, value)){
            model->setParam(key, parse_spice_number(value));
        }
    }

    circuit.addModel(std::move(model));
    return true;
}

bool Parser::parseLine(Circuit& circuit, const std::vector<std::string>& tokens){
    if(tokens[0].empty()){
        return true;
    }

    const char type = static_cast<char>(std::toupper(static_cast<unsigned char>(tokens[0][0])));
    switch(type){
        case 'R':
            if(tokens.size() < 4) throw std::runtime_error("Bad resistor line");
            circuit.addDevice<Resistor>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, parse_spice_value_token(tokens, 3));
            return true;
        case 'C':
            if(tokens.size() < 4) throw std::runtime_error("Bad capacitor line");
            circuit.addDevice<Capacitor>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, parse_spice_value_token(tokens, 3));
            return true;
        case 'L':
            if(tokens.size() < 4) throw std::runtime_error("Bad inductor line");
            circuit.addDevice<Inductor>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, parse_spice_value_token(tokens, 3));
            return true;
        case 'V':
            if(tokens.size() < 4) throw std::runtime_error("Bad voltage source line");
            circuit.addDevice<VoltageSource>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, parse_spice_value_token(tokens, 3));
            return true;
        case 'I':
            if(tokens.size() < 4) throw std::runtime_error("Bad current source line");
            circuit.addDevice<CurrentSource>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, parse_spice_value_token(tokens, 3));
            return true;
        case 'D': {
            if(tokens.size() < 4) throw std::runtime_error("Bad diode line");
            const Model* model = circuit.findModel(to_lower_copy(tokens[3]));
            if(!model) throw std::runtime_error("Unknown diode model: " + tokens[3]);
            if(!model->isDiode()) throw std::runtime_error("Model is not diode type: " + tokens[3]);
            circuit.addDevice<Diode>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, model, parse_spice_named_value(tokens, "area", 1.0));
            return true;
        }
        case 'Q': {
            if(tokens.size() < 5) throw std::runtime_error("Bad BJT line");
            const Model* model = circuit.findModel(to_lower_copy(tokens[4]));
            if(!model) throw std::runtime_error("Unknown BJT model: " + tokens[4]);
            if(!model->isBjt()) throw std::runtime_error("Model is not BJT type: " + tokens[4]);
            circuit.addDevice<BJT>(tokens[0], std::vector<std::string>{tokens[1], tokens[2], tokens[3]}, model, parse_spice_named_value(tokens, "area", 1.0));
            return true;
        }
        case 'M': {
            if(tokens.size() < 6) throw std::runtime_error("Bad MOSFET line");
            const Model* model = circuit.findModel(to_lower_copy(tokens[5]));
            if(!model) throw std::runtime_error("Unknown MOSFET model: " + tokens[5]);
            if(!model->isMosfet()) throw std::runtime_error("Model is not MOSFET type: " + tokens[5]);
            circuit.addDevice<MOSFET>(
                tokens[0],
                std::vector<std::string>{tokens[1], tokens[2], tokens[3], tokens[4]},
                model,
                parse_spice_named_value(tokens, "w", 1.0),
                parse_spice_named_value(tokens, "l", 1.0)
            );
            return true;
        }
        default:
            std::cerr << "Unsupported element: " << tokens[0] << std::endl;
            return false;
    }
}
