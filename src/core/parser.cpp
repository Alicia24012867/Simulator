#include <cctype>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

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
struct LogicalLine {
    std::size_t lineNumber = 0;
    std::string text;
    std::vector<std::string> tokens;
};

ModelType parseModelType(const std::string& token){
    const std::string type = to_lower_copy(token);
    if(type == "d") return ModelType::Diode;
    if(type == "npn") return ModelType::NPN;
    if(type == "pnp") return ModelType::PNP;
    if(type == "nmos" || type == "nch") return ModelType::NMOS;
    if(type == "pmos" || type == "pch") return ModelType::PMOS;
    return ModelType::Unknown;
}

bool supportsModelParameter(ModelType type, const std::string& key){
    if(type == ModelType::Diode){
        return key == "is" || key == "n" || key == "vt" ||
               key == "rs" || key == "gmin";
    }
    if(type == ModelType::NPN || type == ModelType::PNP){
        return key == "is" || key == "bf" || key == "beta" ||
               key == "br" || key == "nf" || key == "nr" ||
               key == "vt" || key == "gmin" || key == "rbe" ||
               key == "rce";
    }
    if(type == ModelType::NMOS || type == ModelType::PMOS){
        return key == "level" || key == "vto" || key == "vt0" ||
               key == "kp" || key == "k" || key == "lambda" ||
               key == "lam" || key == "gmin" || key == "rds";
    }
    return false;
}

void validateModelParameter(ModelType type,
                            const std::string& key,
                            double value){
    if(!supportsModelParameter(type, key)){
        throw std::runtime_error("Unsupported model parameter: " + key);
    }
    if(key == "level"){
        if(value != 1.0){
            throw std::runtime_error("Only MOSFET LEVEL=1 is supported");
        }
        return;
    }
    if(key == "vto" || key == "vt0"){
        return;
    }
    if(key == "rs" || key == "lambda" || key == "lam"){
        if(value < 0.0){
            throw std::runtime_error(
                "Model parameter " + key + " must be non-negative"
            );
        }
        return;
    }
    if(value <= 0.0){
        throw std::runtime_error(
            "Model parameter " + key + " must be positive"
        );
    }
}

void skipPrintSeparators(const std::string& text, std::size_t& pos){
    while(pos < text.size() &&
          (std::isspace(static_cast<unsigned char>(text[pos])) ||
           text[pos] == ',')){
        ++pos;
    }
}

bool startsWithWordIgnoreCase(const std::string& text,
                              std::size_t pos,
                              const std::string& word){
    if(pos + word.size() > text.size()){
        return false;
    }
    if(!equal_ignore_case(text.substr(pos, word.size()), word)){
        return false;
    }

    const std::size_t end = pos + word.size();
    return end == text.size() ||
           std::isspace(static_cast<unsigned char>(text[end])) ||
           text[end] == ',';
}

std::vector<std::string> parsePrintArguments(const std::string& expression){
    std::string normalized = expression;
    for(char& c: normalized){
        if(c == ','){
            c = ' ';
        }
    }

    std::istringstream iss(normalized);
    std::vector<std::string> arguments;
    std::string argument;
    while(iss >> argument){
        arguments.push_back(to_lower_copy(argument));
    }
    return arguments;
}

bool samePrintVariable(const PrintVariable& lhs, const PrintVariable& rhs){
    return lhs.quantity == rhs.quantity &&
           lhs.name == rhs.name &&
           lhs.reference == rhs.reference;
}

bool hasNonGroundNode(const std::vector<std::string>& tokens){
    if(tokens.empty() || tokens[0].empty()){
        return false;
    }

    std::size_t nodeCount = 0;
    switch(std::toupper(static_cast<unsigned char>(tokens[0][0]))){
        case 'Q': nodeCount = 3; break;
        case 'M': nodeCount = 4; break;
        case 'R':
        case 'C':
        case 'L':
        case 'V':
        case 'I':
        case 'D': nodeCount = 2; break;
        default: return false;
    }

    for(std::size_t i = 1; i <= nodeCount && i < tokens.size(); ++i){
        if(tokens[i] != "0" && !equal_ignore_case(tokens[i], "gnd")){
            return true;
        }
    }
    return false;
}

void appendUniquePrintVariable(std::vector<PrintVariable>& variables,
                               PrintVariable variable){
    for(const auto& existing: variables){
        if(samePrintVariable(existing, variable)){
            return;
        }
    }
    variables.push_back(std::move(variable));
}

std::string canonicalOutputNode(std::string name){
    if(equal_ignore_case(name, "gnd")){
        return "0";
    }
    return to_lower_copy(std::move(name));
}

std::size_t sourceValueEnd(const std::vector<std::string>& tokens,
                           std::size_t first){
    if(first >= tokens.size()){
        throw std::runtime_error("Missing source value");
    }
    if(equal_ignore_case(tokens[first], "dc")){
        std::size_t valueIndex = first + 1;
        if(valueIndex < tokens.size() && tokens[valueIndex] == "="){
            ++valueIndex;
        }
        if(valueIndex >= tokens.size()){
            throw std::runtime_error("Missing DC source value");
        }
        return valueIndex + 1;
    }
    if(equal_ignore_case(tokens[first], "dc=")){
        if(first + 1 >= tokens.size()){
            throw std::runtime_error("Missing DC source value");
        }
        return first + 2;
    }
    return first + 1;
}

double positiveElementValue(const std::vector<std::string>& tokens,
                            std::size_t index,
                            const char* elementName){
    const double value = parse_spice_number(tokens.at(index));
    if(value <= 0.0){
        throw std::runtime_error(
            std::string(elementName) + " value must be positive"
        );
    }
    return value;
}

struct InstanceValues {
    double area = 1.0;
    double width = 1.0;
    double length = 1.0;
};

InstanceValues parseInstanceValues(const std::vector<std::string>& tokens,
                                   std::size_t first,
                                   bool allowArea,
                                   bool allowGeometry,
                                   bool allowPositionalArea){
    InstanceValues values;
    bool areaSeen = false;
    bool widthSeen = false;
    bool lengthSeen = false;

    if(allowPositionalArea && first < tokens.size() &&
       tokens[first].find('=') == std::string::npos &&
       !(first + 1 < tokens.size() && !tokens[first + 1].empty() &&
         tokens[first + 1][0] == '=')){
        values.area = parse_spice_number(tokens[first++]);
        if(values.area <= 0.0){
            throw std::runtime_error("Instance parameter area must be positive");
        }
        areaSeen = true;
    }

    for(std::size_t i = first; i < tokens.size(); ++i){
        std::string key;
        std::string value;
        if(!read_spice_assignment(tokens, i, key, value)){
            throw std::runtime_error(
                "Unsupported or malformed instance parameter: " + tokens[i]
            );
        }

        const double parsed = parse_spice_number(value);
        if(parsed <= 0.0){
            throw std::runtime_error(
                "Instance parameter " + key + " must be positive"
            );
        }

        if(key == "area" && allowArea && !areaSeen){
            values.area = parsed;
            areaSeen = true;
        } else if(key == "w" && allowGeometry && !widthSeen){
            values.width = parsed;
            widthSeen = true;
        } else if(key == "l" && allowGeometry && !lengthSeen){
            values.length = parsed;
            lengthSeen = true;
        } else {
            throw std::runtime_error(
                "Unsupported or repeated instance parameter: " + key
            );
        }
    }
    return values;
}
}

bool Parser::parse(Circuit& circuit){
    std::ifstream file(filename_);
    if(!file){
        std::cerr << "Cannot open input netlist <" << filename_ << ">\n";
        return false;
    }

    analysisPlan_ = {};
    title_.clear();

    std::string line;
    if(!std::getline(file, line)){
        std::cerr << "Input netlist is empty <" << filename_ << ">\n";
        return false;
    }
    title_ = trim(line);
    if(title_.empty()){
        title_ = "Untitled circuit";
    }

    std::vector<LogicalLine> lines;
    std::size_t lineNumber = 1;
    bool endFound = false;
    while(std::getline(file, line)){
        ++lineNumber;

        line = trim(strip_spice_comment(line));
        if(line.empty() || line[0] == '*'){
            continue;
        }

        if(endFound){
            std::cerr << filename_ << ':' << lineNumber
                      << ": .end must be the last netlist statement\n";
            return false;
        }

        if(line[0] == '+'){
            if(lines.empty()){
                std::cerr << filename_ << ':' << lineNumber
                          << ": continuation line has no previous statement\n";
                return false;
            }
            lines.back().text += ' ' + trim(line.substr(1));
            lines.back().tokens = tokenize_spice_line(lines.back().text);
            continue;
        }

        std::vector<std::string> tokens = tokenize_spice_line(line);
        if(tokens.empty()){
            continue;
        }
        if(equal_ignore_case(tokens[0], ".end")){
            if(tokens.size() != 1){
                std::cerr << filename_ << ':' << lineNumber
                          << ": .end does not accept arguments\n";
                return false;
            }
            endFound = true;
            continue;
        }
        lines.push_back({lineNumber, line, std::move(tokens)});
    }

    if(file.bad()){
        std::cerr << "Failed while reading input netlist <" << filename_ << ">\n";
        return false;
    }
    if(!endFound){
        std::cerr << filename_ << ": missing required .end directive\n";
        return false;
    }

    std::size_t activeLine = 1;
    try {
        for(const auto& logicalLine: lines){
            activeLine = logicalLine.lineNumber;
            const auto& tokens = logicalLine.tokens;
            if(tokens[0][0] != '.'){
                continue;
            }

            if(equal_ignore_case(tokens[0], ".model")){
                continue;
            }
            if(equal_ignore_case(tokens[0], ".op") ||
               equal_ignore_case(tokens[0], ".tran")){
                parseAnalysisDirective(tokens);
                continue;
            }
            if(equal_ignore_case(tokens[0], ".print")){
                parsePrintDirective(logicalLine.text);
                continue;
            }
            if(equal_ignore_case(tokens[0], ".title")){
                const std::size_t titleStart = logicalLine.text.find_first_of(" \t");
                if(titleStart == std::string::npos ||
                   trim(logicalLine.text.substr(titleStart)).empty()){
                    throw std::runtime_error(".title requires text");
                }
                title_ = trim(logicalLine.text.substr(titleStart));
                continue;
            }

            throw std::runtime_error(
                "Unsupported control directive: " + tokens[0]
            );
        }

        if(!analysisPlan_.transient &&
           analysisPlan_.transientPrintRequested){
            throw std::runtime_error(".print tran requires a .tran analysis");
        }

        std::unordered_set<std::string> modelNames;
        for(const auto& logicalLine: lines){
            activeLine = logicalLine.lineNumber;
            const auto& tokens = logicalLine.tokens;
            if(equal_ignore_case(tokens[0], ".model")){
                if(tokens.size() < 2 ||
                   !modelNames.insert(to_lower_copy(tokens[1])).second){
                    throw std::runtime_error(
                        "Duplicate or missing model name in .model"
                    );
                }
                if(!parseModel(circuit, tokens)){
                    return false;
                }
            }
        }

        std::unordered_set<std::string> deviceNames;
        bool foundNonGroundNode = false;
        for(const auto& logicalLine: lines){
            activeLine = logicalLine.lineNumber;
            const auto& tokens = logicalLine.tokens;
            if(tokens[0][0] == '.'){
                continue;
            }
            if(!deviceNames.insert(to_lower_copy(tokens[0])).second){
                throw std::runtime_error(
                    "Duplicate device name: " + tokens[0]
                );
            }
            if(!parseLine(circuit, tokens)){
                return false;
            }
            foundNonGroundNode = foundNonGroundNode || hasNonGroundNode(tokens);
        }
        if(deviceNames.empty()){
            throw std::runtime_error("Netlist requires at least one element");
        }
        if(!foundNonGroundNode){
            throw std::runtime_error(
                "Netlist requires at least one non-ground node"
            );
        }
    } catch(const std::exception& ex){
        std::cerr << filename_ << ':' << activeLine
                  << ": parse error: " << ex.what() << '\n';
        return false;
    }

    return true;
}

bool Parser::parseAnalysisDirective(const std::vector<std::string>& tokens){
    if(equal_ignore_case(tokens[0], ".op")){
        if(tokens.size() != 1){
            throw std::runtime_error(".op does not accept arguments");
        }
        analysisPlan_.operatingPointRequested = true;
        return true;
    }

    if(!equal_ignore_case(tokens[0], ".tran")){
        return true;
    }

    if(analysisPlan_.transient){
        throw std::runtime_error("Multiple .tran directives are not supported");
    }
    if(tokens.size() < 3){
        throw std::runtime_error(".tran requires TSTEP and TSTOP");
    }

    TransientAnalysisConfig config;
    config.outputInterval = parse_spice_number(tokens[1]);
    config.stopTime = parse_spice_number(tokens[2]);

    if(config.outputInterval <= 0.0){
        throw std::runtime_error(".tran TSTEP must be positive");
    }
    if(config.stopTime <= 0.0){
        throw std::runtime_error(".tran TSTOP must be positive");
    }

    std::vector<double> optionalTimes;
    for(std::size_t i = 3; i < tokens.size(); ++i){
        if(equal_ignore_case(tokens[i], "uic")){
            if(config.useInitialConditions){
                throw std::runtime_error(".tran specifies UIC more than once");
            }
            config.useInitialConditions = true;
            continue;
        }
        optionalTimes.push_back(parse_spice_number(tokens[i]));
    }

    if(optionalTimes.size() > 2){
        throw std::runtime_error(".tran accepts at most TSTART and TMAX after TSTEP and TSTOP");
    }
    if(!optionalTimes.empty()){
        config.outputStartTime = optionalTimes[0];
        if(config.outputStartTime < 0.0 ||
           config.outputStartTime >= config.stopTime){
            throw std::runtime_error(".tran TSTART must be non-negative and smaller than TSTOP");
        }
    }
    if(optionalTimes.size() == 2){
        config.maximumStep = optionalTimes[1];
        if(*config.maximumStep <= 0.0){
            throw std::runtime_error(".tran TMAX must be positive");
        }
    }

    analysisPlan_.transient = config;
    return true;
}

void Parser::parsePrintDirective(const std::string& line){
    std::istringstream iss(line);
    std::string directive;
    std::string analysis;
    iss >> directive >> analysis;

    if(analysis.empty()){
        throw std::runtime_error(".print requires an analysis type");
    }

    std::vector<PrintVariable>* variables = nullptr;
    if(equal_ignore_case(analysis, "op")){
        analysisPlan_.operatingPointRequested = true;
        analysisPlan_.operatingPointPrintRequested = true;
        variables = &analysisPlan_.operatingPointPrints;
    } else if(equal_ignore_case(analysis, "tran")){
        analysisPlan_.transientPrintRequested = true;
        variables = &analysisPlan_.transientPrints;
    } else {
        throw std::runtime_error(
            "Unsupported .print analysis type: " + analysis
        );
    }

    std::string expressions;
    std::getline(iss, expressions);
    std::size_t pos = 0;
    bool foundVariable = false;

    while(true){
        skipPrintSeparators(expressions, pos);
        if(pos >= expressions.size()){
            break;
        }

        if(startsWithWordIgnoreCase(expressions, pos, "time")){
            if(!equal_ignore_case(analysis, "tran")){
                throw std::runtime_error("time is only valid for .print tran");
            }
            pos += 4;
            foundVariable = true;
            continue;
        }

        const char function = static_cast<char>(
            std::tolower(static_cast<unsigned char>(expressions[pos]))
        );
        if(function != 'v' && function != 'i'){
            throw std::runtime_error(
                "Expected v(node), v(node1,node2), or i(device) in .print"
            );
        }
        ++pos;
        while(pos < expressions.size() &&
              std::isspace(static_cast<unsigned char>(expressions[pos]))){
            ++pos;
        }
        if(pos >= expressions.size() || expressions[pos] != '('){
            throw std::runtime_error("Missing '(' in .print expression");
        }

        const std::size_t close = expressions.find(')', pos + 1);
        if(close == std::string::npos){
            throw std::runtime_error("Missing ')' in .print expression");
        }

        const auto arguments = parsePrintArguments(
            expressions.substr(pos + 1, close - pos - 1)
        );
        PrintVariable variable;
        if(function == 'v'){
            if(arguments.empty() || arguments.size() > 2){
                throw std::runtime_error(
                    "Voltage output requires one or two node names"
                );
            }
            variable.quantity = PrintQuantity::Voltage;
            variable.name = canonicalOutputNode(arguments[0]);
            variable.reference = arguments.size() == 2
                ? canonicalOutputNode(arguments[1])
                : "0";
        } else {
            if(arguments.size() != 1){
                throw std::runtime_error(
                    "Current output requires exactly one device name"
                );
            }
            variable.quantity = PrintQuantity::BranchCurrent;
            variable.name = arguments[0];
            variable.reference.clear();
        }

        appendUniquePrintVariable(*variables, std::move(variable));
        foundVariable = true;
        pos = close + 1;
    }

    if(!foundVariable){
        throw std::runtime_error(".print requires at least one output expression");
    }
}

bool Parser::parseModel(Circuit& circuit, const std::vector<std::string>& tokens){
    if(tokens.size() < 3){
        throw std::runtime_error(".model requires name and type");
    }

    const ModelType type = parseModelType(tokens[2]);
    if(type == ModelType::Unknown){
        throw std::runtime_error("Unsupported model type: " + tokens[2]);
    }

    auto model = std::make_unique<Model>(to_lower_copy(tokens[1]), type);

    for(std::size_t i = 3; i < tokens.size(); ++i){
        std::string key;
        std::string value;
        if(!read_spice_assignment(tokens, i, key, value)){
            throw std::runtime_error(
                "Malformed model parameter: " + tokens[i]
            );
        }
        const double parsed = parse_spice_number(value);
        validateModelParameter(type, key, parsed);
        model->setParam(key, parsed);
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
            if(tokens.size() != 4) throw std::runtime_error("Resistor requires exactly two nodes and one value");
            circuit.addDevice<Resistor>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, positiveElementValue(tokens, 3, "Resistor"));
            return true;
        case 'C':
            if(tokens.size() != 4) throw std::runtime_error("Capacitor requires exactly two nodes and one value");
            circuit.addDevice<Capacitor>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, positiveElementValue(tokens, 3, "Capacitor"));
            return true;
        case 'L':
            if(tokens.size() != 4) throw std::runtime_error("Inductor requires exactly two nodes and one value");
            circuit.addDevice<Inductor>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, positiveElementValue(tokens, 3, "Inductor"));
            return true;
        case 'V':
            if(tokens.size() < 4) throw std::runtime_error("Bad voltage source line");
            if(sourceValueEnd(tokens, 3) != tokens.size()) throw std::runtime_error("Only a single DC voltage-source value is supported");
            circuit.addDevice<VoltageSource>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, parse_spice_value_token(tokens, 3));
            return true;
        case 'I':
            if(tokens.size() < 4) throw std::runtime_error("Bad current source line");
            if(sourceValueEnd(tokens, 3) != tokens.size()) throw std::runtime_error("Only a single DC current-source value is supported");
            circuit.addDevice<CurrentSource>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, parse_spice_value_token(tokens, 3));
            return true;
        case 'D': {
            if(tokens.size() < 4) throw std::runtime_error("Bad diode line");
            const Model* model = circuit.findModel(to_lower_copy(tokens[3]));
            if(!model) throw std::runtime_error("Unknown diode model: " + tokens[3]);
            if(!model->isDiode()) throw std::runtime_error("Model is not diode type: " + tokens[3]);
            const InstanceValues values = parseInstanceValues(
                tokens,
                4,
                true,
                false,
                true
            );
            circuit.addDevice<Diode>(tokens[0], std::vector<std::string>{tokens[1], tokens[2]}, model, values.area);
            return true;
        }
        case 'Q': {
            if(tokens.size() < 5) throw std::runtime_error("Bad BJT line");
            const Model* model = circuit.findModel(to_lower_copy(tokens[4]));
            if(!model) throw std::runtime_error("Unknown BJT model: " + tokens[4]);
            if(!model->isBjt()) throw std::runtime_error("Model is not BJT type: " + tokens[4]);
            const InstanceValues values = parseInstanceValues(
                tokens,
                5,
                true,
                false,
                true
            );
            circuit.addDevice<BJT>(tokens[0], std::vector<std::string>{tokens[1], tokens[2], tokens[3]}, model, values.area);
            return true;
        }
        case 'M': {
            if(tokens.size() < 6) throw std::runtime_error("Bad MOSFET line");
            const Model* model = circuit.findModel(to_lower_copy(tokens[5]));
            if(!model) throw std::runtime_error("Unknown MOSFET model: " + tokens[5]);
            if(!model->isMosfet()) throw std::runtime_error("Model is not MOSFET type: " + tokens[5]);
            const InstanceValues values = parseInstanceValues(
                tokens,
                6,
                false,
                true,
                false
            );
            circuit.addDevice<MOSFET>(
                tokens[0],
                std::vector<std::string>{tokens[1], tokens[2], tokens[3], tokens[4]},
                model,
                values.width,
                values.length
            );
            return true;
        }
        default:
            throw std::runtime_error("Unsupported element: " + tokens[0]);
    }
}
