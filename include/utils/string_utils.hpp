#pragma once
#include <cctype>
#include <cmath>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

inline std::string trim(const std::string& str){
    auto i = str.find_first_not_of(" \t\r\n");
    if(i == std::string::npos){
        return "";
    }

    auto j = str.find_last_not_of(" \t\r\n");
    if(j == std::string::npos){
        return "";
    }

    return str.substr(i, j-i+1);
}

inline std::string to_lower_copy(std::string s){
    for(char& c: s){
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

inline bool equal_ignore_case(const std::string& a, const std::string& b){
    if(a.size() != b.size()){
        return false;
    }

    for(std::size_t i = 0; i < a.size(); ++i){
        if(std::tolower(static_cast<unsigned char>(a[i])) !=
            std::tolower(static_cast<unsigned char>(b[i]))){
                return false;
            }
    }

    return true;
}

inline std::string strip_spice_comment(const std::string& line){
    std::size_t pos = line.find_first_of(";$");
    const std::size_t slashPos = line.find("//");
    if(slashPos != std::string::npos &&
       (pos == std::string::npos || slashPos < pos)){
        pos = slashPos;
    }
    return pos == std::string::npos ? line : line.substr(0, pos);
}

inline std::vector<std::string> tokenize_spice_line(const std::string& line){
    std::string normalized;
    normalized.reserve(line.size());

    for(char c: line){
        normalized.push_back((c == '(' || c == ')' || c == ',') ? ' ' : c);
    }

    std::istringstream iss(normalized);
    std::vector<std::string> tokens;
    std::string token;
    while(iss >> token){
        tokens.push_back(token);
    }
    return tokens;
}

inline double parse_spice_number(std::string text){
    static const std::regex spiceNumber(
        R"(^([+-]?(?:(?:[0-9]+(?:\.[0-9]*)?)|(?:\.[0-9]+))(?:[eE][+-]?[0-9]+)?)([A-Za-z]*)$)"
    );
    std::smatch match;
    if(!std::regex_match(text, match, spiceNumber)){
        throw std::runtime_error("Invalid SPICE number: " + text);
    }

    const double value = std::stod(match[1].str());
    const std::string suffix = to_lower_copy(match[2].str());

    auto scaled = [value](double multiplier){
        const double result = value * multiplier;
        if(!std::isfinite(result)){
            throw std::runtime_error("SPICE number must be finite");
        }
        return result;
    };

    if(suffix.empty()) return scaled(1.0);

    if(suffix.rfind("meg", 0) == 0) return scaled(1e6);
    if(suffix.rfind("mil", 0) == 0) return scaled(25.4e-6);

    switch(suffix[0]){
        case 'a': return scaled(1e-18);
        case 'f': return scaled(1e-15);
        case 'p': return scaled(1e-12);
        case 'n': return scaled(1e-9);
        case 'u': return scaled(1e-6);
        case 'm': return scaled(1e-3);
        case 'k': return scaled(1e3);
        case 'g': return scaled(1e9);
        case 't': return scaled(1e12);
        default: return scaled(1.0);
    }
}

inline bool read_spice_assignment(const std::vector<std::string>& tokens,
                                  std::size_t& i,
                                  std::string& key,
                                  std::string& value){
    const std::string& token = tokens[i];
    const std::size_t eq = token.find('=');

    if(eq != std::string::npos){
        key = to_lower_copy(token.substr(0, eq));
        value = token.substr(eq + 1);
        if(value.empty() && i + 1 < tokens.size()){
            value = tokens[++i];
        }
        return !key.empty() && !value.empty();
    }

    if(i + 1 < tokens.size() && tokens[i + 1] == "="){
        if(i + 2 >= tokens.size()){
            return false;
        }
        key = to_lower_copy(tokens[i]);
        value = tokens[i + 2];
        i += 2;
        return !key.empty() && !value.empty();
    }

    if(i + 1 < tokens.size() && tokens[i + 1].size() > 1 &&
       tokens[i + 1][0] == '='){
        key = to_lower_copy(tokens[i]);
        value = tokens[i + 1].substr(1);
        ++i;
        return !key.empty() && !value.empty();
    }

    return false;
}

inline double parse_spice_value_token(const std::vector<std::string>& tokens,
                                      std::size_t first){
    if(first >= tokens.size()){
        throw std::runtime_error("Missing value");
    }

    if(equal_ignore_case(tokens[first], "dc")){
        std::size_t valueIndex = first + 1;
        bool separatedEquals = false;
        if(valueIndex < tokens.size() && tokens[valueIndex] == "="){
            separatedEquals = true;
            ++valueIndex;
        }
        if(valueIndex >= tokens.size()){
            throw std::runtime_error("Missing DC value");
        }
        const std::string& value = tokens[valueIndex];
        return parse_spice_number(
            !separatedEquals && value.size() > 1 && value[0] == '='
                ? value.substr(1)
                : value
        );
    }

    const std::string dcPrefix = "dc=";
    const std::string token = to_lower_copy(tokens[first]);
    if(token == dcPrefix){
        if(first + 1 >= tokens.size()){
            throw std::runtime_error("Missing DC value");
        }
        return parse_spice_number(tokens[first + 1]);
    }
    if(token.rfind(dcPrefix, 0) == 0){
        return parse_spice_number(token.substr(dcPrefix.size()));
    }

    return parse_spice_number(tokens[first]);
}

inline double parse_spice_named_value(const std::vector<std::string>& tokens,
                                      const std::string& key,
                                      double fallback){
    const std::string wanted = to_lower_copy(key);
    for(std::size_t i = 0; i < tokens.size(); ++i){
        std::string found;
        std::string value;
        if(read_spice_assignment(tokens, i, found, value) && found == wanted){
            return parse_spice_number(value);
        }
    }
    return fallback;
}
