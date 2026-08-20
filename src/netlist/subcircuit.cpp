#include "netlist/subcircuit.h"

#include <cctype>
#include <utility>

#include "utils/string_utils.hpp"

namespace {
bool isSubcircuitInstance(const std::vector<std::string>& tokens) {
    return !tokens.empty() && !tokens.front().empty() &&
           std::toupper(static_cast<unsigned char>(tokens.front().front())) ==
               'X';
}

std::size_t elementNodeCount(const std::vector<std::string>& tokens,
                             std::size_t lineNumber) {
    if(tokens.empty() || tokens.front().empty()) {
        throw SubcircuitError(lineNumber, "Empty element statement");
    }

    switch(std::toupper(static_cast<unsigned char>(tokens.front().front()))) {
        case 'Q': return 3;
        case 'M': return 4;
        case 'R':
        case 'C':
        case 'L':
        case 'V':
        case 'I':
        case 'D': return 2;
        case 'X':
            if(tokens.size() < 3) {
                throw SubcircuitError(
                    lineNumber,
                    "Subcircuit instance requires nodes and a subcircuit name"
                );
            }
            return tokens.size() - 2;
        default:
            throw SubcircuitError(
                lineNumber,
                "Unsupported element: " + tokens.front()
            );
    }
}

bool isGroundNode(const std::string& canonicalName) {
    return canonicalName == "0" || canonicalName == "gnd";
}
}

SubcircuitError::SubcircuitError(std::size_t lineNumber, std::string message):
    std::runtime_error(std::move(message)),
    lineNumber_(lineNumber) {}

std::vector<LogicalNetlistLine> SubcircuitLibrary::collect(
    std::vector<LogicalNetlistLine> lines)
{
    definitions_.clear();
    definitions_.reserve(lines.size());

    std::vector<LogicalNetlistLine> topLevelLines;
    topLevelLines.reserve(lines.size());

    Definition currentDefinition;
    bool insideDefinition = false;

    for(auto& logicalLine: lines) {
        const auto& tokens = logicalLine.tokens;
        if(tokens.empty()) {
            continue;
        }

        if(equal_ignore_case(tokens.front(), ".subckt")) {
            if(insideDefinition) {
                throw SubcircuitError(
                    logicalLine.lineNumber,
                    "Nested .subckt definitions are not supported"
                );
            }
            if(tokens.size() < 3) {
                throw SubcircuitError(
                    logicalLine.lineNumber,
                    ".subckt requires a name and at least one pin"
                );
            }

            currentDefinition = {};
            currentDefinition.lineNumber = logicalLine.lineNumber;
            currentDefinition.name = to_lower_copy(tokens[1]);
            currentDefinition.pins.reserve(tokens.size() - 2);
            currentDefinition.pinIndex.reserve(tokens.size() - 2);
            for(std::size_t i = 2; i < tokens.size(); ++i) {
                const std::string pin = to_lower_copy(tokens[i]);
                if(isGroundNode(pin) ||
                   !currentDefinition.pinIndex.emplace(
                       pin,
                       currentDefinition.pins.size()
                   ).second) {
                    throw SubcircuitError(
                        logicalLine.lineNumber,
                        ".subckt pins must be unique non-ground nodes"
                    );
                }
                currentDefinition.pins.push_back(pin);
            }
            insideDefinition = true;
            continue;
        }

        if(equal_ignore_case(tokens.front(), ".ends")) {
            if(!insideDefinition) {
                throw SubcircuitError(
                    logicalLine.lineNumber,
                    ".ends has no matching .subckt"
                );
            }
            if(tokens.size() > 2 ||
               (tokens.size() == 2 &&
                !equal_ignore_case(tokens[1], currentDefinition.name))) {
                throw SubcircuitError(
                    logicalLine.lineNumber,
                    ".ends name does not match its .subckt definition"
                );
            }

            const std::string name = currentDefinition.name;
            if(!definitions_.emplace(name, std::move(currentDefinition)).second) {
                throw SubcircuitError(
                    logicalLine.lineNumber,
                    "Duplicate .subckt definition: " + name
                );
            }
            insideDefinition = false;
            continue;
        }

        if(insideDefinition) {
            currentDefinition.body.push_back(std::move(logicalLine));
        } else {
            topLevelLines.push_back(std::move(logicalLine));
        }
    }

    if(insideDefinition) {
        throw SubcircuitError(
            currentDefinition.lineNumber,
            ".subckt is missing a matching .ends"
        );
    }

    return topLevelLines;
}

void SubcircuitLibrary::expandInstance(const LogicalNetlistLine& instance,
                                       const ElementVisitor& visitor) const {
    if(!isSubcircuitInstance(instance.tokens)) {
        throw SubcircuitError(
            instance.lineNumber,
            "Expected a subcircuit instance"
        );
    }
    expand(
        instance.tokens,
        to_lower_copy(instance.tokens.front()),
        instance.lineNumber,
        visitor
    );
}

void SubcircuitLibrary::expand(const std::vector<std::string>& instanceTokens,
                               const std::string& instancePath,
                               std::size_t sourceLine,
                               const ElementVisitor& visitor) const {
    if(instanceTokens.size() < 3) {
        throw SubcircuitError(
            sourceLine,
            "Subcircuit instance requires nodes and a subcircuit name"
        );
    }

    const std::string subcircuitName = to_lower_copy(instanceTokens.back());
    const auto definitionIt = definitions_.find(subcircuitName);
    if(definitionIt == definitions_.end()) {
        throw SubcircuitError(
            sourceLine,
            "Unknown subcircuit: " + instanceTokens.back()
        );
    }

    const Definition& definition = definitionIt->second;
    const std::size_t actualNodeCount = instanceTokens.size() - 2;
    if(actualNodeCount != definition.pins.size()) {
        throw SubcircuitError(
            sourceLine,
            "Subcircuit " + instanceTokens.back() + " expects " +
            std::to_string(definition.pins.size()) + " nodes but received " +
            std::to_string(actualNodeCount)
        );
    }

    for(const auto& bodyLine: definition.body) {
        if(bodyLine.tokens.empty()) {
            continue;
        }
        if(bodyLine.tokens.front().front() == '.') {
            throw SubcircuitError(
                bodyLine.lineNumber,
                "Control directives inside .subckt are not supported: " +
                bodyLine.tokens.front()
            );
        }

        std::vector<std::string> flattened = bodyLine.tokens;
        const std::size_t nodeCount = elementNodeCount(
            flattened,
            bodyLine.lineNumber
        );
        if(flattened.size() <= nodeCount) {
            throw SubcircuitError(
                bodyLine.lineNumber,
                "Malformed element in .subckt: " + flattened.front()
            );
        }

        for(std::size_t i = 1; i <= nodeCount; ++i) {
            const std::string node = to_lower_copy(flattened[i]);
            if(isGroundNode(node)) {
                flattened[i] = "0";
                continue;
            }

            const auto pin = definition.pinIndex.find(node);
            flattened[i] = pin == definition.pinIndex.end()
                ? instancePath + ":" + node
                : instanceTokens[pin->second + 1];
        }

        if(isSubcircuitInstance(flattened)) {
            expand(
                flattened,
                instancePath + "/" + to_lower_copy(flattened.front()),
                bodyLine.lineNumber,
                visitor
            );
        } else {
            // Keep the designator first because primitive parsing dispatches on
            // it; the hierarchy suffix makes flattened instance names unique.
            flattened.front() += "@" + instancePath;
            visitor({bodyLine.lineNumber, std::move(flattened)});
        }
    }
}
