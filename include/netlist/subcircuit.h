#pragma once

#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "netlist/source.h"

struct FlattenedNetlistElement {
    std::size_t lineNumber = 0;
    std::vector<std::string> tokens;
};

class SubcircuitError : public std::runtime_error {
public:
    SubcircuitError(std::size_t lineNumber, std::string message);

    std::size_t lineNumber() const noexcept {
        return lineNumber_;
    }

private:
    std::size_t lineNumber_;
};

// Stores `.subckt` definitions and expands X instances into primitive element
// statements.  It deliberately does not construct devices: Parser remains the
// sole owner of model resolution and primitive-device construction.
class SubcircuitLibrary {
public:
    using ElementVisitor = std::function<void(FlattenedNetlistElement&&)>;

    // Consumes all lexical lines, retaining subcircuit definitions internally
    // and returning only top-level statements in their original order.
    std::vector<LogicalNetlistLine> collect(
        std::vector<LogicalNetlistLine> lines
    );

    // Recursively expands an X statement.  Each resulting primitive is sent
    // to visitor with the source line that produced it.
    void expandInstance(const LogicalNetlistLine& instance,
                        const ElementVisitor& visitor) const;

private:
    struct Definition {
        std::size_t lineNumber = 0;
        std::string name;
        std::vector<std::string> pins;
        // Canonical pin name -> positional index.  Building it once per
        // definition avoids an allocation-heavy bindings map per instance.
        std::unordered_map<std::string, std::size_t> pinIndex;
        std::vector<LogicalNetlistLine> body;
    };

    void expand(const std::vector<std::string>& instanceTokens,
                const std::string& instancePath,
                std::size_t sourceLine,
                const ElementVisitor& visitor) const;

    std::unordered_map<std::string, Definition> definitions_;
};
