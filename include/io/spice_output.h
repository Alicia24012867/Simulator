#pragma once

#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>

class Circuit;
struct AnalysisPlan;

class SpiceOutputWriter {
public:
    static void writeOperatingPoint(std::ostream& os,
                                    const Circuit& circuit,
                                    const std::string& title,
                                    const AnalysisPlan& plan);

    static void writeTransient(std::ostream& os,
                               const Circuit& circuit,
                               const std::string& title,
                               const AnalysisPlan& plan);

    static void writePtaDiagnostics(std::ostream& os,
                                    const Circuit& circuit);
};

class SpiceRawWriter {
public:
    static void writeOperatingPoint(std::ostream& os,
                                    const Circuit& circuit,
                                    const std::string& title);

    static void writeTransient(std::ostream& os,
                               const Circuit& circuit,
                               const std::string& title);
};

class SpiceOutputFiles {
public:
    static bool validatePaths(
        const std::string& inputPath,
        const std::optional<std::string>& listingPath,
        const std::optional<std::string>& rawPath,
        std::ostream& error
    );

    static bool writeAtomically(
        const std::optional<std::string>& listingPath,
        std::string_view listing,
        const std::optional<std::string>& rawPath,
        std::string_view raw,
        std::ostream& error
    );
};
