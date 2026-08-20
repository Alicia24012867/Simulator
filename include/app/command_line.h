#pragma once

#include <cstddef>
#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include "analysis/pta_analysis.h"

namespace simulator::app {

struct CommandLineOptions {
    std::string inputPath;
    std::optional<std::string> listingPath;
    std::optional<std::string> rawPath;
    std::optional<std::filesystem::path> configPath;
    std::size_t configSearchDepth = 8;
    bool printConfigPath = false;
    PtaMode ptaMode = PtaMode::Disabled;
    bool ptaModeSpecified = false;
    bool ptaDiagnostics = false;
    std::vector<std::string> operatingPointOptionAssignments;
    std::vector<std::string> ptaOptionAssignments;
    std::vector<std::string> transientOptionAssignments;
    bool parseOnly = false;
    bool helpRequested = false;
};

void printUsage(std::ostream& output, const char* program);

bool parseCommandLine(int argc,
                      char* argv[],
                      CommandLineOptions& options,
                      std::ostream& output,
                      std::ostream& error);

}  // namespace simulator::app
