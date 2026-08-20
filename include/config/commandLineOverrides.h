#pragma once

#include <optional>
#include <string>

#include "analysis/ptaAnalysis.h"
#include "analysis/solverOptions.h"
#include "analysis/transientAnalysis.h"
#include "config/netlistParameterLocks.h"

namespace simulator::config {

bool applyOperatingPointOption(
    const std::string& assignment,
    OperatingPointSolverOptions& options,
    std::string& key,
    std::string& error
);

bool applyPtaOption(
    const std::string& assignment,
    PtaAnalysisConfig& options,
    std::string& key,
    std::string& error,
    const PtaParameterLocks& netlistLocks = {}
);

bool applyTransientOption(
    const std::string& assignment,
    std::optional<TransientAnalysisConfig>& options,
    std::string& key,
    std::string& error,
    const std::optional<TransientAnalysisConfig>& baseOptions = std::nullopt,
    std::optional<double> hardMaximumStep = std::nullopt,
    const TransientParameterLocks& netlistLocks = {}
);

}  // namespace simulator::config
