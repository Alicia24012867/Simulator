#pragma once

#include <optional>

#include "analysis/ptaAnalysis.h"
#include "analysis/solverOptions.h"
#include "analysis/transientAnalysis.h"
#include "config/overrides.h"

namespace simulator::config {

void applyConfigOverrides(
    const ConfigOverrides& overrides,
    OperatingPointSolverOptions& operatingPoint,
    PtaAnalysisConfig& pta,
    std::optional<TransientAnalysisConfig>& transient,
    bool pstranForcesPtaMode
);

}  // namespace simulator::config
