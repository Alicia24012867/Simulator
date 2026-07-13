#pragma once

#include <optional>

// Parsed .tran parameters. This is declarative netlist data, not solver state.
struct TransientAnalysisConfig {
    double outputInterval = 0.0;  // TSTEP
    double stopTime = 0.0;        // TSTOP
    double outputStartTime = 0.0; // TSTART
    std::optional<double> maximumStep;  // TMAX
    bool useInitialConditions = false; // UIC
};

// Analysis requests found in one netlist. More analysis types can be added here
// without changing Parser's public result interface.
struct AnalysisPlan {
    bool operatingPointRequested = false;
    std::optional<TransientAnalysisConfig> transient;
};
