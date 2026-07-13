#pragma once

#include <optional>
#include <string>
#include <vector>

enum class PrintQuantity {
    Voltage,
    BranchCurrent
};

// One SPICE .print expression. For a voltage, name is the positive node and
// reference is the optional negative node. For a current, name is the device.
struct PrintVariable {
    PrintQuantity quantity = PrintQuantity::Voltage;
    std::string name;
    std::string reference = "0";
};

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
    bool operatingPointPrintRequested = false;
    bool transientPrintRequested = false;
    std::optional<TransientAnalysisConfig> transient;
    std::vector<PrintVariable> operatingPointPrints;
    std::vector<PrintVariable> transientPrints;
};
