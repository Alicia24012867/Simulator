#pragma once

#include <optional>
#include <string>
#include <vector>

#include "analysis/ptaAnalysis.h"
#include "analysis/transientAnalysis.h"

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

// Parameters accepted by the .pstran pseudo-transient control card.  The
// solver has direct equivalents for convval, the step limits, CEPTA tau,
// source ramping, and the BJT junction-voltage seed.  kvgs0 remains retained
// for syntax compatibility until its model-specific scaling rule is known.
struct PstranAnalysisConfig {
    double convergenceValue = 1.0;
    double initialStep = 1.0e-9;
    double minimumStep = 1.0e-15;
    double maximumStep = 1.0e3;
    double tau = 0.0;
    double vbe0 = 0.0;
    double kvgs0 = 0.0;
    double tauRamp = 0.0;

    PtaAnalysisConfig makePtaConfig() const {
        PtaAnalysisConfig config;
        config.mode = PtaMode::Force;
        config.initialStep = initialStep;
        config.minimumStep = minimumStep;
        config.maximumStep = maximumStep;
        config.derivativeTolerance = convergenceValue;
        config.dcResidualTolerance = convergenceValue;
        config.compoundTimeConstant = tau;
        config.sourceRampTime = tauRamp;
        config.initialBjtVbe = vbe0;
        return config;
    }
};

// Analysis requests found in one netlist. More analysis types can be added here
// without changing Parser's public result interface.
struct AnalysisPlan {
    bool operatingPointRequested = false;
    bool operatingPointPrintRequested = false;
    bool transientPrintRequested = false;
    std::optional<TransientAnalysisConfig> transient;
    // HSPICE-compatible .option(s) DELMAX.  Once a .tran card is present it
    // is folded into TransientAnalysisConfig::maximumStep as a hard cap.
    std::optional<double> delmax;
    std::optional<PstranAnalysisConfig> pseudoTransient;
    std::vector<PrintVariable> operatingPointPrints;
    std::vector<PrintVariable> transientPrints;
};
