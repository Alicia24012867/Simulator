#pragma once

enum class PtaMode{
    Disabled,   // normal
    Force,      // pta method is required explicitly
    Fallback    // pta called when NR failed
};

struct PtaAnalysisConfig{
    PtaMode mode = PtaMode::Disabled;

    // time control
    double initialStep;
    double minimumStep;
    double maximumStep;
    int maximumSteps;

    // stability
    double derivativeTolerance;
    double dcResidualTolerance;

    // initial value & boundaries
    double initialNodeCapacitance;
    double minimumNodeCapacitance;
    double maximumNodeCapacitance;
    double currentSourceCapacitance;
    double voltageSourceInductance;

    // Adaptive rules
    double failedStepScale;         //used when NR failed
    double capacitanceGrowScale;    
    double smallOscillationScale;
    double mediumOscillationScale;
    double heavyOscillationScale;

    bool includeMosBulk = false;
    bool includeDiodes = false;

    void validate() const;
};