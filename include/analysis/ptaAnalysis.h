#pragma once

#include <cmath>
#include <stdexcept>

enum class PtaMode{
    Disabled,   // normal NR 
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

    void validate() const {
        switch(mode){
            case PtaMode::Disabled:
            case PtaMode::Force:
            case PtaMode::Fallback:
                break;
            default:
                throw std::invalid_argument("PTA mode is invalid");
        }

        const auto isPositiveFinite = [](double value) {
            return std::isfinite(value) && value > 0.0;
        };

        if(!isPositiveFinite(initialStep) ||
           !isPositiveFinite(minimumStep) ||
           !isPositiveFinite(maximumStep) ||
           minimumStep > initialStep ||
           initialStep > maximumStep){
            throw std::invalid_argument(
                "PTA time steps must satisfy 0 < minimum <= initial <= maximum"
            );
        }

        if(maximumSteps <= 0){
            throw std::invalid_argument(
                "PTA maximum step count must be positive"
            );
        }

        if(!isPositiveFinite(derivativeTolerance) ||
           !isPositiveFinite(dcResidualTolerance)){
            throw std::invalid_argument(
                "PTA convergence tolerances must be positive and finite"
            );
        }

        if(!isPositiveFinite(initialNodeCapacitance) ||
           !isPositiveFinite(minimumNodeCapacitance) ||
           !isPositiveFinite(maximumNodeCapacitance) ||
           minimumNodeCapacitance > initialNodeCapacitance ||
           initialNodeCapacitance > maximumNodeCapacitance){
            throw std::invalid_argument(
                "PTA node capacitances must satisfy 0 < minimum <= initial <= maximum"
            );
        }

        if(!isPositiveFinite(currentSourceCapacitance) ||
           !isPositiveFinite(voltageSourceInductance)){
            throw std::invalid_argument(
                "PTA source pseudo-element values must be positive and finite"
            );
        }

        if(!std::isfinite(failedStepScale) ||
           failedStepScale <= 0.0 || failedStepScale >= 1.0){
            throw std::invalid_argument(
                "PTA failed-step scale must be finite and in (0, 1)"
            );
        }

        if(!std::isfinite(capacitanceGrowScale) ||
           capacitanceGrowScale <= 1.0){
            throw std::invalid_argument(
                "PTA capacitance growth scale must be finite and greater than 1"
            );
        }

        if(!std::isfinite(smallOscillationScale) ||
           !std::isfinite(mediumOscillationScale) ||
           !std::isfinite(heavyOscillationScale) ||
           heavyOscillationScale <= 0.0 ||
           heavyOscillationScale >= mediumOscillationScale ||
           mediumOscillationScale >= smallOscillationScale ||
           smallOscillationScale >= 1.0){
            throw std::invalid_argument(
                "PTA oscillation scales must satisfy 0 < heavy < medium < small < 1"
            );
        }
    }
};
