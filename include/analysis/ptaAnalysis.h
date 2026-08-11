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
    double initialStep = 1.0e-9;
    double minimumStep = 1.0e-15;
    double maximumStep = 1.0e3;
    int maximumSteps = 10000;

    // stability
    double derivativeTolerance = 1.0e-8;
    double dcResidualTolerance = 1.0e-9;

    // initial value & boundaries
    double initialNodeCapacitance = 1.0e-12;
    double minimumNodeCapacitance = 1.0e-18;
    double maximumNodeCapacitance = 1.0e-3;
    double currentSourceCapacitance = 1.0e-12;
    double voltageSourceInductance = 1.0e-9;

    // Adaptive rules
    double failedStepScale = 0.5;         // used when NR failed
    double successfulStepScale = 2.0;
    double capacitanceGrowScale = 2.0;
    double smallOscillationScale = 0.9;
    double mediumOscillationScale = 0.7;
    double heavyOscillationScale = 0.5;

    double mediumOscillationRatio = 0.5;
    double heavyOscillationRatio = 1.0;

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

        if(!std::isfinite(successfulStepScale) ||
           successfulStepScale <= 1.0){
            throw std::invalid_argument(
                "PTA successful-step scale must be finite and greater than 1"
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

        if(!std::isfinite(mediumOscillationRatio) ||
           !std::isfinite(heavyOscillationRatio) ||
           mediumOscillationRatio <= 0.0 ||
           mediumOscillationRatio >= heavyOscillationRatio){
            throw std::invalid_argument(
                "PTA oscillation ratios must satisfy 0 < medium < heavy"
            );
        }
    }
};
