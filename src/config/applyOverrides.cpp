#include "config/applyOverrides.h"

#include <algorithm>
#include <stdexcept>

#include "analysis/ptaAnalysis.h"
#include "analysis/solverOptions.h"
#include "analysis/transientAnalysis.h"

namespace simulator::config {
namespace {

void applyNewtonOverrides(
    const NewtonOverrides& overrides,
    NewtonSolverOptions& options
){
    if(overrides.maximumIterations){
        options.maximumIterations = *overrides.maximumIterations;
    }
    if(overrides.tolerance){
        options.tolerance = *overrides.tolerance;
    }
    if(overrides.maximumSolutionStep){
        options.maximumSolutionStep = *overrides.maximumSolutionStep;
    }
}

void applySourceSteppingOverrides(
    const SourceSteppingOverrides& overrides,
    SourceSteppingOptions& options
){
    if(overrides.enabled){
        options.enabled = *overrides.enabled;
    }
    if(overrides.initialStep){
        options.initialStep = *overrides.initialStep;
    }
    if(overrides.maximumStep){
        options.maximumStep = *overrides.maximumStep;
    }
    if(overrides.minimumStep){
        options.minimumStep = *overrides.minimumStep;
    }
    if(overrides.growthFactor){
        options.growthFactor = *overrides.growthFactor;
    }
    if(overrides.failureScale){
        options.failureScale = *overrides.failureScale;
    }
}

PtaMode toPtaMode(PtaModeOverride mode){
    switch(mode){
        case PtaModeOverride::Disabled:
            return PtaMode::Disabled;
        case PtaModeOverride::Force:
            return PtaMode::Force;
        case PtaModeOverride::Fallback:
            return PtaMode::Fallback;
    }

    throw std::invalid_argument("PTA override mode is invalid");
}

void applyPtaOverrides(
    const PtaOverrides& overrides,
    PtaAnalysisConfig& config,
    bool pstranForcesPtaMode
){
    if(overrides.mode){
        const PtaMode mode = toPtaMode(*overrides.mode);
        if(pstranForcesPtaMode && mode != PtaMode::Force){
            throw std::invalid_argument(
                ".pstran cannot be combined with a non-force PTA mode"
            );
        }
        config.mode = mode;
    }
    if(overrides.newton){
        applyNewtonOverrides(*overrides.newton, config.newtonOptions);
    }

    if(overrides.initialStep) config.initialStep = *overrides.initialStep;
    if(overrides.minimumStep) config.minimumStep = *overrides.minimumStep;
    if(overrides.maximumStep) config.maximumStep = *overrides.maximumStep;
    if(overrides.maximumSteps) config.maximumSteps = *overrides.maximumSteps;

    if(overrides.derivativeTolerance){
        config.derivativeTolerance = *overrides.derivativeTolerance;
    }
    if(overrides.derivativeRelativeTolerance){
        config.derivativeRelativeTolerance =
            *overrides.derivativeRelativeTolerance;
    }
    if(overrides.derivativeVoltageAbsoluteTolerance){
        config.derivativeVoltageAbsoluteTolerance =
            *overrides.derivativeVoltageAbsoluteTolerance;
    }
    if(overrides.derivativeCurrentAbsoluteTolerance){
        config.derivativeCurrentAbsoluteTolerance =
            *overrides.derivativeCurrentAbsoluteTolerance;
    }

    if(overrides.dcResidualTolerance){
        config.dcResidualTolerance = *overrides.dcResidualTolerance;
    }
    if(overrides.dcResidualRelativeTolerance){
        config.dcResidualRelativeTolerance =
            *overrides.dcResidualRelativeTolerance;
    }
    if(overrides.dcVoltageAbsoluteTolerance){
        config.dcVoltageAbsoluteTolerance =
            *overrides.dcVoltageAbsoluteTolerance;
    }
    if(overrides.dcCurrentAbsoluteTolerance){
        config.dcCurrentAbsoluteTolerance =
            *overrides.dcCurrentAbsoluteTolerance;
    }

    if(overrides.initialNodeCapacitance){
        config.initialNodeCapacitance = *overrides.initialNodeCapacitance;
    }
    if(overrides.minimumNodeCapacitance){
        config.minimumNodeCapacitance = *overrides.minimumNodeCapacitance;
    }
    if(overrides.maximumNodeCapacitance){
        config.maximumNodeCapacitance = *overrides.maximumNodeCapacitance;
    }
    if(overrides.currentSourceCapacitance){
        config.currentSourceCapacitance = *overrides.currentSourceCapacitance;
    }
    if(overrides.voltageSourceInductance){
        config.voltageSourceInductance = *overrides.voltageSourceInductance;
    }

    if(overrides.compoundTimeConstant){
        config.compoundTimeConstant = *overrides.compoundTimeConstant;
    }
    if(overrides.compoundInitialResistance){
        config.compoundInitialResistance = *overrides.compoundInitialResistance;
    }
    if(overrides.compoundInitialConductance){
        config.compoundInitialConductance = *overrides.compoundInitialConductance;
    }
    if(overrides.sourceRampTime){
        config.sourceRampTime = *overrides.sourceRampTime;
    }
    if(overrides.initialBjtVbe.specified){
        config.initialBjtVbe = overrides.initialBjtVbe.value;
    }

    if(overrides.failedStepScale){
        config.failedStepScale = *overrides.failedStepScale;
    }
    if(overrides.successfulStepScale){
        config.successfulStepScale = *overrides.successfulStepScale;
    }
    if(overrides.capacitanceGrowScale){
        config.capacitanceGrowScale = *overrides.capacitanceGrowScale;
    }
    if(overrides.smallOscillationScale){
        config.smallOscillationScale = *overrides.smallOscillationScale;
    }
    if(overrides.mediumOscillationScale){
        config.mediumOscillationScale = *overrides.mediumOscillationScale;
    }
    if(overrides.heavyOscillationScale){
        config.heavyOscillationScale = *overrides.heavyOscillationScale;
    }
    if(overrides.mediumOscillationRatio){
        config.mediumOscillationRatio = *overrides.mediumOscillationRatio;
    }
    if(overrides.heavyOscillationRatio){
        config.heavyOscillationRatio = *overrides.heavyOscillationRatio;
    }

    if(overrides.includeMosBulk){
        config.includeMosBulk = *overrides.includeMosBulk;
    }
    if(overrides.includeDiodes){
        config.includeDiodes = *overrides.includeDiodes;
    }

    if(pstranForcesPtaMode){
        config.mode = PtaMode::Force;
    }
}

void applyTransientSolverOverrides(
    const TransientSolverOverrides& overrides,
    TransientSolverOptions& options
){
    if(overrides.newton){
        applyNewtonOverrides(*overrides.newton, options.newtonOptions);
    }
    if(overrides.relativeTolerance){
        options.relativeTolerance = *overrides.relativeTolerance;
    }
    if(overrides.voltageAbsoluteTolerance){
        options.voltageAbsoluteTolerance =
            *overrides.voltageAbsoluteTolerance;
    }
    if(overrides.currentAbsoluteTolerance){
        options.currentAbsoluteTolerance =
            *overrides.currentAbsoluteTolerance;
    }
    if(overrides.minimumStep){
        options.minimumStep = *overrides.minimumStep;
    }
    if(overrides.safetyFactor){
        options.safetyFactor = *overrides.safetyFactor;
    }
    if(overrides.minimumScale){
        options.minimumScale = *overrides.minimumScale;
    }
    if(overrides.maximumScale){
        options.maximumScale = *overrides.maximumScale;
    }
    if(overrides.convergenceFailureScale){
        options.convergenceFailureScale = *overrides.convergenceFailureScale;
    }
    if(overrides.maximumRejects){
        options.maximumRejects = *overrides.maximumRejects;
    }
}

void applyTransientOverrides(
    const TransientOverrides& overrides,
    std::optional<TransientAnalysisConfig>& transient
){
    if(overrides.enabled && !*overrides.enabled){
        transient.reset();
        return;
    }

    if(!transient){
        transient.emplace();
    }

    TransientAnalysisConfig& config = *transient;
    if(overrides.outputInterval){
        config.outputInterval = *overrides.outputInterval;
    }
    if(overrides.stopTime){
        config.stopTime = *overrides.stopTime;
    }
    if(overrides.outputStartTime){
        config.outputStartTime = *overrides.outputStartTime;
    }
    if(overrides.maximumStep){
        config.maximumStep = config.maximumStep
            ? std::min(*config.maximumStep, *overrides.maximumStep)
            : *overrides.maximumStep;
    }
    if(overrides.useInitialConditions){
        config.useInitialConditions = *overrides.useInitialConditions;
    }
    if(overrides.solver){
        applyTransientSolverOverrides(*overrides.solver, config.solverOptions);
    }
}

}  // namespace

void applyConfigOverrides(
    const ConfigOverrides& overrides,
    OperatingPointSolverOptions& operatingPoint,
    PtaAnalysisConfig& pta,
    std::optional<TransientAnalysisConfig>& transient,
    bool pstranForcesPtaMode
){
    if(overrides.operatingPoint){
        if(overrides.operatingPoint->newton){
            applyNewtonOverrides(
                *overrides.operatingPoint->newton,
                operatingPoint.newton
            );
        }
        if(overrides.operatingPoint->sourceStepping){
            applySourceSteppingOverrides(
                *overrides.operatingPoint->sourceStepping,
                operatingPoint.sourceStepping
            );
        }
    }

    if(overrides.pta){
        applyPtaOverrides(*overrides.pta, pta, pstranForcesPtaMode);
    }

    if(overrides.transient){
        applyTransientOverrides(*overrides.transient, transient);
    }
}

}  // namespace simulator::config
