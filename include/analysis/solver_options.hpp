#pragma once

#include <cmath>

struct NewtonSolverOptions {
    int maximumIterations = 1000;
    // Legacy unified absolute tolerance.  New configuration should prefer the
    // voltage/current tolerances below; configuration and CLI users that set
    // this field still update both of them for backward compatibility.
    double tolerance = 1.0e-9;
    // The Newton update is normalized per unknown: node voltages use the
    // voltage absolute tolerance and branch currents use the current one.
    // Defaults reproduce the former strict 1 n raw-update gate.  Relative
    // tolerance is opt-in so existing OP/PTA behavior remains stable while
    // experiments can select physically scaled mixed-unit tolerances.
    double relativeTolerance = 0.0;
    double voltageAbsoluteTolerance = 1.0e-9;
    double currentAbsoluteTolerance = 1.0e-9;
    // Both normalized metrics must be below their respective limits before a
    // nonlinear solve is accepted.  A value of one corresponds to the usual
    // absolute-plus-relative tolerance test.
    double normalizedUpdateTolerance = 1.0;
    double normalizedResidualTolerance = 1.0;
    // Trial updates are backtracked until the reassembled nonlinear residual
    // satisfies an Armijo-style sufficient-decrease condition.
    int maximumBacktracks = 8;
    double backtrackScale = 0.5;
    double sufficientDecrease = 1.0e-4;
    double maximumSolutionStep = 1.0;
    // Ordinary OP/TRAN may make limited progress after an exhausted Armijo
    // search only when the full limited step remains finite and its residual
    // is bounded.  Set this to zero to retain strictly monotone searches.
    int maximumConsecutiveNonMonotoneSteps = 1;
    double maximumNonMonotoneResidualGrowth = 4.0;

    bool valid() const noexcept {
        return maximumIterations > 0 &&
            std::isfinite(tolerance) && tolerance > 0.0 &&
            std::isfinite(relativeTolerance) && relativeTolerance >= 0.0 &&
            std::isfinite(voltageAbsoluteTolerance) &&
            voltageAbsoluteTolerance > 0.0 &&
            std::isfinite(currentAbsoluteTolerance) &&
            currentAbsoluteTolerance > 0.0 &&
            std::isfinite(normalizedUpdateTolerance) &&
            normalizedUpdateTolerance > 0.0 &&
            std::isfinite(normalizedResidualTolerance) &&
            normalizedResidualTolerance > 0.0 &&
            maximumBacktracks >= 0 &&
            std::isfinite(backtrackScale) &&
            backtrackScale > 0.0 && backtrackScale < 1.0 &&
            std::isfinite(sufficientDecrease) &&
            sufficientDecrease > 0.0 && sufficientDecrease < 1.0 &&
            std::isfinite(maximumSolutionStep) &&
            maximumSolutionStep > 0.0 &&
            maximumConsecutiveNonMonotoneSteps >= 0 &&
            std::isfinite(maximumNonMonotoneResidualGrowth) &&
            maximumNonMonotoneResidualGrowth >= 1.0;
    }
};

struct SourceSteppingOptions {
    bool enabled = true;
    double initialStep = 0.1;
    double maximumStep = 0.25;
    double minimumStep = 1.0e-4;
    double growthFactor = 1.5;
    double failureScale = 0.5;

    bool valid() const noexcept {
        return std::isfinite(initialStep) && initialStep > 0.0 &&
            std::isfinite(maximumStep) &&
            maximumStep >= initialStep && maximumStep <= 1.0 &&
            std::isfinite(minimumStep) &&
            minimumStep > 0.0 && minimumStep <= initialStep &&
            std::isfinite(growthFactor) && growthFactor > 1.0 &&
            std::isfinite(failureScale) &&
            failureScale > 0.0 && failureScale < 1.0;
    }
};

struct OperatingPointSolverOptions {
    NewtonSolverOptions newton;
    SourceSteppingOptions sourceStepping;

    bool valid() const noexcept {
        return newton.valid() && sourceStepping.valid();
    }
};
