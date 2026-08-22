#pragma once

#include <cmath>

struct NewtonSolverOptions {
    int maximumIterations = 1000;
    double tolerance = 1.0e-9;
    double maximumSolutionStep = 1.0;

    bool valid() const noexcept {
        return maximumIterations > 0 &&
            std::isfinite(tolerance) && tolerance > 0.0 &&
            std::isfinite(maximumSolutionStep) &&
            maximumSolutionStep > 0.0;
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
