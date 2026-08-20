#pragma once

#include <string>
#include <vector>

struct DeviceTypeDiagnostics {
    std::string type;
    int count = 0;
};

struct CircuitDiagnostics {
    int deviceCount = 0;
    int nonlinearDeviceCount = 0;
    int modelCount = 0;
    int nodeCount = 0;
    int unknownCount = 0;
    int matrixNonZeros = 0;
    int pseudoDeviceCount = 0;
    std::vector<DeviceTypeDiagnostics> devicesByType;

    bool hasFiniteSolution = false;
    double minimumNodeVoltage = 0.0;
    double maximumNodeVoltage = 0.0;
    double maximumAbsoluteSolution = 0.0;
    std::string maximumAbsoluteSolutionVariable;
    double maximumAbsoluteBranchCurrent = 0.0;
    std::string maximumAbsoluteBranchCurrentDevice;
};

// Diagnostics are deliberately plain data objects.  The solver owns and
// populates them; presentation layers can serialize them without reaching
// into Circuit internals or parsing stderr.
struct NewtonSolveDiagnostics {
    bool attempted = false;
    bool converged = false;
    bool usedNewtonRaphson = false;
    int iterations = 0;
    int maximumIterations = 0;
    int dampedSteps = 0;
    double finalDelta = 0.0;
    double tolerance = 0.0;
    double cpuSeconds = 0.0;
    double wallSeconds = 0.0;
    std::string failureReason;
};

struct SourceSteppingAttemptDiagnostics {
    int attempt = 0;
    double acceptedScaleBefore = 0.0;
    // Actual source-scale increment after clamping the target to 1.0.
    double stepSize = 0.0;
    double targetScale = 0.0;
    bool accepted = false;
    std::string status;
    NewtonSolveDiagnostics newton;
};

struct PtaStepAttemptDiagnostics {
    int attempt = 0;
    double startTime = 0.0;
    double targetTime = 0.0;
    double timeStep = 0.0;
    double sourceScale = 0.0;
    int integrationOrder = 1;
    bool accepted = false;
    bool reachedSteadyState = false;
    bool retriedWithSmallerStep = false;
    bool restartedAfterCapacitanceGrowth = false;
    bool hasConvergenceMetrics = false;
    double normalizedDerivative = 0.0;
    double normalizedDcResidual = 0.0;
    int capacitanceReductions = 0;
    std::string status;
    std::string failureReason;
    NewtonSolveDiagnostics newton;
};

struct OperatingPointDiagnostics {
    bool attempted = false;
    bool converged = false;
    std::string finalMethod;
    std::string failureReason;

    // Totals cover the complete method chain.  In fallback mode this means
    // direct NR, source stepping, and PTA rather than only the final stage.
    int iterations = 0;
    int maxIterations = 0;
    int dampedSteps = 0;
    double finalDelta = 0.0;
    double tolerance = 0.0;
    double cpuSeconds = 0.0;
    double wallSeconds = 0.0;

    NewtonSolveDiagnostics directNewton;

    int sourceSteps = 0;
    int failedSourceSteps = 0;
    double sourceScale = 0.0;
    double minSourceStep = 0.0;
    std::vector<SourceSteppingAttemptDiagnostics> sourceAttempts;

    bool ptaAttempted = false;
    int ptaIterations = 0;
    int ptaDampedSteps = 0;
    int ptaAcceptedSteps = 0;
    int ptaRejectedSteps = 0;
    int ptaCapacitanceGrowths = 0;
    int ptaCapacitanceReductions = 0;
    int ptaMinimumStepRecoveries = 0;
    double ptaFinalTime = 0.0;
    double ptaFinalStep = 0.0;
    double ptaMinimumAttemptedStep = 0.0;
    double ptaMaximumAttemptedStep = 0.0;
    double ptaFinalSourceScale = 0.0;
    double ptaCpuSeconds = 0.0;
    double ptaWallSeconds = 0.0;
    bool hasPtaConvergenceMetrics = false;
    double ptaNormalizedDerivative = 0.0;
    double ptaNormalizedDcResidual = 0.0;
    std::vector<PtaStepAttemptDiagnostics> ptaAttempts;
};

struct TransientDiagnostics {
    bool attempted = false;
    bool converged = false;
    bool usedInitialConditions = false;
    std::string finalMethod;
    std::string failureReason;

    int timeSteps = 0;
    int outputPoints = 0;
    int iterations = 0;
    int maxIterations = 0;
    int dampedSteps = 0;
    double finalTime = 0.0;
    double finalDelta = 0.0;
    double tolerance = 0.0;
    double initializationCpuSeconds = 0.0;
    double initializationWallSeconds = 0.0;
    double cpuSeconds = 0.0;
    double wallSeconds = 0.0;

    int attemptedSteps = 0;
    int backwardEulerAttempts = 0;
    int bdf2Attempts = 0;
    int rejectedSteps = 0;
    int convergenceRejectedSteps = 0;
    int errorRejectedSteps = 0;
    int invalidEstimateFailures = 0;

    double minimumAttemptedStep = 0.0;
    double maximumAttemptedStep = 0.0;
    double minimumAcceptedStep = 0.0;
    double maximumAcceptedStep = 0.0;
    double lastAttemptedStep = 0.0;
    double lastAttemptedTime = 0.0;
    int lastIntegrationOrder = 0;
    bool hasNormalizedError = false;
    double lastNormalizedError = 0.0;
    double maximumNormalizedError = 0.0;
    NewtonSolveDiagnostics lastNewton;
};
