#include "circuit/circuit.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <limits>
#include <set>
#include <unordered_set>

#include "analysis/analysis_plan.h"
#include "analysis/solver_options.h"
#include "analysis/transient_analysis.h"
#include "circuit/node_map.h"
#include "devices/device.hpp"
#include "math/mna.hpp"
#include "math/newton_step.hpp"
#include "models/model.hpp"
#include "devices/pseudo_device.hpp"
#include "analysis/pta_analysis.h"

namespace {
constexpr double kSourceScaleDone = 1.0 - 1.0e-12;
constexpr double kTimeRelativeTolerance =
    64.0 * std::numeric_limits<double>::epsilon();

using SteadyClock = std::chrono::steady_clock;

double elapsedWallSeconds(SteadyClock::time_point start){
    return std::chrono::duration<double>(SteadyClock::now() - start).count();
}

void updateStepRange(double step, double& minimum, double& maximum){
    if(!std::isfinite(step) || step <= 0.0){
        return;
    }
    if(minimum == 0.0 || step < minimum){
        minimum = step;
    }
    maximum = std::max(maximum, step);
}

const char* transientFailureReason(TransientStepAction action){
    switch(action){
        case TransientStepAction::FailInvalidEstimate:
            return "transient error estimate is invalid";
        case TransientStepAction::FailMinimumStep:
            return "transient step cannot be reduced above the minimum step";
        case TransientStepAction::FailRejectLimit:
            return "transient step rejection limit was reached";
        default:
            return "transient step controller failed";
    }
}

bool timeReached(double time, double target){
    const double scale = std::max(
        std::abs(time),
        std::abs(target)
    );
    return time >= target - kTimeRelativeTolerance * scale;
}

bool sameTime(double lhs, double rhs){
    const double scale = std::max(std::abs(lhs), std::abs(rhs));
    return std::abs(lhs - rhs) <= kTimeRelativeTolerance * scale;
}

bool advanceOutputTime(double& nextOutputTime,
                       double currentTime,
                       double outputInterval){
    do {
        const double previousOutputTime = nextOutputTime;
        nextOutputTime += outputInterval;
        if(nextOutputTime <= previousOutputTime){
            return false;
        }
    } while(timeReached(currentTime, nextOutputTime));
    return true;
}
}

Circuit::Circuit():
    mna_(std::make_unique<MNA>()),
    nodeMap_(std::make_unique<NodeMap>()) {}

Circuit::~Circuit() = default;

const Model* Circuit::addModel(std::unique_ptr<Model> model){
    const std::string name = model->name();
    auto& slot = models_[name];
    slot = std::move(model);
    return slot.get();
}

const Model* Circuit::findModel(const std::string& name) const{
    auto it = models_.find(name);
    return it == models_.end() ? nullptr : it->second.get();
}

PtaDiagnostics Circuit::ptaDiagnostics() const{
    return {
        operatingPointStats_.ptaAttempted,
        operatingPointStats_.converged,
        operatingPointStats_.hasPtaConvergenceMetrics,
        operatingPointStats_.ptaIterations,
        operatingPointStats_.ptaCapacitanceGrowths,
        operatingPointStats_.ptaCapacitanceReductions,
        operatingPointStats_.ptaMinimumStepRecoveries,
        operatingPointStats_.ptaNormalizedDerivative,
        operatingPointStats_.ptaNormalizedDcResidual
    };
}

const OperatingPointDiagnostics&
Circuit::operatingPointDiagnostics() const noexcept {
    return operatingPointStats_;
}

const TransientDiagnostics& Circuit::transientDiagnostics() const noexcept {
    return transientStats_;
}

CircuitDiagnostics Circuit::circuitDiagnostics() const {
    CircuitDiagnostics diagnostics;
    diagnostics.deviceCount = static_cast<int>(devices_.size());
    diagnostics.modelCount = static_cast<int>(models_.size());
    diagnostics.nodeCount = nodeMap_ ? nodeMap_->nodeCount() : 0;
    diagnostics.unknownCount = mna_ ? mna_->size() : 0;
    diagnostics.matrixNonZeros = mna_ ? mna_->nonZeroCount() : 0;
    diagnostics.pseudoDeviceCount = static_cast<int>(pseudoDevices_.size());

    constexpr int deviceTypeCount = 8;
    int counts[deviceTypeCount] = {};
    for(const auto& device: devices_){
        const int typeIndex = static_cast<int>(device->getType());
        if(typeIndex >= 0 && typeIndex < deviceTypeCount){
            ++counts[typeIndex];
        }
        if(device->isNonlinear()){
            ++diagnostics.nonlinearDeviceCount;
        }
    }
    for(int typeIndex = 0; typeIndex < deviceTypeCount; ++typeIndex){
        if(counts[typeIndex] == 0){
            continue;
        }
        diagnostics.devicesByType.push_back({
            deviceTypeName(static_cast<DeviceType>(typeIndex)),
            counts[typeIndex]
        });
    }

    if(!mna_ || mna_->solution().size() == 0 ||
       !mna_->solution().allFinite()){
        return diagnostics;
    }

    diagnostics.hasFiniteSolution = true;
    const Eigen::VectorXd& solution = mna_->solution();
    diagnostics.maximumAbsoluteSolution = solution.cwiseAbs().maxCoeff();

    if(nodeMap_ && nodeMap_->nodeCount() > 0){
        const auto nodeVoltages = solution.head(nodeMap_->nodeCount());
        diagnostics.minimumNodeVoltage = nodeVoltages.minCoeff();
        diagnostics.maximumNodeVoltage = nodeVoltages.maxCoeff();

        Eigen::Index maximumNode = 0;
        nodeVoltages.cwiseAbs().maxCoeff(&maximumNode);
        diagnostics.maximumAbsoluteSolutionVariable =
            "v(" + nodeMap_->nodeNameByIdx()[
                static_cast<std::size_t>(maximumNode)] + ")";
    }

    for(const auto& device: devices_){
        const int branch = device->branchUnknown();
        if(branch < 0 ||
           static_cast<Eigen::Index>(branch) >= solution.size()){
            continue;
        }
        const double magnitude = std::abs(solution[branch]);
        if(magnitude >= diagnostics.maximumAbsoluteBranchCurrent){
            diagnostics.maximumAbsoluteBranchCurrent = magnitude;
            diagnostics.maximumAbsoluteBranchCurrentDevice = device->getName();
        }
        if(magnitude >= diagnostics.maximumAbsoluteSolution){
            diagnostics.maximumAbsoluteSolution = magnitude;
            diagnostics.maximumAbsoluteSolutionVariable =
                "i(" + device->getName() + ")";
        }
    }
    return diagnostics;
}

int Circuit::allocateUnknown(){
    return nextUnknown_++;
}

bool Circuit::build(const PtaAnalysisConfig& config){
    nodeMap_->build(devices_);

    for(auto& device: devices_){
        device->bindNodes(*nodeMap_);
    }

    cacheOperatingPointDeviceRoles();

    nextUnknown_ = nodeMap_->nodeCount();

    for(auto& device: devices_){
        device->allocateUnknown(*this);
    }

    if(config.mode != PtaMode::Disabled){
        collectPendingPtaPlacements(config);
        materializePseudoDevices(config);
    }else{
        pendingPtaPlacements_.clear();
        pseudoDevices_.clear();
        ptaNodeCaps_.clear();
    }

    mna_->resize(nextUnknown_);
    mna_->reservePattern(
        devices_.size() * 12 + static_cast<std::size_t>(nextUnknown_)
    );

    for(auto& device: devices_){
        device->pattern(*mna_);
    }
    for(auto& pseudoDevice: pseudoDevices_){
        pseudoDevice->pattern(*mna_);
    }

    mna_->build();

    for(auto& device: devices_){
        device->bindMatrix(*mna_);
    }
    for(auto& pseudoDevice: pseudoDevices_){
        pseudoDevice->bindMatrix(*mna_);
    }
    mna_->releaseBuildMetadata();

    return true;
}

bool Circuit::solve(){
    return solveOperatingPoint();
}

bool Circuit::solveOperatingPoint(){
    return solveOperatingPoint({});
}

bool Circuit::solveOperatingPoint(
    const OperatingPointSolverOptions& options
){
    operatingPointStats_ = {};
    operatingPointStats_.attempted = true;
    operatingPointStats_.maxIterations = options.newton.maximumIterations;
    operatingPointStats_.tolerance = options.newton.tolerance;
    operatingPointStats_.minSourceStep = options.sourceStepping.minimumStep;

    const std::clock_t startClock = std::clock();
    const SteadyClock::time_point startWall = SteadyClock::now();
    const auto finish = [this, startClock, startWall](
        bool converged,
        const std::string& failureReason = std::string{}
    ) {
        operatingPointStats_.converged = converged;
        operatingPointStats_.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        operatingPointStats_.wallSeconds = elapsedWallSeconds(startWall);
        if(converged){
            operatingPointStats_.failureReason.clear();
        }else if(!failureReason.empty()){
            operatingPointStats_.failureReason = failureReason;
        }
        return converged;
    };

    if(!options.valid()){
        return finish(false, "operating-point solver configuration is invalid");
    }

    const AssembleCallback assemble = [this] {
        assembleOperatingPointSystem();
    };
    const Eigen::VectorXd initialSolution = mna_->solution();

    if(!hasNonlinearDevices()){
        setOperatingPointSourceScale(1.0);

        NewtonSolveDiagnostics linearStats;
        const bool linearSolved = solveLinearSystem(assemble, linearStats);
        operatingPointStats_.directNewton = linearStats;
        addNewtonStats(linearStats);

        operatingPointStats_.sourceScale = linearSolved ? 1.0 : 0.0;
        operatingPointStats_.finalMethod = "linear";
        return finish(linearSolved, linearStats.failureReason);
    }

    saveNonlinearIterationStates();
    setOperatingPointSourceScale(1.0);

    NewtonSolveDiagnostics directStats;
    const bool directConverged = solveNewtonSystem(
        assemble,
        directStats,
        options.newton
    );
    operatingPointStats_.directNewton = directStats;
    addNewtonStats(directStats);

    if(directConverged){
        operatingPointStats_.sourceScale = 1.0;
        operatingPointStats_.finalMethod = "direct Newton-Raphson";
        return finish(true);
    }

    restoreNonlinearIterationStates();
    mna_->setSolution(initialSolution);
    setOperatingPointSourceScale(0.0);

    if(!options.sourceStepping.enabled){
        return finish(
            false,
            "direct Newton-Raphson failed (" + directStats.failureReason +
                ") and source stepping is disabled"
        );
    }
    if(!solveOperatingPointWithSourceStepping(assemble, options)){
        return finish(
            false,
            operatingPointStats_.failureReason.empty()
                ? "source stepping failed"
                : operatingPointStats_.failureReason
        );
    }

    setOperatingPointSourceScale(1.0);
    operatingPointStats_.finalMethod = "source stepping";
    return finish(true);
}

bool Circuit::solveTransient(const TransientAnalysisConfig& config){
    return solveTransient(config, {});
}

bool Circuit::solveTransient(
    const TransientAnalysisConfig& config,
    const OperatingPointSolverOptions& operatingPointOptions
){
    transientStats_ = {};
    transientStats_.attempted = true;
    transientStats_.usedInitialConditions = config.useInitialConditions;
    transientStats_.finalMethod =
        "adaptive Backward Euler / variable-step BDF2";
    transientStats_.maxIterations =
        config.solverOptions.newtonOptions.maximumIterations;
    transientStats_.tolerance = config.solverOptions.newtonOptions.tolerance;
    transientSamples_.clear();

    const std::clock_t startClock = std::clock();
    const SteadyClock::time_point startWall = SteadyClock::now();
    TransientIntegrator integrator;
    double time = 0.0;

    const auto finishTransient = [this, startClock, startWall, &time](
        bool converged,
        const std::string& failureReason = std::string{}
    ) {
        transientStats_.converged = converged;
        transientStats_.finalTime = time;
        transientStats_.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        transientStats_.wallSeconds = elapsedWallSeconds(startWall);
        if(converged){
            transientStats_.failureReason.clear();
        }else if(!failureReason.empty()){
            transientStats_.failureReason = failureReason;
        }
        return converged;
    };

    const double maximumIntegrationStep = config.maximumStep
        ? *config.maximumStep
        : config.outputInterval;

    if(!config.valid() || !operatingPointOptions.valid()){
        return finishTransient(
            false,
            "transient or operating-point initialization configuration is invalid"
        );
    }

    const bool hasMos3UicChargeHistory = config.useInitialConditions &&
        std::any_of(
            devices_.begin(), devices_.end(),
            [](const std::unique_ptr<Device>& device){
                return device->requiresUicChargeHistoryProtection();
            }
        );

    Eigen::VectorXd initialSolution;

    setOperatingPointSourceScale(1.0);
    if(config.useInitialConditions){
        initialSolution = Eigen::VectorXd::Zero(mna_->size());
        mna_->setSolution(initialSolution);
    } else {
        if(!solveOperatingPoint(operatingPointOptions)){
            transientStats_.initializationCpuSeconds =
                double(std::clock() - startClock) / CLOCKS_PER_SEC;
            transientStats_.initializationWallSeconds =
                elapsedWallSeconds(startWall);
            return finishTransient(
                false,
                "transient operating-point initialization failed: " +
                    operatingPointStats_.failureReason
            );
        }
        initialSolution = mna_->solution();
    }

    integrator.Initialize(time, initialSolution);
    for(auto& device: devices_){
        device->initializeTransientHistory(initialSolution);
    }

    transientStats_.initializationCpuSeconds =
        double(std::clock() - startClock) / CLOCKS_PER_SEC;
    transientStats_.initializationWallSeconds = elapsedWallSeconds(startWall);

    double nextOutputTime = config.outputStartTime;
    if(timeReached(time, nextOutputTime)){
        recordTransientSample(time);
        if(!advanceOutputTime(
            nextOutputTime,
            time,
            config.outputInterval
        )){
            return finishTransient(
                false,
                "transient output time cannot be advanced"
            );
        }
    }

    double proposedStep = maximumIntegrationStep;

    const auto failTransient = [&finishTransient](const std::string& reason) {
        return finishTransient(false, reason);
    };

    while(!timeReached(time, config.stopTime)){
        double hardStepLimit = std::min(
            {
                maximumIntegrationStep,
                nextOutputTime - time,
                config.stopTime - time
            }
        );

        const bool hasBdf2History = !hasMos3UicChargeHistory &&
            integrator.olderSolution() != nullptr;
        if(hasBdf2History){
            const double bdf2StepLimit = std::nextafter(
                integrator.previousStep() * integrator.maximumBdf2StepRatio(),
                0.0
            );
            hardStepLimit = std::min(hardStepLimit, bdf2StepLimit);
        }

        if(!std::isfinite(hardStepLimit) || hardStepLimit <= 0.0){
            return failTransient(
                "transient hard step limit is non-finite or non-positive"
            );
        }

        double candidateStep = std::min(proposedStep, hardStepLimit);
        if(!std::isfinite(candidateStep) ||
           candidateStep <= 0.0 ||
           time + candidateStep <= time){
            return failTransient(
                "transient candidate step is invalid or cannot advance time"
            );
        }

        int rejectedAttempts = 0;

        while(true){
            const double nextTime = time + candidateStep;
            if(!std::isfinite(nextTime) || nextTime <= time){
                return failTransient(
                    "transient candidate time is invalid or did not advance"
                );
            }

            const Eigen::VectorXd acceptedSolution = mna_->solution();
            saveNonlinearIterationStates();

            const auto runTransientAttempt =
                [&](const TransientIntegrator& trialIntegrator,
                    double trialTime) {
                    TransientStepAttempt trial = tryTransientStep(
                        trialIntegrator,
                        trialTime,
                        config.solverOptions
                    );
                    ++transientStats_.attemptedSteps;
                    const double attemptedStep =
                        trialTime - trialIntegrator.acceptedTime();
                    updateStepRange(
                        attemptedStep,
                        transientStats_.minimumAttemptedStep,
                        transientStats_.maximumAttemptedStep
                    );
                    if(trial.integrationOrder == 1){
                        ++transientStats_.backwardEulerAttempts;
                    }else if(trial.integrationOrder == 2){
                        ++transientStats_.bdf2Attempts;
                    }
                    addTransientStats(trial.newtonStats);
                    return trial;
                };

            TransientStepAttempt attempt;

            if(!hasBdf2History){
                const double halfStep = 0.5 * candidateStep;
                const double halfTime = time + halfStep;

                if(!std::isfinite(halfStep) ||
                   halfStep < config.solverOptions.minimumStep ||
                   !std::isfinite(halfTime) ||
                   halfTime <= time ||
                   nextTime <= halfTime){
                    restoreTransientCheckpoint(acceptedSolution);
                    return failTransient(
                        "Backward Euler step-doubling requires a valid half-step "
                        "at or above the transient minimum step"
                    );
                }

                TransientIntegrator coarseIntegrator(
                    integrator.maximumBdf2StepRatio()
                );
                if(hasMos3UicChargeHistory){
                    coarseIntegrator.restartFrom(time, acceptedSolution);
                }
                TransientStepAttempt coarse = runTransientAttempt(
                    hasMos3UicChargeHistory ? coarseIntegrator : integrator,
                    nextTime
                );
                attempt = coarse;

                if(coarse.converged && coarse.integrationOrder == 1){
                    // The coarse solve may change nonlinear limiting state;
                    // start the fine pair from the same accepted checkpoint.
                    restoreTransientCheckpoint(acceptedSolution);

                    TransientIntegrator fineIntegrator(
                        integrator.maximumBdf2StepRatio()
                    );
                    fineIntegrator.restartFrom(time, acceptedSolution);

                    TransientStepAttempt firstHalf = runTransientAttempt(
                        fineIntegrator,
                        halfTime
                    );
                    attempt = firstHalf;

                    if(firstHalf.converged &&
                       firstHalf.integrationOrder == 1){
                        advanceTransientHistory(
                            acceptedSolution, firstHalf.solution
                        );
                        firstHalf.transientHistoryAdvanced = true;
                        // restartFrom deliberately clears history: the
                        // second half is BE rather than an internal BDF2 step.
                        fineIntegrator.restartFrom(
                            halfTime,
                            firstHalf.solution
                        );

                        TransientStepAttempt secondHalf =
                            runTransientAttempt(
                                fineIntegrator,
                                nextTime
                            );

                        if(secondHalf.converged &&
                           secondHalf.integrationOrder == 1){
                            advanceTransientHistory(
                                firstHalf.solution, secondHalf.solution
                            );
                            secondHalf.transientHistoryAdvanced = true;
                            const TransientErrorEstimate estimate =
                                estimateTransientSolutionDifference(
                                    acceptedSolution,
                                    secondHalf.solution,
                                    coarse.solution,
                                    nodeMap_->nodeCount(),
                                    config.solverOptions,
                                    !hasMos3UicChargeHistory
                                );

                            // Commit the fine endpoint, never the coarse
                            // whole-step solution or a Richardson extrapolate.
                            secondHalf.errorEstimateValid = estimate.valid;
                            secondHalf.normalizedError =
                                estimate.normalizedError;
                            secondHalf.suggestedStepScale =
                                estimate.suggestedScale;
                        }
                        attempt = std::move(secondHalf);
                    }
                }
            } else {
                attempt = runTransientAttempt(integrator, nextTime);
            }

            transientStats_.lastAttemptedStep = candidateStep;
            transientStats_.lastAttemptedTime = nextTime;
            transientStats_.lastIntegrationOrder = attempt.integrationOrder;
            transientStats_.lastNewton = attempt.newtonStats;
            if(attempt.errorEstimateValid &&
               std::isfinite(attempt.normalizedError)){
                transientStats_.hasNormalizedError = true;
                transientStats_.lastNormalizedError =
                    attempt.normalizedError;
                transientStats_.maximumNormalizedError = std::max(
                    transientStats_.maximumNormalizedError,
                    attempt.normalizedError
                );
            }

            TransientStepControlInput controlInput;
            controlInput.converged = attempt.converged;
            controlInput.requiresBdf2 = hasBdf2History;
            controlInput.integrationOrder = attempt.integrationOrder;
            controlInput.errorEstimateValid = attempt.errorEstimateValid;
            controlInput.normalizedError = attempt.normalizedError;
            controlInput.suggestedStepScale = attempt.suggestedStepScale;
            controlInput.currentTime = time;
            controlInput.attemptedStep = candidateStep;
            controlInput.hardStepLimit = hardStepLimit;
            controlInput.rejectedAttempts = rejectedAttempts;

            const TransientStepControlDecision decision =
                decideTransientStep(controlInput, config.solverOptions);

            if(decision.action == TransientStepAction::Accept){
                mna_->setSolution(attempt.solution);
                if(!attempt.transientHistoryAdvanced){
                    advanceTransientHistory(
                        integrator.currentSolution(), attempt.solution
                    );
                }
                integrator.accept(nextTime, std::move(attempt.solution));

                time = nextTime;
                if(timeReached(time, config.stopTime)){
                    time = config.stopTime;
                }
                ++transientStats_.timeSteps;
                updateStepRange(
                    candidateStep,
                    transientStats_.minimumAcceptedStep,
                    transientStats_.maximumAcceptedStep
                );

                proposedStep = std::min(
                    maximumIntegrationStep,
                    decision.nextStep
                );

                if(!std::isfinite(proposedStep) || proposedStep <= 0.0){
                    return failTransient(
                        "transient step controller proposed an invalid next step"
                    );
                }

                if(timeReached(time, nextOutputTime)){
                    recordTransientSample(time);
                    if(!timeReached(time, config.stopTime) &&
                       !advanceOutputTime(
                           nextOutputTime,
                           time,
                           config.outputInterval
                       )){
                        return failTransient(
                            "transient output time cannot be advanced"
                        );
                    }
                }

                break;
            }

            ++transientStats_.rejectedSteps;
            if(!attempt.converged){
                ++transientStats_.convergenceRejectedSteps;
            } else if(decision.action ==
                      TransientStepAction::FailInvalidEstimate){
                ++transientStats_.invalidEstimateFailures;
            } else if(attempt.errorEstimateValid &&
                      attempt.normalizedError > 1.0){
                ++transientStats_.errorRejectedSteps;
            }

            restoreTransientCheckpoint(acceptedSolution);

            if(decision.action == TransientStepAction::RetryConvergence ||
               decision.action == TransientStepAction::RetryError){
                candidateStep = decision.nextStep;
                ++rejectedAttempts;
                continue;
            }

            return failTransient(transientFailureReason(decision.action));
        }
    }

    if(transientSamples_.empty() ||
       !sameTime(transientSamples_.back().time, time)){
        recordTransientSample(time);
    }

    return finishTransient(true);
}

bool Circuit::solveAdaptivePta(const PtaAnalysisConfig& config){
    const bool continuesFailedOrdinarySolve =
        operatingPointStats_.attempted &&
        !operatingPointStats_.converged &&
        !operatingPointStats_.ptaAttempted;
    if(!continuesFailedOrdinarySolve){
        operatingPointStats_ = {};
    }
    operatingPointStats_.attempted = true;
    operatingPointStats_.ptaAttempted = true;
    operatingPointStats_.maxIterations = config.newtonOptions.maximumIterations;
    operatingPointStats_.tolerance = config.newtonOptions.tolerance;
    operatingPointStats_.ptaAttempts.clear();

    const double priorCpuSeconds = operatingPointStats_.cpuSeconds;
    const double priorWallSeconds = operatingPointStats_.wallSeconds;
    const std::clock_t startClock = std::clock();
    const SteadyClock::time_point startWall = SteadyClock::now();
    const auto finish = [
        this,
        startClock,
        startWall,
        priorCpuSeconds,
        priorWallSeconds
    ](
        bool converged,
        const std::string& failureReason = std::string{}
    ) {
        operatingPointStats_.ptaCpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        operatingPointStats_.ptaWallSeconds =
            elapsedWallSeconds(startWall);
        operatingPointStats_.cpuSeconds =
            priorCpuSeconds + operatingPointStats_.ptaCpuSeconds;
        operatingPointStats_.wallSeconds =
            priorWallSeconds + operatingPointStats_.ptaWallSeconds;
        operatingPointStats_.converged = converged;
        if(converged){
            operatingPointStats_.finalMethod =
                "pseudo-transient analysis (PTA)";
            operatingPointStats_.failureReason.clear();
        }else if(!failureReason.empty()){
            operatingPointStats_.failureReason = failureReason;
        }
        return converged;
    };

    TransientIntegrator integrator;
    double time = 0.0;
    double step = config.initialStep;

    initializePtaStates(config, time);

    integrator.Initialize(time, mna_->solution());

    Eigen::VectorXd derivative(mna_->size());
    Eigen::VectorXd matrixProduct(mna_->size());
    Eigen::VectorXd residual(mna_->size());

    for(int stepCount = 0;
        stepCount < config.maximumSteps;
        ++stepCount)
    {
        double candidateStep = std::min(step, config.maximumStep);
        if(integrator.olderSolution() != nullptr){
            const double bdf2StepLimit = std::nextafter(
                integrator.previousStep() *
                    integrator.maximumBdf2StepRatio(),
                0.0
            );
            candidateStep = std::min(candidateStep, bdf2StepLimit);
        }

        const double nextTime = time + candidateStep;
        PtaStepAttemptDiagnostics attempt;
        attempt.attempt = stepCount + 1;
        attempt.startTime = time;
        attempt.targetTime = nextTime;
        attempt.timeStep = candidateStep;
        operatingPointStats_.ptaFinalStep = candidateStep;
        updateStepRange(
            candidateStep,
            operatingPointStats_.ptaMinimumAttemptedStep,
            operatingPointStats_.ptaMaximumAttemptedStep
        );

        if(!std::isfinite(candidateStep) ||
           candidateStep < config.minimumStep ||
           !std::isfinite(nextTime) ||
           nextTime <= time){
            attempt.status = "failed";
            attempt.failureReason =
                "PTA time step fell below the minimum or cannot advance pseudo-time";
            operatingPointStats_.ptaAttempts.push_back(std::move(attempt));
            return finish(
                false,
                "PTA time step fell below the minimum or cannot advance pseudo-time"
            );
        }

        const Eigen::VectorXd acceptedSolution =
            integrator.currentSolution();

        const double sourceScale = ptaSourceRampScale(
            nextTime,
            config.sourceRampTime
        );
        attempt.sourceScale = sourceScale;
        operatingPointStats_.ptaFinalSourceScale = sourceScale;
        if(!std::isfinite(sourceScale)){
            attempt.status = "failed";
            attempt.failureReason = "PTA source-ramp scale is non-finite";
            operatingPointStats_.ptaAttempts.push_back(std::move(attempt));
            return finish(false, "PTA source-ramp scale is non-finite");
        }
        setOperatingPointSourceScale(sourceScale);

        mna_->setSolution(integrator.predict(nextTime));
        saveNonlinearIterationStates();

        const TransientStampContext ctx =
            integrator.makeContext(nextTime);
        attempt.integrationOrder = ctx.derivative.order;

        const AssembleCallback assemble = [this, &ctx] {
            assemblePtaSystem(ctx);
        };

        NewtonSolveDiagnostics stats;
        const bool converged = hasNonlinearDevices()
            ? solveNewtonSystem(assemble, stats, config.newtonOptions)
            : solveLinearSystem(assemble, stats);
        attempt.newton = stats;
        addNewtonStats(stats);
        operatingPointStats_.ptaIterations += stats.iterations;
        operatingPointStats_.ptaDampedSteps += stats.dampedSteps;

        if(!converged){
            ++operatingPointStats_.ptaRejectedSteps;
            restoreTransientCheckpoint(acceptedSolution);

            const double reducedStep = std::max(
                config.minimumStep,
                candidateStep * config.failedStepScale
            );

            if(reducedStep < candidateStep){
                attempt.retriedWithSmallerStep = true;
                attempt.status = "retry with smaller pseudo-time step";
                attempt.failureReason = stats.failureReason;
                operatingPointStats_.ptaAttempts.push_back(std::move(attempt));
                step = reducedStep;
                continue;
            }

            if(!growAllPtaNodeCapacitances(config)){
                attempt.status = "failed";
                attempt.failureReason =
                    "PTA Newton solve failed at the minimum step and node "
                    "capacitance cannot be increased";
                operatingPointStats_.ptaAttempts.push_back(std::move(attempt));
                return finish(
                    false,
                    "PTA Newton solve failed at the minimum step and node "
                    "capacitance cannot be increased"
                );
            }

            ++operatingPointStats_.ptaMinimumStepRecoveries;
            ++operatingPointStats_.ptaCapacitanceGrowths;
            attempt.restartedAfterCapacitanceGrowth = true;
            attempt.status = "restart after PTA capacitance growth";
            attempt.failureReason =
                "Newton solve failed at the minimum PTA step";
            operatingPointStats_.ptaAttempts.push_back(std::move(attempt));
            integrator.Initialize(time, acceptedSolution);
            step = reducedStep;
            continue;
        }

        Eigen::VectorXd currentSolution = mna_->solution();

        // BDF coefficients sum to zero.  Evaluate the derivative from state
        // differences so a settled solution does not lose precision through
        // cancellation among terms of order |x| / h.
        derivative = currentSolution - ctx.previousSolution;
        derivative *= ctx.derivative.alpha0;
        if(ctx.olderSolution != nullptr){
            derivative += ctx.derivative.alpha2 *
                (*ctx.olderSolution - ctx.previousSolution);
        }
        if(!derivative.allFinite()){
            attempt.status = "failed";
            attempt.failureReason = "PTA derivative contains a non-finite value";
            operatingPointStats_.ptaAttempts.push_back(std::move(attempt));
            return finish(
                false,
                "PTA derivative contains a non-finite value"
            );
        }
        const PtaDerivativeEstimate derivativeEstimate =
            estimatePtaNormalizedDerivative(
                derivative,
                currentSolution,
                ctx.previousSolution,
                nodeMap_->nodeCount(),
                ctx.timeStep,
                config
        );
        if(!derivativeEstimate.valid){
            attempt.status = "failed";
            attempt.failureReason = "PTA normalized derivative estimate is invalid";
            operatingPointStats_.ptaAttempts.push_back(std::move(attempt));
            return finish(
                false,
                "PTA normalized derivative estimate is invalid"
            );
        }

        // Test the original DC residual F(x), excluding artificial PTA terms
        // and always using the nominal independent-source values.  The ramp
        // is a convergence aid and must not redefine the target DC system.
        setOperatingPointSourceScale(1.0);
        assembleOperatingPointSystem();
        if(!mna_->evaluateResidual(matrixProduct, residual)){
            attempt.status = "failed";
            attempt.failureReason = "PTA DC residual contains a non-finite value";
            operatingPointStats_.ptaAttempts.push_back(std::move(attempt));
            return finish(
                false,
                "PTA DC residual contains a non-finite value"
            );
        }
        const PtaResidualEstimate dcResidualEstimate =
            estimatePtaNormalizedResidual(
                residual,
                matrixProduct,
                mna_->rhs(),
                nodeMap_->nodeCount(),
                config
        );
        if(!dcResidualEstimate.valid){
            attempt.status = "failed";
            attempt.failureReason = "PTA normalized DC residual estimate is invalid";
            operatingPointStats_.ptaAttempts.push_back(std::move(attempt));
            return finish(
                false,
                "PTA normalized DC residual estimate is invalid"
            );
        }

        attempt.hasConvergenceMetrics = true;
        attempt.normalizedDerivative =
            derivativeEstimate.normalizedDerivative;
        attempt.normalizedDcResidual =
            dcResidualEstimate.normalizedResidual;
        operatingPointStats_.hasPtaConvergenceMetrics = true;
        operatingPointStats_.ptaNormalizedDerivative =
            derivativeEstimate.normalizedDerivative;
        operatingPointStats_.ptaNormalizedDcResidual =
            dcResidualEstimate.normalizedResidual;

        if(derivativeEstimate.normalizedDerivative <
               config.derivativeTolerance &&
           dcResidualEstimate.normalizedResidual <
               config.dcResidualTolerance){
            attempt.accepted = true;
            attempt.reachedSteadyState = true;
            attempt.status = "steady state converged";
            ++operatingPointStats_.ptaAcceptedSteps;
            operatingPointStats_.ptaFinalTime = nextTime;
            operatingPointStats_.sourceScale = 1.0;
            operatingPointStats_.ptaAttempts.push_back(std::move(attempt));
            return finish(true);
        }

        const int reductionsBefore =
            operatingPointStats_.ptaCapacitanceReductions;
        updatePtaNodeCapacitancesAfterAcceptedStep(
            currentSolution,
            acceptedSolution,
            config
        );
        attempt.capacitanceReductions =
            operatingPointStats_.ptaCapacitanceReductions - reductionsBefore;
        attempt.accepted = true;
        attempt.status = "accepted";
        ++operatingPointStats_.ptaAcceptedSteps;
        operatingPointStats_.ptaFinalTime = nextTime;
        operatingPointStats_.ptaAttempts.push_back(std::move(attempt));

        integrator.accept(nextTime, std::move(currentSolution));
        time = nextTime;
        step = std::min(
            config.maximumStep,
            candidateStep * config.successfulStepScale
        );
    }

    return finish(
        false,
        "PTA maximum pseudo-time step count was reached before convergence"
    );
}

Circuit::TransientStepAttempt Circuit::tryTransientStep(
    const TransientIntegrator& integrator,
    double targetTime,
    const TransientSolverOptions& options
){
    TransientStepAttempt attempt;

    mna_->setSolution(integrator.predict(targetTime));

    const TransientStampContext ctx = integrator.makeContext(targetTime);

    attempt.integrationOrder = ctx.derivative.order;

    const AssembleCallback assemble = [this, &ctx]{
        assembleTransientSystem(ctx);
    };
    attempt.converged = hasNonlinearDevices()
        ? solveNewtonSystem(
            assemble,
            attempt.newtonStats,
            options.newtonOptions
        )
        : solveLinearSystem(assemble, attempt.newtonStats);

    if(attempt.converged){
        attempt.solution = mna_->solution();

        const TransientErrorEstimate estimate = integrator.estimateError(
            targetTime,
            attempt.solution,
            nodeMap_->nodeCount(),
            options
        );

        attempt.errorEstimateValid = estimate.valid;
        attempt.normalizedError = estimate.normalizedError;
        attempt.suggestedStepScale = estimate.suggestedScale;
    }

    return attempt;
}

void Circuit::addTransientStats(const NewtonSolveDiagnostics& stats){
    transientStats_.iterations += stats.iterations;
    transientStats_.dampedSteps += stats.dampedSteps;
    transientStats_.finalDelta = stats.finalDelta;
}

void Circuit::restoreTransientCheckpoint(const Eigen::VectorXd& acceptedSolution){
    restoreNonlinearIterationStates();
    mna_->setSolution(acceptedSolution);
}

void Circuit::advanceTransientHistory(const Eigen::VectorXd& previous,
                                      const Eigen::VectorXd& accepted){
    for(auto& device: devices_){
        device->acceptTransientSolution(previous, accepted);
    }
}

bool Circuit::solveOperatingPointWithSourceStepping(
    const AssembleCallback& assemble,
    const OperatingPointSolverOptions& options
){
    Eigen::VectorXd acceptedSolution = mna_->solution();
    double acceptedScale = 0.0;
    double sourceStep = options.sourceStepping.initialStep;

    while(acceptedScale < kSourceScaleDone){
        const double trialScale = std::min(1.0, acceptedScale + sourceStep);
        const double actualStep = trialScale - acceptedScale;

        SourceSteppingAttemptDiagnostics attempt;
        attempt.attempt =
            static_cast<int>(operatingPointStats_.sourceAttempts.size()) + 1;
        attempt.acceptedScaleBefore = acceptedScale;
        attempt.stepSize = actualStep;
        attempt.targetScale = trialScale;

        mna_->setSolution(acceptedSolution);
        saveNonlinearIterationStates();
        setOperatingPointSourceScale(trialScale);

        NewtonSolveDiagnostics trialStats;
        const bool converged = solveNewtonSystem(
            assemble,
            trialStats,
            options.newton
        );
        attempt.newton = trialStats;
        addNewtonStats(trialStats);

        if(converged){
            attempt.accepted = true;
            attempt.status = "accepted";
            operatingPointStats_.sourceAttempts.push_back(std::move(attempt));
            acceptedScale = trialScale;
            acceptedSolution = mna_->solution();
            operatingPointStats_.sourceScale = acceptedScale;
            ++operatingPointStats_.sourceSteps;
            sourceStep = std::min(
                options.sourceStepping.maximumStep,
                sourceStep * options.sourceStepping.growthFactor
            );
            continue;
        }

        restoreNonlinearIterationStates();
        mna_->setSolution(acceptedSolution);
        setOperatingPointSourceScale(acceptedScale);
        ++operatingPointStats_.failedSourceSteps;
        attempt.status = "rejected";
        operatingPointStats_.sourceAttempts.push_back(std::move(attempt));

        sourceStep *= options.sourceStepping.failureScale;
        if(sourceStep < options.sourceStepping.minimumStep){
            operatingPointStats_.failureReason =
                "source stepping failed because the next source step is "
                "below the configured minimum";
            return false;
        }
    }

    mna_->setSolution(acceptedSolution);
    return true;
}

bool Circuit::solveLinearSystem(const AssembleCallback& assemble,
                                NewtonSolveDiagnostics& stats){
    stats = {};
    stats.attempted = true;
    stats.maximumIterations = 1;
    const std::clock_t startClock = std::clock();
    const SteadyClock::time_point startWall = SteadyClock::now();
    const auto finish = [&stats, startClock, startWall](
        bool converged,
        const std::string& failureReason = std::string{}
    ) {
        stats.converged = converged;
        stats.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        stats.wallSeconds = elapsedWallSeconds(startWall);
        stats.failureReason = converged ? std::string{} : failureReason;
        return converged;
    };

    assemble();
    stats.iterations = 1;

    if(!mna_->solve()){
        return finish(
            false,
            "sparse linear system factorization or solve failed"
        );
    }
    if(!mna_->solution().allFinite()){
        return finish(false, "sparse linear solution contains a non-finite value");
    }

    stats.finalDelta = 0.0;
    return finish(true);
}

bool Circuit::solveNewtonSystem(const AssembleCallback& assemble,
                                NewtonSolveDiagnostics& stats,
                                const NewtonSolverOptions& options){
    stats = {};
    stats.attempted = true;
    stats.usedNewtonRaphson = true;
    stats.maximumIterations = options.maximumIterations;
    stats.tolerance = options.tolerance;

    const std::clock_t startClock = std::clock();
    const SteadyClock::time_point startWall = SteadyClock::now();
    const auto finish = [&stats, startClock, startWall](
        bool converged,
        const std::string& failureReason = std::string{}
    ) {
        stats.converged = converged;
        stats.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        stats.wallSeconds = elapsedWallSeconds(startWall);
        stats.failureReason = converged ? std::string{} : failureReason;
        return converged;
    };

    if(!options.valid()){
        return finish(false, "Newton-Raphson configuration is invalid");
    }

    Eigen::VectorXd previous = mna_->solution();

    for(int iter = 0; iter < options.maximumIterations; ++iter){
        stats.iterations = iter + 1;
        assemble();

        if(!mna_->solve()){
            return finish(
                false,
                "sparse linear system factorization or solve failed during "
                "Newton-Raphson iteration"
            );
        }

        Eigen::VectorXd& current = mna_->solution();
        const NewtonStepResult step = limitNewtonStep(
            current,
            previous,
            options.maximumSolutionStep
        );
        if(!std::isfinite(step.delta)){
            return finish(
                false,
                "Newton-Raphson update is non-finite"
            );
        }
        if(step.limited){
            ++stats.dampedSteps;
        }

        stats.finalDelta = step.delta;
        if(step.delta < options.tolerance){
            return finish(true);
        }

        previous = current;
    }

    return finish(
        false,
        "Newton-Raphson maximum iteration count was reached"
    );
}

void Circuit::addNewtonStats(const NewtonSolveDiagnostics& stats){
    operatingPointStats_.iterations += stats.iterations;
    operatingPointStats_.dampedSteps += stats.dampedSteps;
    operatingPointStats_.finalDelta = stats.finalDelta;
}

void Circuit::cacheOperatingPointDeviceRoles(){
    sourceSteppingDevices_.clear();
    iterationStateDevices_.clear();
    hasNonlinearDevices_ = false;

    for(auto& device: devices_){
        if(device->getType() == DeviceType::VoltageSource ||
           device->getType() == DeviceType::CurrentSource){
            sourceSteppingDevices_.push_back(device.get());
        }

        if(device->isNonlinear()){
            hasNonlinearDevices_ = true;
            iterationStateDevices_.push_back(device.get());
        }
    }
}

void Circuit::assembleOperatingPointSystem(){
    mna_->clear();
    for(auto& device: devices_){
        device->stampOperatingPoint();
    }
}

void Circuit::assembleTransientSystem(const TransientStampContext& ctx){
    mna_->clear();
    for(auto& device: devices_){
        device->stampTransient(ctx);
    }
}

void Circuit::assemblePtaSystem(const TransientStampContext& ctx){
    mna_->clear();

    for(auto& device: devices_){
        device->stampOperatingPoint();
    }

    for(auto& pseudoDevice: pseudoDevices_){
        pseudoDevice->stampPseudo(ctx);
    }
}

bool Circuit::hasNonlinearDevices() const{
    return hasNonlinearDevices_;
}

void Circuit::setOperatingPointSourceScale(double scale){
    if(scale == operatingPointSourceScale_){
        return;
    }

    operatingPointSourceScale_ = scale;
    for(auto* device: sourceSteppingDevices_){
        device->setOperatingPointSourceScale(scale);
    }
}

void Circuit::initializePtaStates(const PtaAnalysisConfig& config,
                                  double time){
    const double sourceScale = ptaSourceRampScale(
        time,
        config.sourceRampTime
    );
    setOperatingPointSourceScale(sourceScale);

    if(config.initialBjtVbe){
        for(auto& device: devices_){
            if(device->getType() == DeviceType::BJT){
                device->initializePtaState(*config.initialBjtVbe);
            }
        }
    }
}

void Circuit::saveNonlinearIterationStates(){
    for(auto* device: iterationStateDevices_){
        device->saveIterationState();
    }
}

void Circuit::restoreNonlinearIterationStates(){
    for(auto* device: iterationStateDevices_){
        device->restoreIterationState();
    }
}

void Circuit::recordTransientSample(double time){
    transientSamples_.push_back({time, mna_->solution()});
    ++transientStats_.outputPoints;
}

void Circuit::collectPendingPtaPlacements(const PtaAnalysisConfig& config){
    pendingPtaPlacements_.clear();

    const auto addNodeCaps = 
        [this](const Device* owner, std::initializer_list<int> terminals){
            const auto& nodes = owner->getNodeIds();

            for(int terminal: terminals){
                if(terminal < static_cast<int>(nodes.size()) &&
                    nodes[terminal] >= 0){
                        pendingPtaPlacements_.push_back({
                            PtaPlacementKind::TransistorNodeCap,
                            owner,
                            terminal
                        });
                }
            }
    };

    for(auto& device: devices_){
        const Device* owner = device.get();

        switch(owner->getType()){
            case DeviceType::VoltageSource:{
                pendingPtaPlacements_.push_back({
                    PtaPlacementKind::VoltageSourceSeriesInductor,
                    owner,
                    -1
                });
                break;
            }
            case DeviceType::CurrentSource:{
                pendingPtaPlacements_.push_back({
                    PtaPlacementKind::CurrentSourceParallelCap,
                    owner,
                    -1
                });
                break;
            }
            case DeviceType::BJT:{
                addNodeCaps(owner, {0, 1, 2});
                break;
            }
            case DeviceType::MOSFET:{
                addNodeCaps(owner, {0, 1, 2});
                if(config.includeMosBulk){
                    addNodeCaps(owner, {3});
                }
                break;
            }
            case DeviceType::Diode:{
                if(config.includeDiodes){
                    addNodeCaps(owner, {0, 1});
                }
                break;
            }
            default:
                break;
        }
    }
}

void Circuit::materializePseudoDevices(const PtaAnalysisConfig& config){
    pseudoDevices_.clear();
    ptaNodeCaps_.clear();
    std::unordered_set<int> cappedNodes;
    std::set<std::pair<int, int>> cappedSourcePairs;
    std::unordered_set<int> inductedBranches;

    const auto addPseudo = [this](std::unique_ptr<PseudoDevice> device){
        pseudoDevices_.push_back(std::move(device));
    };

    const bool useCompoundElements = config.compoundTimeConstant > 0.0;

    for(const PendingPtaPlacement& placement: pendingPtaPlacements_){
        const Device* owner = placement.owner;
        if(owner == nullptr){
            continue;
        }

        const auto nodes = owner->getNodeIds();

        switch (placement.kind){
            case PtaPlacementKind::TransistorNodeCap:{
                const int terminal = placement.terminal;
                if(terminal < 0 || terminal >= static_cast<int>(nodes.size())){
                    continue;
                }

                const int node = nodes[terminal];
                if(node >= 0 && cappedNodes.insert(node).second){
                    const int branch = useCompoundElements
                        ? allocateUnknown()
                        : -1;
                    auto capacitor = std::make_unique<PseudoCapacitor>(
                        node,
                        -1,
                        config.initialNodeCapacitance,
                        useCompoundElements
                            ? config.compoundInitialResistance
                            : 0.0,
                        config.compoundTimeConstant,
                        branch
                    );
                    PseudoCapacitor* rawCapacitor = capacitor.get();

                    pseudoDevices_.push_back(std::move(capacitor));
                    ptaNodeCaps_.push_back({
                        node,
                        rawCapacitor,
                        config.initialNodeCapacitance,
                        0.0,
                        false
                    });
                }
                break;
            }

            case PtaPlacementKind::CurrentSourceParallelCap: {
                if(nodes.size() < 2){
                    continue;
                }

                const int p = nodes[0];
                const int n = nodes[1];
                const auto endpoints = std::minmax(p, n);
                const std::pair<int, int> key{
                    endpoints.first,
                    endpoints.second
                };

                if(p != n && cappedSourcePairs.insert(key).second){
                    const int branch = useCompoundElements
                        ? allocateUnknown()
                        : -1;
                    addPseudo(std::make_unique<PseudoCapacitor>(
                        p,
                        n,
                        config.currentSourceCapacitance,
                        useCompoundElements
                            ? config.compoundInitialResistance
                            : 0.0,
                        config.compoundTimeConstant,
                        branch
                    ));
                }
                break;
            }

            case PtaPlacementKind::VoltageSourceSeriesInductor:{
                const int branch = owner->branchUnknown();
                if(branch >= 0 && inductedBranches.insert(branch).second){
                    addPseudo(std::make_unique<PseudoInductor>(
                        branch,
                        config.voltageSourceInductance,
                        useCompoundElements
                            ? config.compoundInitialConductance
                            : 0.0,
                        config.compoundTimeConstant,
                        nodes.size() >= 1 ? nodes[0] : -1,
                        nodes.size() >= 2 ? nodes[1] : -1,
                        owner->nominalSourceValue(),
                        config.sourceRampTime
                    ));
                }
                break;
            }
        }
    }
}

bool Circuit::growAllPtaNodeCapacitances(const PtaAnalysisConfig& config){
    bool grewAnyCapacitance = false;

    for(PtaNodeCapState& state: ptaNodeCaps_){
        if(state.capacitor == nullptr ||
            !std::isfinite(state.capacitance) ||
            state.capacitance <= 0){
                continue;
        }

        const double nextCapacitance = std::min(
            state.capacitance * config.capacitanceGrowScale,
            config.maximumNodeCapacitance
        );

        if(!std::isfinite(nextCapacitance) ||
            nextCapacitance <= state.capacitance){
                continue;
        }

        state.capacitor->setValue(nextCapacitance);
        state.capacitance = nextCapacitance;
        state.previousDelta = 0.0;
        state.hasPreviousDelta = false;

        grewAnyCapacitance = true;
    }

    return grewAnyCapacitance;
}

void Circuit::updatePtaNodeCapacitancesAfterAcceptedStep(
    const Eigen::VectorXd& currentSolution,
    const Eigen::VectorXd& previousSolution,
    const PtaAnalysisConfig& config
){
    for(auto& state: ptaNodeCaps_){
        if(state.node < 0 ||
           state.node >= currentSolution.size() ||
           state.node >= previousSolution.size()){
            state.previousDelta = 0.0;
            state.hasPreviousDelta = false;
            continue;
        }

        const double currentDelta =
            currentSolution[state.node] - previousSolution[state.node];

        if(!std::isfinite(currentDelta)){
            state.previousDelta = 0.0;
            state.hasPreviousDelta = false;
            continue;
        }

        const bool oscillating =
            state.hasPreviousDelta &&
            std::isfinite(state.previousDelta) &&
            ((state.previousDelta < 0.0 && currentDelta > 0.0) ||
            (state.previousDelta > 0.0 && currentDelta < 0.0));

        if(oscillating &&
           state.capacitor != nullptr &&
           std::isfinite(state.capacitance) &&
           state.capacitance > 0.0){
            const double ratio =
                std::abs(currentDelta) / std::abs(state.previousDelta);

            double capacitanceScale = config.smallOscillationScale;
            if(ratio >= config.heavyOscillationRatio){
                capacitanceScale = config.heavyOscillationScale;
            }else if(ratio >= config.mediumOscillationRatio){
                capacitanceScale = config.mediumOscillationScale;
            }

            const double nextCapacitance = std::max(
                config.minimumNodeCapacitance,
                state.capacitance * capacitanceScale
            );

            if(std::isfinite(nextCapacitance) &&
               nextCapacitance < state.capacitance){
                state.capacitor->setValue(nextCapacitance);
                state.capacitance = nextCapacitance;
                ++operatingPointStats_.ptaCapacitanceReductions;
            }
        }

        state.previousDelta = currentDelta;
        state.hasPreviousDelta = true;
    }
}
