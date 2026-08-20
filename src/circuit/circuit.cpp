#include "circuit/circuit.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <limits>
#include <set>
#include <unordered_set>

#include "analysis/analysisPlan.h"
#include "analysis/solverOptions.h"
#include "analysis/transientAnalysis.h"
#include "circuit/nodeMap.h"
#include "devices/device.hpp"
#include "math/mna.hpp"
#include "math/newtonStep.hpp"
#include "models/model.hpp"
#include "devices/pseudoDevice.hpp"
#include "analysis/ptaAnalysis.h"

namespace {
constexpr double kSourceScaleDone = 1.0 - 1.0e-12;
constexpr double kTimeRelativeTolerance =
    64.0 * std::numeric_limits<double>::epsilon();

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
        operatingPointStats_.iterations,
        operatingPointStats_.ptaCapacitanceGrowths,
        operatingPointStats_.ptaCapacitanceReductions,
        operatingPointStats_.ptaMinimumStepRecoveries,
        operatingPointStats_.ptaNormalizedDerivative,
        operatingPointStats_.ptaNormalizedDcResidual
    };
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
    operatingPointStats_.maxIterations = options.newton.maximumIterations;
    operatingPointStats_.tolerance = options.newton.tolerance;
    operatingPointStats_.minSourceStep = options.sourceStepping.minimumStep;

    if(!options.valid()){
        return false;
    }

    const AssembleCallback assemble = [this] {
        assembleOperatingPointSystem();
    };
    const std::clock_t startClock = std::clock();
    const Eigen::VectorXd initialSolution = mna_->solution();

    if(!hasNonlinearDevices()){
        setOperatingPointSourceScale(1.0);

        NewtonStats linearStats;
        const bool linearSolved = solveLinearSystem(assemble, linearStats);
        addNewtonStats(linearStats);

        operatingPointStats_.sourceScale = linearSolved ? 1.0 : 0.0;
        operatingPointStats_.converged = linearSolved;
        operatingPointStats_.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        return linearSolved;
    }

    saveNonlinearIterationStates();
    setOperatingPointSourceScale(1.0);

    NewtonStats directStats;
    const bool directConverged = solveNewtonSystem(
        assemble,
        directStats,
        options.newton
    );
    addNewtonStats(directStats);

    if(directConverged){
        operatingPointStats_.sourceScale = 1.0;
        operatingPointStats_.converged = true;
        operatingPointStats_.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        return true;
    }

    restoreNonlinearIterationStates();
    mna_->setSolution(initialSolution);
    setOperatingPointSourceScale(0.0);

    if(!options.sourceStepping.enabled ||
       !solveOperatingPointWithSourceStepping(assemble, options)){
        operatingPointStats_.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        return false;
    }

    setOperatingPointSourceScale(1.0);
    operatingPointStats_.converged = true;
    operatingPointStats_.cpuSeconds =
        double(std::clock() - startClock) / CLOCKS_PER_SEC;
    return true;
}

bool Circuit::solveTransient(const TransientAnalysisConfig& config){
    return solveTransient(config, {});
}

bool Circuit::solveTransient(
    const TransientAnalysisConfig& config,
    const OperatingPointSolverOptions& operatingPointOptions
){
    transientStats_ = {};
    transientStats_.maxIterations =
        config.solverOptions.newtonOptions.maximumIterations;
    transientStats_.tolerance = config.solverOptions.newtonOptions.tolerance;
    transientSamples_.clear();

    const std::clock_t startClock = std::clock();
    TransientIntegrator integrator;
    double time = 0.0;

    const double maximumIntegrationStep = config.maximumStep
        ? *config.maximumStep
        : config.outputInterval;

    if(!config.valid() || !operatingPointOptions.valid()){
        transientStats_.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        return false;
    }

    Eigen::VectorXd initialSolution;

    setOperatingPointSourceScale(1.0);
    if(config.useInitialConditions){
        initialSolution = Eigen::VectorXd::Zero(mna_->size());
        mna_->setSolution(initialSolution);
    } else {
        if(!solveOperatingPoint(operatingPointOptions)){
            transientStats_.initializationCpuSeconds =
                double(std::clock() - startClock) / CLOCKS_PER_SEC;
            transientStats_.cpuSeconds =
                transientStats_.initializationCpuSeconds;
            return false;
        }
        initialSolution = mna_->solution();
    }

    integrator.Initialize(time, initialSolution);

    transientStats_.initializationCpuSeconds =
        double(std::clock() - startClock) / CLOCKS_PER_SEC;

    double nextOutputTime = config.outputStartTime;
    if(timeReached(time, nextOutputTime)){
        recordTransientSample(time);
        if(!advanceOutputTime(
            nextOutputTime,
            time,
            config.outputInterval
        )){
            transientStats_.cpuSeconds =
                double(std::clock() - startClock) / CLOCKS_PER_SEC;
            return false;
        }
    }

    double proposedStep = maximumIntegrationStep;

    const auto failTransient = [&] {
        transientStats_.finalTime = time;
        transientStats_.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        return false;
    };

    while(!timeReached(time, config.stopTime)){
        double hardStepLimit = std::min(
            {
                maximumIntegrationStep,
                nextOutputTime - time,
                config.stopTime - time
            }
        );

        const bool hasBdf2History = integrator.olderSolution() != nullptr;
        if(hasBdf2History){
            const double bdf2StepLimit = std::nextafter(
                integrator.previousStep() * integrator.maximumBdf2StepRatio(),
                0.0
            );
            hardStepLimit = std::min(hardStepLimit, bdf2StepLimit);
        }

        if(!std::isfinite(hardStepLimit) || hardStepLimit <= 0.0){
            return failTransient();
        }

        double candidateStep = std::min(proposedStep, hardStepLimit);
        if(!std::isfinite(candidateStep) ||
           candidateStep <= 0.0 ||
           time + candidateStep <= time){
            return failTransient();
        }

        int rejectedAttempts = 0;

        while(true){
            const double nextTime = time + candidateStep;
            if(!std::isfinite(nextTime) || nextTime <= time){
                return failTransient();
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
                    return failTransient();
                }

                TransientStepAttempt coarse = runTransientAttempt(
                    integrator,
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
                        attempt = secondHalf;

                        if(secondHalf.converged &&
                           secondHalf.integrationOrder == 1){
                            const TransientErrorEstimate estimate =
                                estimateTransientSolutionDifference(
                                    acceptedSolution,
                                    secondHalf.solution,
                                    coarse.solution,
                                    nodeMap_->nodeCount(),
                                    config.solverOptions
                                );

                            // Commit the fine endpoint, never the coarse
                            // whole-step solution or a Richardson extrapolate.
                            attempt.solution = secondHalf.solution;
                            attempt.errorEstimateValid = estimate.valid;
                            attempt.normalizedError =
                                estimate.normalizedError;
                            attempt.suggestedStepScale =
                                estimate.suggestedScale;
                        }
                    }
                }
            } else {
                attempt = runTransientAttempt(integrator, nextTime);
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
                integrator.accept(nextTime, attempt.solution);

                time = nextTime;
                if(timeReached(time, config.stopTime)){
                    time = config.stopTime;
                }
                ++transientStats_.timeSteps;

                proposedStep = std::min(
                    maximumIntegrationStep,
                    decision.nextStep
                );

                if(!std::isfinite(proposedStep) || proposedStep <= 0.0){
                    return failTransient();
                }

                if(timeReached(time, nextOutputTime)){
                    recordTransientSample(time);
                    if(!timeReached(time, config.stopTime) &&
                       !advanceOutputTime(
                           nextOutputTime,
                           time,
                           config.outputInterval
                       )){
                        return failTransient();
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

            return failTransient();
        }
    }

    if(transientSamples_.empty() ||
       !sameTime(transientSamples_.back().time, time)){
        recordTransientSample(time);
    }

    transientStats_.converged = true;
    transientStats_.finalTime = time;
    transientStats_.cpuSeconds =
        double(std::clock() - startClock) / CLOCKS_PER_SEC;
    return true;
}

bool Circuit::solveAdaptivePta(const PtaAnalysisConfig& config){
    operatingPointStats_ = {};
    operatingPointStats_.ptaAttempted = true;
    operatingPointStats_.maxIterations = config.newtonOptions.maximumIterations;
    operatingPointStats_.tolerance = config.newtonOptions.tolerance;
    operatingPointStats_.minSourceStep = config.minimumStep;
    operatingPointStats_.sourceScale = 1.0;

    const std::clock_t startClock = std::clock();
    const auto finish = [this, startClock](bool converged) {
        operatingPointStats_.converged = converged;
        operatingPointStats_.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        return converged;
    };

    TransientIntegrator integrator;
    double time = 0.0;
    double step = config.initialStep;

    initializePtaStates(config, time);

    integrator.Initialize(time, mna_->solution());

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
        if(!std::isfinite(candidateStep) ||
           candidateStep < config.minimumStep ||
           !std::isfinite(nextTime) ||
           nextTime <= time){
            return finish(false);
        }

        const Eigen::VectorXd acceptedSolution =
            integrator.currentSolution();

        const double sourceScale = ptaSourceRampScale(
            nextTime,
            config.sourceRampTime
        );
        if(!std::isfinite(sourceScale)){
            return finish(false);
        }
        setOperatingPointSourceScale(sourceScale);

        mna_->setSolution(integrator.predict(nextTime));
        saveNonlinearIterationStates();

        const TransientStampContext ctx =
            integrator.makeContext(nextTime);

        const AssembleCallback assemble = [this, &ctx] {
            assemblePtaSystem(ctx);
        };

        NewtonStats stats;
        const bool converged = hasNonlinearDevices()
            ? solveNewtonSystem(assemble, stats, config.newtonOptions)
            : solveLinearSystem(assemble, stats);
        addNewtonStats(stats);

        if(!converged){
            restoreTransientCheckpoint(acceptedSolution);

            const double reducedStep = std::max(
                config.minimumStep,
                candidateStep * config.failedStepScale
            );

            if(reducedStep < candidateStep){
                step = reducedStep;
                continue;
            }

            if(!growAllPtaNodeCapacitances(config)){
                return finish(false);
            }

            ++operatingPointStats_.ptaMinimumStepRecoveries;
            ++operatingPointStats_.ptaCapacitanceGrowths;
            integrator.Initialize(time, acceptedSolution);
            step = reducedStep;
            continue;
        }

        const Eigen::VectorXd currentSolution = mna_->solution();

        // BDF coefficients sum to zero.  Evaluate the derivative from state
        // differences so a settled solution does not lose precision through
        // cancellation among terms of order |x| / h.
        Eigen::VectorXd derivative =
            ctx.derivative.alpha0 *
            (currentSolution - ctx.previousSolution);
        if(ctx.olderSolution != nullptr){
            derivative += ctx.derivative.alpha2 *
                (*ctx.olderSolution - ctx.previousSolution);
        }
        if(!derivative.allFinite()){
            return finish(false);
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
            return finish(false);
        }

        // Test the original DC residual F(x), excluding artificial PTA terms
        // and always using the nominal independent-source values.  The ramp
        // is a convergence aid and must not redefine the target DC system.
        setOperatingPointSourceScale(1.0);
        assembleOperatingPointSystem();
        Eigen::VectorXd matrixProduct;
        Eigen::VectorXd residual;
        if(!mna_->evaluateResidual(matrixProduct, residual)){
            return finish(false);
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
            return finish(false);
        }

        operatingPointStats_.hasPtaConvergenceMetrics = true;
        operatingPointStats_.ptaNormalizedDerivative =
            derivativeEstimate.normalizedDerivative;
        operatingPointStats_.ptaNormalizedDcResidual =
            dcResidualEstimate.normalizedResidual;

        if(derivativeEstimate.normalizedDerivative <
               config.derivativeTolerance &&
           dcResidualEstimate.normalizedResidual <
               config.dcResidualTolerance){
            return finish(true);
        }

        updatePtaNodeCapacitancesAfterAcceptedStep(
            currentSolution,
            acceptedSolution,
            config
        );

        integrator.accept(nextTime, currentSolution);
        time = nextTime;
        step = std::min(
            config.maximumStep,
            candidateStep * config.successfulStepScale
        );
    }

    return finish(false);
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

void Circuit::addTransientStats(const NewtonStats& stats){
    transientStats_.iterations += stats.iterations;
    transientStats_.dampedSteps += stats.dampedSteps;
    transientStats_.finalDelta = stats.finalDelta;
}

void Circuit::restoreTransientCheckpoint(const Eigen::VectorXd& acceptedSolution){
    restoreNonlinearIterationStates();
    mna_->setSolution(acceptedSolution);
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

        mna_->setSolution(acceptedSolution);
        saveNonlinearIterationStates();
        setOperatingPointSourceScale(trialScale);

        NewtonStats trialStats;
        const bool converged = solveNewtonSystem(
            assemble,
            trialStats,
            options.newton
        );
        addNewtonStats(trialStats);

        if(converged){
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

        sourceStep *= options.sourceStepping.failureScale;
        if(sourceStep < options.sourceStepping.minimumStep){
            return false;
        }
    }

    mna_->setSolution(acceptedSolution);
    return true;
}

bool Circuit::solveLinearSystem(const AssembleCallback& assemble,
                                NewtonStats& stats){
    stats = {};
    assemble();

    if(!mna_->solve()){
        return false;
    }

    stats.iterations = 1;
    stats.finalDelta = 0.0;
    return true;
}

bool Circuit::solveNewtonSystem(const AssembleCallback& assemble,
                                NewtonStats& stats,
                                const NewtonSolverOptions& options){
    stats = {};

    Eigen::VectorXd previous = mna_->solution();

    for(int iter = 0; iter < options.maximumIterations; ++iter){
        stats.iterations = iter + 1;
        assemble();

        if(!mna_->solve()){
            return false;
        }

        Eigen::VectorXd& current = mna_->solution();
        const NewtonStepResult step = limitNewtonStep(
            current,
            previous,
            options.maximumSolutionStep
        );
        if(!std::isfinite(step.delta)){
            return false;
        }
        if(step.limited){
            ++stats.dampedSteps;
        }

        stats.finalDelta = step.delta;
        if(step.delta < options.tolerance){
            return true;
        }

        previous = current;
    }

    return false;
}

void Circuit::addNewtonStats(const NewtonStats& stats){
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
