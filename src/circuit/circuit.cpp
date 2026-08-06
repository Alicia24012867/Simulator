#include "circuit/circuit.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <limits>

#include "analysis/analysisPlan.h"
#include "analysis/transientAnalysis.h"
#include "circuit/nodeMap.h"
#include "devices/device.hpp"
#include "math/mna.hpp"
#include "math/newtonStep.hpp"
#include "models/model.hpp"

namespace {
constexpr int kMaxNewtonIterations = 1000;
constexpr double kNewtonTolerance = 1.0e-9;
constexpr double kSolutionMaxStep = 1.0;
constexpr double kInitialSourceStep = 0.1;
constexpr double kMaxSourceStep = 0.25;
constexpr double kMinSourceStep = 1.0e-4;
constexpr double kSourceStepGrowth = 1.5;
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

int Circuit::allocateUnknown(){
    return nextUnknown_++;
}

bool Circuit::build(){
    nodeMap_->build(devices_);

    for(auto& device: devices_){
        device->bindNodes(*nodeMap_);
    }

    cacheOperatingPointDeviceRoles();

    nextUnknown_ = nodeMap_->nodeCount();

    for(auto& device: devices_){
        device->allocateUnknown(*this);
    }

    mna_->resize(nextUnknown_);
    mna_->reservePattern(
        devices_.size() * 12 + static_cast<std::size_t>(nextUnknown_)
    );

    for(auto& device: devices_){
        device->pattern(*mna_);
    }

    mna_->build();

    for(auto& device: devices_){
        device->bindMatrix(*mna_);
    }
    mna_->releaseBuildMetadata();

    return true;
}

bool Circuit::solve(){
    return solveOperatingPoint();
}

bool Circuit::solveOperatingPoint(){
    operatingPointStats_ = {};
    operatingPointStats_.maxIterations = kMaxNewtonIterations;
    operatingPointStats_.tolerance = kNewtonTolerance;
    operatingPointStats_.minSourceStep = kMinSourceStep;

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
    const bool directConverged = solveNewtonSystem(assemble, directStats);
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

    if(!solveOperatingPointWithSourceStepping(assemble)){
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
    transientStats_ = {};
    transientStats_.maxIterations = kMaxNewtonIterations;
    transientStats_.tolerance = kNewtonTolerance;
    transientSamples_.clear();

    const std::clock_t startClock = std::clock();
    TransientIntegrator integrator;
    double time = 0.0;

    const double maximumIntegrationStep = config.maximumStep
        ? *config.maximumStep
        : config.outputInterval;

    if(!std::isfinite(config.outputInterval) ||
       !std::isfinite(config.stopTime) ||
       !std::isfinite(config.outputStartTime) ||
       !std::isfinite(maximumIntegrationStep) ||
       config.outputInterval <= 0.0 ||
       maximumIntegrationStep <= 0.0 ||
       config.outputStartTime < 0.0 ||
       config.outputStartTime >= config.stopTime ||
       config.stopTime <= 0.0 ||
       !config.solverOptions.validFor(maximumIntegrationStep)){
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
        if(!solveOperatingPoint()){
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
        ? solveNewtonSystem(assemble, attempt.newtonStats)
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
    const AssembleCallback& assemble)
{
    Eigen::VectorXd acceptedSolution = mna_->solution();
    double acceptedScale = 0.0;
    double sourceStep = kInitialSourceStep;

    while(acceptedScale < kSourceScaleDone){
        const double trialScale = std::min(1.0, acceptedScale + sourceStep);

        mna_->setSolution(acceptedSolution);
        saveNonlinearIterationStates();
        setOperatingPointSourceScale(trialScale);

        NewtonStats trialStats;
        const bool converged = solveNewtonSystem(assemble, trialStats);
        addNewtonStats(trialStats);

        if(converged){
            acceptedScale = trialScale;
            acceptedSolution = mna_->solution();
            operatingPointStats_.sourceScale = acceptedScale;
            ++operatingPointStats_.sourceSteps;
            sourceStep = std::min(
                kMaxSourceStep,
                sourceStep * kSourceStepGrowth
            );
            continue;
        }

        restoreNonlinearIterationStates();
        mna_->setSolution(acceptedSolution);
        setOperatingPointSourceScale(acceptedScale);
        ++operatingPointStats_.failedSourceSteps;

        sourceStep *= 0.5;
        if(sourceStep < kMinSourceStep){
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
                                NewtonStats& stats){
    stats = {};

    Eigen::VectorXd previous = mna_->solution();

    for(int iter = 0; iter < kMaxNewtonIterations; ++iter){
        stats.iterations = iter + 1;
        assemble();

        if(!mna_->solve()){
            return false;
        }

        Eigen::VectorXd& current = mna_->solution();
        const NewtonStepResult step = limitNewtonStep(
            current,
            previous,
            kSolutionMaxStep
        );
        if(!std::isfinite(step.delta)){
            return false;
        }
        if(step.limited){
            ++stats.dampedSteps;
        }

        stats.finalDelta = step.delta;
        if(step.delta < kNewtonTolerance){
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
