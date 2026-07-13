#include "../include/core/circuit.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <limits>

#include "../include/core/nodeMap.h"
#include "../include/devices/device.hpp"
#include "../include/math/mna.hpp"
#include "../include/models/model.hpp"
#include "../include/core/analysisPlan.h"
#include "../include/core/transientContext.h"

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
    Eigen::VectorXd previousSolution;
    double time = 0.0;

    const double maximumIntegrationStep = config.maximumStep
        ? *config.maximumStep
        : config.outputInterval;

    if(config.outputInterval <= 0.0 ||
       maximumIntegrationStep <= 0.0 ||
       config.outputStartTime < 0.0 ||
       config.outputStartTime >= config.stopTime ||
       config.stopTime <= 0.0){
        transientStats_.cpuSeconds =
            double(std::clock() - startClock) / CLOCKS_PER_SEC;
        return false;
    }

    setOperatingPointSourceScale(1.0);
    if(config.useInitialConditions){
        previousSolution = Eigen::VectorXd::Zero(mna_->size());
        mna_->setSolution(previousSolution);
    } else {
        if(!solveOperatingPoint()){
            transientStats_.initializationCpuSeconds =
                double(std::clock() - startClock) / CLOCKS_PER_SEC;
            transientStats_.cpuSeconds =
                transientStats_.initializationCpuSeconds;
            return false;
        }
        previousSolution = mna_->solution();
    }
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

    while(time < config.stopTime){
        const double nextTime = std::min(
            {
                time + maximumIntegrationStep,
                nextOutputTime,
                config.stopTime
            }
        );
        const double step = nextTime - time;

        if(step <= 0.0){
            transientStats_.finalTime = time;
            transientStats_.cpuSeconds =
                double(std::clock() - startClock) / CLOCKS_PER_SEC;
            return false;
        }

        mna_->setSolution(previousSolution);

        const TransientStampContext ctx{
            nextTime,
            step,
            previousSolution
        };

        const AssembleCallback assemble = [this, &ctx]{
            assembleTransientSystem(ctx);
        };

        NewtonStats stats;
        const bool solved = hasNonlinearDevices()
            ? solveNewtonSystem(assemble, stats)
            : solveLinearSystem(assemble, stats);

        transientStats_.iterations += stats.iterations;
        transientStats_.dampedSteps += stats.dampedSteps;
        transientStats_.finalDelta = stats.finalDelta;

        if(!solved){
            transientStats_.finalTime = time;
            transientStats_.cpuSeconds =
                double(std::clock() - startClock) / CLOCKS_PER_SEC;
            return false;
        }

        previousSolution = mna_->solution();
        time = nextTime;
        ++transientStats_.timeSteps;

        if(timeReached(time, nextOutputTime)){
            recordTransientSample(time);
            if(time < config.stopTime &&
               !advanceOutputTime(
                   nextOutputTime,
                   time,
                   config.outputInterval
               )){
                transientStats_.finalTime = time;
                transientStats_.cpuSeconds =
                    double(std::clock() - startClock) / CLOCKS_PER_SEC;
                return false;
            }
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

        Eigen::VectorXd current = mna_->solution();
        if(current.size() != previous.size()){
            return false;
        }

        const Eigen::VectorXd step = current - previous;
        const double rawDelta = step.lpNorm<Eigen::Infinity>();
        if(rawDelta > kSolutionMaxStep){
            current = previous + step * (kSolutionMaxStep / rawDelta);
            mna_->setSolution(current);
            ++stats.dampedSteps;
        }

        const double delta = (current - previous).lpNorm<Eigen::Infinity>();
        stats.finalDelta = delta;
        if(delta < kNewtonTolerance){
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
