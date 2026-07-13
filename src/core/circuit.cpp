#include "../include/core/circuit.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <ostream>

#include "../include/core/nodeMap.h"
#include "../include/devices/device.hpp"
#include "../include/math/mna.hpp"
#include "../include/models/model.hpp"

namespace {
constexpr int kMaxNewtonIterations = 1000;
constexpr double kNewtonTolerance = 1.0e-9;
constexpr double kSolutionMaxStep = 1.0;
constexpr double kInitialSourceStep = 0.1;
constexpr double kMaxSourceStep = 0.25;
constexpr double kMinSourceStep = 1.0e-4;
constexpr double kSourceStepGrowth = 1.5;
constexpr double kSourceScaleDone = 1.0 - 1.0e-12;
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

void Circuit::printOperatingPoint(std::ostream& os) const{
    os << "Operating Point\n";
    os << std::scientific << std::setprecision(10);

    os << "Newton Info\n";
    os << "converged " << (operatingPointStats_.converged ? "yes" : "no") << '\n';
    os << "iterations " << operatingPointStats_.iterations << '\n';
    os << "max_iterations " << operatingPointStats_.maxIterations << '\n';
    os << "final_delta " << operatingPointStats_.finalDelta << '\n';
    os << "tolerance " << operatingPointStats_.tolerance << '\n';
    os << "damped_steps " << operatingPointStats_.dampedSteps << '\n';
    os << "source_steps " << operatingPointStats_.sourceSteps << '\n';
    os << "failed_source_steps " << operatingPointStats_.failedSourceSteps << '\n';
    os << "source_scale " << operatingPointStats_.sourceScale << '\n';
    os << "min_source_step " << operatingPointStats_.minSourceStep << '\n';
    os << "cpu_time_seconds " << operatingPointStats_.cpuSeconds << '\n';

    const auto& names = nodeMap_->nodeNameByIdx();
    const auto& x = mna_->solution();

    os << "Node Voltages\n";
    for(std::size_t i = 0; i < names.size(); ++i){
        os << "v(" << names[i] << ") " << x[static_cast<int>(i)] << '\n';
    }

    os << "Branch Currents\n";
    for(const auto& device: devices_){
        const int branch = device->branchUnknown();
        if(branch >= 0){
            os << "i(" << device->getName() << ") " << x[branch] << '\n';
        }
    }
}
