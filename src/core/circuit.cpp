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
    mna(std::make_unique<MNA>()),
    nodeMap(std::make_unique<NodeMap>()) {}

Circuit::~Circuit() = default;

const Model* Circuit::addModel(std::unique_ptr<Model> model){
    const std::string name = model->name();
    auto& slot = models[name];
    slot = std::move(model);
    return slot.get();
}

const Model* Circuit::findModel(const std::string& name) const{
    auto it = models.find(name);
    return it == models.end() ? nullptr : it->second.get();
}

int Circuit::allocateUnknown(){
    return nextUnknown++;
}

bool Circuit::build(){
    nodeMap->build(devices);

    for(auto& d: devices){
        d->bindNodes(*nodeMap);
    }

    nextUnknown = nodeMap->nodeCount();

    for(auto& d: devices){
        d->allocateUnknown(*this);
    }

    mna->resize(nextUnknown);

    for(auto& d: devices){
        d->pattern(*mna);
    }

    mna->build();

    for(auto&d: devices){
        d->bindMatrix(*mna);
    }

    return true;
}

bool Circuit::solve(){
    solveStats = {};
    solveStats.maxIterations = kMaxNewtonIterations;
    solveStats.tolerance = kNewtonTolerance;
    solveStats.minSourceStep = kMinSourceStep;

    const std::clock_t startClock = std::clock();
    Eigen::VectorXd acceptedSolution = mna->solution();
    double acceptedScale = 0.0;
    double sourceStep = kInitialSourceStep;

    while(acceptedScale < kSourceScaleDone){
        const double trialScale = std::min(1.0, acceptedScale + sourceStep);

        mna->setSolution(acceptedSolution);
        saveDeviceStates();
        setSourceScale(trialScale);

        NewtonStats trialStats;
        const bool converged = solveNewton(trialStats);

        solveStats.iterations += trialStats.iterations;
        solveStats.dampedSteps += trialStats.dampedSteps;
        solveStats.finalDelta = trialStats.finalDelta;

        if(converged){
            acceptedScale = trialScale;
            acceptedSolution = mna->solution();
            solveStats.sourceScale = acceptedScale;
            ++solveStats.sourceSteps;
            sourceStep = std::min(kMaxSourceStep, sourceStep * kSourceStepGrowth);
            continue;
        }

        restoreDeviceStates();
        mna->setSolution(acceptedSolution);
        setSourceScale(acceptedScale);
        ++solveStats.failedSourceSteps;

        sourceStep *= 0.5;
        if(sourceStep < kMinSourceStep){
            solveStats.cpuSeconds = double(std::clock() - startClock) / CLOCKS_PER_SEC;
            return false;
        }
    }

    setSourceScale(1.0);
    mna->setSolution(acceptedSolution);
    solveStats.converged = true;
    solveStats.cpuSeconds = double(std::clock() - startClock) / CLOCKS_PER_SEC;
    return true;
}

bool Circuit::solveNewton(NewtonStats& stats){
    stats = {};

    Eigen::VectorXd previous = mna->solution();

    for(int iter = 0; iter < kMaxNewtonIterations; ++iter){
        stats.iterations = iter + 1;
        mna->clear();

        for(auto& d: devices){
            d->stamp();
        }

        if(!mna->solve()){
            return false;
        }

        Eigen::VectorXd current = mna->solution();
        if(current.size() != previous.size()){
            return false;
        }

        Eigen::VectorXd step = current - previous;
        const double rawDelta = step.lpNorm<Eigen::Infinity>();
        if(rawDelta > kSolutionMaxStep){
            current = previous + step * (kSolutionMaxStep / rawDelta);
            mna->setSolution(current);
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

void Circuit::setSourceScale(double scale){
    for(auto& d: devices){
        d->setSourceScale(scale);
    }
}

void Circuit::saveDeviceStates(){
    for(auto& d: devices){
        d->saveState();
    }
}

void Circuit::restoreDeviceStates(){
    for(auto& d: devices){
        d->restoreState();
    }
}

void Circuit::printOperatingPoint(std::ostream& os) const{
    os << "Operating Point\n";
    os << std::scientific << std::setprecision(10);

    os << "Newton Info\n";
    os << "converged " << (solveStats.converged ? "yes" : "no") << '\n';
    os << "iterations " << solveStats.iterations << '\n';
    os << "max_iterations " << solveStats.maxIterations << '\n';
    os << "final_delta " << solveStats.finalDelta << '\n';
    os << "tolerance " << solveStats.tolerance << '\n';
    os << "damped_steps " << solveStats.dampedSteps << '\n';
    os << "source_steps " << solveStats.sourceSteps << '\n';
    os << "failed_source_steps " << solveStats.failedSourceSteps << '\n';
    os << "source_scale " << solveStats.sourceScale << '\n';
    os << "min_source_step " << solveStats.minSourceStep << '\n';
    os << "cpu_time_seconds " << solveStats.cpuSeconds << '\n';

    const auto& names = nodeMap->nodeNameByIdx();
    const auto& x = mna->solution();

    os << "Node Voltages\n";
    for(std::size_t i = 0; i < names.size(); ++i){
        os << "v(" << names[i] << ") " << x[static_cast<int>(i)] << '\n';
    }

    os << "Branch Currents\n";
    for(const auto& device: devices){
        const int branch = device->branchUnknown();
        if(branch >= 0){
            os << "i(" << device->getName() << ") " << x[branch] << '\n';
        }
    }
}
