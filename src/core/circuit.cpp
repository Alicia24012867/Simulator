#include "../include/core/circuit.h"

#include <ctime>
#include <iomanip>
#include <ostream>

#include "../include/core/nodeMap.h"
#include "../include/devices/device.hpp"
#include "../include/math/mna.hpp"
#include "../include/models/model.hpp"

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
    constexpr int maxIterations = 150;
    constexpr double tolerance = 1.0e-9;
    constexpr double maxStep = 1.0;

    solveStats = {};
    solveStats.maxIterations = maxIterations;
    solveStats.tolerance = tolerance;

    const std::clock_t startClock = std::clock();
    Eigen::VectorXd previous = mna->solution();

    for(int iter = 0; iter < maxIterations; ++iter){
        solveStats.iterations = iter + 1;
        mna->clear();

        for(auto& d: devices){
            d->stamp();
        }

        if(!mna->solve()){
            solveStats.cpuSeconds = double(std::clock() - startClock) / CLOCKS_PER_SEC;
            return false;
        }

        Eigen::VectorXd current = mna->solution();
        if(current.size() != previous.size()){
            return false;
        }

        Eigen::VectorXd step = current - previous;
        const double rawDelta = step.lpNorm<Eigen::Infinity>();
        if(rawDelta > maxStep){
            current = previous + step * (maxStep / rawDelta);
            mna->setSolution(current);
            ++solveStats.dampedSteps;
        }

        const double delta = (current - previous).lpNorm<Eigen::Infinity>();
        solveStats.finalDelta = delta;
        if(delta < tolerance){
            solveStats.converged = true;
            solveStats.cpuSeconds = double(std::clock() - startClock) / CLOCKS_PER_SEC;
            return true;
        }

        previous = current;
    }

    solveStats.cpuSeconds = double(std::clock() - startClock) / CLOCKS_PER_SEC;
    return false;
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
