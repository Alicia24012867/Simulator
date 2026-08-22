#include "circuit/circuit.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <string>

#include "analysis/solver_options.hpp"
#include "analysis/transient_analysis.hpp"
#include "circuit/node_map.hpp"
#include "devices/device.hpp"
#include "devices/pseudo_device.hpp"
#include "solver/newton_convergence.hpp"
#include "solver/mna.hpp"
#include "solver/newton_step.hpp"

namespace {
using SteadyClock = std::chrono::steady_clock;

double elapsedWallSeconds(SteadyClock::time_point start){
    return std::chrono::duration<double>(SteadyClock::now() - start).count();
}
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
    Eigen::VectorXd matrixProduct;
    Eigen::VectorXd residual;
    const int voltageUnknownCount = nodeMap_->nodeCount();

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
        const NewtonUpdateEstimate updateEstimate =
            estimateNormalizedNewtonUpdate(
                current,
                previous,
                voltageUnknownCount,
                options.relativeTolerance,
                options.voltageAbsoluteTolerance,
                options.currentAbsoluteTolerance
            );
        if(!updateEstimate.valid){
            return finish(
                false,
                "Newton-Raphson normalized update estimate is invalid"
            );
        }
        stats.hasNormalizedUpdate = true;
        stats.normalizedUpdate = updateEstimate.normalizedUpdate;

        // Reassemble at the candidate solution.  The resulting A*x-b is the
        // nonlinear defect F(x), unlike the zero residual of the just-solved
        // linearization.  Avoid this extra stamp until the update is close
        // enough to be a possible convergence candidate.
        if(updateEstimate.normalizedUpdate <
           options.normalizedUpdateTolerance){
            assemble();
            if(!mna_->evaluateResidual(matrixProduct, residual)){
                return finish(
                    false,
                    "Newton-Raphson nonlinear residual contains a non-finite value"
                );
            }
            const NewtonResidualEstimate residualEstimate =
                estimateNormalizedNewtonResidual(
                    residual,
                    matrixProduct,
                    mna_->rhs(),
                    voltageUnknownCount,
                    options.relativeTolerance,
                    options.voltageAbsoluteTolerance,
                    options.currentAbsoluteTolerance
                );
            if(!residualEstimate.valid){
                return finish(
                    false,
                    "Newton-Raphson normalized residual estimate is invalid"
                );
            }
            stats.hasNormalizedResidual = true;
            stats.normalizedResidual = residualEstimate.normalizedResidual;
            if(residualEstimate.normalizedResidual <
               options.normalizedResidualTolerance){
                return finish(true);
            }
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
