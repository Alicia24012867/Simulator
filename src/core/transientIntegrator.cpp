#include "../../include/core/transientIntegrator.h"

#include <cassert>

TransientIntegrator::TransientIntegrator(
    double maximumBdf2StepRatio
): maximumBdf2StepRatio_(maximumBdf2StepRatio) {}

void TransientIntegrator::Initialize(
    double initialTime,
    const Eigen::VectorXd& initialSolution
){
    acceptedTime_ = initialTime;
    solutionN_ = initialSolution;

    initialized_ = true;
}

void TransientIntegrator::restartFrom(
    double time,
    const Eigen::VectorXd& solution
){
    acceptedTime_ = time;
    solutionN_ = solution;

    initialized_ = true;
    hasOlderSolution_ = false;
}

bool TransientIntegrator::canUseBdf2(double targetTime) const{
    if(!initialized_ || !hasOlderSolution_ || previousStep_ <= 0.0) return false;

    assert(targetTime > acceptedTime_);
    double ratio = (targetTime - acceptedTime_) / previousStep_;
    if(ratio > maximumBdf2StepRatio_)   return false;

    return true;
}

TransientDerivativeCoefficients TransientIntegrator::coefficients(double targetTime) const {
    const double dt = targetTime - acceptedTime_;

    if(!canUseBdf2(targetTime)){
        return {
            1,
            1.0 / dt,
            -1.0 / dt,
            0
        };
    }

    return {
        2,
        3.0 / (2.0 * dt),
        -2.0 / dt,
        1.0 / (2.0 * dt)
    };
}

Eigen::VectorXd TransientIntegrator::predict(double targetTime) const{
    if(!canUseBdf2(targetTime)){
        return solutionN_;
    }

    double ratio = (targetTime - acceptedTime_) / previousStep_;
    Eigen::VectorXd delta = solutionN_ - solutionNm1_;
    return solutionN_ + ratio * delta;
}

TransientStampContext TransientIntegrator::makeContext(double targetTime) const{
    return{
        targetTime,
        targetTime - acceptedTime_,
        coefficients(targetTime),
        solutionN_,
        hasOlderSolution_ ? &solutionNm1_ : nullptr
    };
}

void TransientIntegrator::accept(
    double acceptedTime,
    const Eigen::VectorXd& acceptedSolution
){
    acceptedTime_ = acceptedTime;
    solutionNm1_ = solutionN_;
    solutionN_ = acceptedSolution;

    hasOlderSolution_ = true;
}

double TransientIntegrator::acceptedTime() const noexcept{
    return acceptedTime_;
}

double TransientIntegrator::previousStep() const noexcept{
    return previousStep_;
}

const Eigen::VectorXd& TransientIntegrator::currentSolution() const noexcept{
    return solutionN_;
}

const Eigen::VectorXd* TransientIntegrator::olderSolution() const noexcept{
    return &solutionNm1_;
}