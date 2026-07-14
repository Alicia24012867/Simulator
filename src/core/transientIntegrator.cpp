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
    previousStep_ = 0.0;
    solutionN_ = initialSolution;

    initialized_ = true;
    hasOlderSolution_ = false;
}

void TransientIntegrator::restartFrom(
    double time,
    const Eigen::VectorXd& solution
){
    Initialize(time, solution);
}

bool TransientIntegrator::canUseBdf2(double targetTime) const{
    if(!initialized_ || !hasOlderSolution_ || previousStep_ <= 0.0) return false;

    const double dt = targetTime - acceptedTime_;
    if(dt <= 0.0)   return false;

    double ratio = dt / previousStep_;
    if(ratio > maximumBdf2StepRatio_)   return false;

    return true;
}

TransientDerivativeCoefficients TransientIntegrator::coefficients(double targetTime) const {
    const double dt = targetTime - acceptedTime_;

    assert(initialized_);
    assert(dt > 0.0);

    if(!canUseBdf2(targetTime)){
        return {
            1,
            1.0 / dt,
            -1.0 / dt,
            0.0
        };
    }

    const double ratio = dt / previousStep_;
    const double dominator = (1.0 + ratio) * dt;

    return {
        2,
        (1.0 + 2.0 * ratio) / dominator,
        -(1.0 + ratio) / dt,
        ratio * ratio / dominator
    };
}

Eigen::VectorXd TransientIntegrator::predict(double targetTime) const{
    if(!canUseBdf2(targetTime)){
        return solutionN_;
    }

    const double ratio = (targetTime - acceptedTime_) / previousStep_;

    Eigen::VectorXd prediction = solutionN_ - solutionNm1_;
    prediction *= ratio;
    prediction += solutionN_;
    return prediction;
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
    double nextAcceptedTime,
    const Eigen::VectorXd& acceptedSolution
){
    assert(initialized_);
    assert(nextAcceptedTime > acceptedTime_);
    assert(solutionN_.size() == acceptedSolution.size());

    previousStep_ = nextAcceptedTime - acceptedTime_;
    acceptedTime_ = nextAcceptedTime;
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
    return hasOlderSolution_ ? &solutionNm1_ : nullptr;
}