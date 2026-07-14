#pragma once

#include <Eigen/Core>

#include "transientContext.hpp"

class TransientIntegrator{
public:
    explicit TransientIntegrator(
        double maximumBdf2StepRatio = 2.0
    );

    void Initialize(
        double initialTime,
        const Eigen::VectorXd& initialSolution
    );

    void restartFrom(
        double time,
        const Eigen::VectorXd& solution
    );

    bool canUseBdf2(double targetTime) const;

    TransientDerivativeCoefficients coefficients(double targetTime) const;

    // only used in the initial guess.
    Eigen::VectorXd predict(double targetTime) const;

    TransientStampContext makeContext(double targetTime) const;

    void accept(
        double nextAcceptedTime,
        const Eigen::VectorXd& acceptedSolution
    );

    double acceptedTime() const noexcept;
    double previousStep() const noexcept;

    const Eigen::VectorXd& currentSolution() const noexcept;
    const Eigen::VectorXd* olderSolution() const noexcept;

private:
    double acceptedTime_ = 0.0;     // time for x_n
    double previousStep_ = 0.0;
    
    Eigen::VectorXd solutionN_;     // x_n
    Eigen::VectorXd solutionNm1_;   // x_{n-1} m means minus

    bool hasOlderSolution_ = false; // whether x_{n-1} exists
    bool initialized_ = false;
    double maximumBdf2StepRatio_ = 2.0; // 最大步长增大比例
};