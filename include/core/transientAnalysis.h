#pragma once

#include <cassert>
#include <optional>

#include <Eigen/Core>

struct TransientSolverOptions {
    double relativeTolerance = 1.0e-4;
    double voltageAbsoluteTolerance = 1.0e-6;
    double currentAbsoluteTolerance = 1.0e-9;

    double minimumStep = 1.0e-15;

    double safetyFactor = 0.9;
    double minimumScale = 0.2;
    double maximumScale = 2.0;
    double convergenceFailureScale = 0.25;

    int maximumRejects = 10;
};

// Parsed .tran parameters plus transient solver configuration.
struct TransientAnalysisConfig {
    double outputInterval = 0.0;  // TSTEP
    double stopTime = 0.0;        // TSTOP
    double outputStartTime = 0.0; // TSTART
    std::optional<double> maximumStep; // TMAX
    bool useInitialConditions = false; // UIC
    TransientSolverOptions solverOptions;
};

struct TransientDerivativeCoefficients {
    int order = 1;
    double alpha0 = 0.0;
    double alpha1 = 0.0;
    double alpha2 = 0.0;
};

struct TransientStampContext {
    double targetTime = 0.0;
    double timeStep = 0.0;
    TransientDerivativeCoefficients derivative;
    const Eigen::VectorXd& previousSolution;
    const Eigen::VectorXd* olderSolution = nullptr;

    double previousSolutionVal(int idx) const {
        if(idx < 0){
            return 0.0;
        }
        assert(idx < previousSolution.size());
        return previousSolution[idx];
    }

    double olderSolutionVal(int idx) const {
        if(idx < 0){
            return 0.0;
        }
        assert(olderSolution != nullptr);
        assert(idx < olderSolution->size());
        return (*olderSolution)[idx];
    }

    double historyDerivativeVal(int idx) const {
        double history = derivative.alpha1 * previousSolutionVal(idx);
        if(derivative.alpha2 != 0.0){
            history += derivative.alpha2 * olderSolutionVal(idx);
        }
        return history;
    }

    double historyDerivativeDifference(int p, int n) const {
        return historyDerivativeVal(p) - historyDerivativeVal(n);
    }
};

class TransientIntegrator {
public:
    explicit TransientIntegrator(
        double maximumBdf2StepRatio = 2.0
    ): maximumBdf2StepRatio_(maximumBdf2StepRatio) {}

    void Initialize(
        double initialTime,
        const Eigen::VectorXd& initialSolution
    ){
        acceptedTime_ = initialTime;
        previousStep_ = 0.0;
        solutionN_ = initialSolution;
        initialized_ = true;
        hasOlderSolution_ = false;
    }

    void restartFrom(
        double time,
        const Eigen::VectorXd& solution
    ){
        Initialize(time, solution);
    }

    bool canUseBdf2(double targetTime) const {
        if(!initialized_ || !hasOlderSolution_ || previousStep_ <= 0.0){
            return false;
        }

        const double dt = targetTime - acceptedTime_;
        if(dt <= 0.0){
            return false;
        }

        return dt / previousStep_ <= maximumBdf2StepRatio_;
    }

    TransientDerivativeCoefficients coefficients(double targetTime) const {
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
        const double denominator = (1.0 + ratio) * dt;
        return {
            2,
            (1.0 + 2.0 * ratio) / denominator,
            -(1.0 + ratio) / dt,
            ratio * ratio / denominator
        };
    }

    Eigen::VectorXd predict(double targetTime) const {
        if(!canUseBdf2(targetTime)){
            return solutionN_;
        }

        const double ratio =
            (targetTime - acceptedTime_) / previousStep_;
        return solutionN_ + ratio * (solutionN_ - solutionNm1_);
    }

    TransientStampContext makeContext(double targetTime) const {
        return {
            targetTime,
            targetTime - acceptedTime_,
            coefficients(targetTime),
            solutionN_,
            hasOlderSolution_ ? &solutionNm1_ : nullptr
        };
    }

    void accept(
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

    double acceptedTime() const noexcept {
        return acceptedTime_;
    }

    double previousStep() const noexcept {
        return previousStep_;
    }

    const Eigen::VectorXd& currentSolution() const noexcept {
        return solutionN_;
    }

    const Eigen::VectorXd* olderSolution() const noexcept {
        return hasOlderSolution_ ? &solutionNm1_ : nullptr;
    }

private:
    double acceptedTime_ = 0.0;
    double previousStep_ = 0.0;
    Eigen::VectorXd solutionN_;
    Eigen::VectorXd solutionNm1_;
    bool hasOlderSolution_ = false;
    bool initialized_ = false;
    double maximumBdf2StepRatio_ = 2.0;
};
