#pragma once

#include <cassert>
#include <Eigen/Core>


struct TransientDerivativeCoefficients{
    int order = 1;

    double alpha0 = 0.0;
    double alpha1 = 0.0;
    double alpha2 = 0.0;
};
struct TransientStampContext{
    double targetTime = 0.0;
    double timeStep = 0.0;

    TransientDerivativeCoefficients derivative;

    const Eigen::VectorXd& previousSolution;
    const Eigen::VectorXd* olderSolution = nullptr;

    double previousSolutionVal(int idx) const {
        if(idx < 0) return 0.0;
        assert(idx < previousSolution.size());
        return previousSolution[idx];
    }

    double olderSolutionVal(int idx) const {
        if(idx < 0) return 0.0;
        assert(olderSolution != nullptr);
        assert(idx < olderSolution->size());
        return (*olderSolution)[idx];
    }

    double historyDerivativeVal(int idx) const {
        double history = derivative.alpha1 * previousSolutionVal(idx);

        if(derivative.alpha2 == 0)  return history;

        history += derivative.alpha2 * olderSolutionVal(idx);
        return history;
    }

    double historyDerivativeDifference(int p, int n) const {
        return historyDerivativeVal(p)
            - historyDerivativeVal(n);
    }
};