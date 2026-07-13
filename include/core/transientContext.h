#pragma once

#include <cassert>
#include <Eigen/Core>

struct TransientStampContext{
    double targetTime = 0.0;
    double timeStep = 0.0;
    const Eigen::VectorXd& previousSolution;

    double previousSolutionVal(int idx) const {
        if(idx < 0) return 0.0;
        assert(idx < previousSolution.size());
        return previousSolution[idx];
    }
};