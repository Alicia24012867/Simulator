#pragma once

#include <cassert>

#include "device.hpp"
#include "../core/circuit.h"
#include "../math/mna.hpp"
#include "../core/transientContext.h"


class Inductor: public Device{
public:
    Inductor(std::string name, std::vector<std::string> nodes, double L):
                Device(name, nodes, DeviceType::Inductor), inductance_(L) {
                    assert(L > 0.0);
                }

    void allocateUnknown(Circuit& circuit) override{
        branch = circuit.allocateUnknown();
    }

    int branchUnknown() const override{
        return branch;
    }

    void pattern(MNA& mna) override{
        assert(branch >=0);

        const int p = nodeIds[0];
        const int n = nodeIds[1];

        if(p >=0){
            mna.addPattern(p, branch);
            mna.addPattern(branch, p);
        }
        if(n >= 0){
            mna.addPattern(n,branch);
            mna.addPattern(branch,n);
        }

        mna.addPattern(branch, branch);
    }

    void bindMatrix(MNA& mna) override{
        const int p = nodeIds[0];
        const int n = nodeIds[1];

        if(p >= 0){
            posBranch = mna.ptr(p, branch);
            branchPos = mna.ptr(branch, p);
        }
        if(n >= 0){
            negBranch = mna.ptr(n, branch);
            branchNeg = mna.ptr(branch, n);
        }

        bb = mna.ptr(branch, branch);
        rhs = &mna.rhs(branch);
    }

    void stampOperatingPoint() override {
        if(posBranch)    *posBranch += 1.0;
        if(negBranch)    *negBranch -= 1.0;
        if(branchPos)    *branchPos += 1.0;
        if(branchNeg)    *branchNeg -= 1.0;
    }

    void stampTransient(const TransientStampContext& ctx) override {
        const double preI = ctx.previousSolutionVal(branch);
        const double r = inductance_ / ctx.timeStep;
    
        stampOperatingPoint();
        *bb -= r;
        *rhs -= r * preI;
    }

private:
    double inductance_;

    double* posBranch = nullptr;
    double* negBranch = nullptr;
    double* branchPos = nullptr;
    double* branchNeg = nullptr;

    double* bb = nullptr;
    double* rhs = nullptr;

    int branch = -1;
};
