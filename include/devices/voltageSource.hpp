#pragma once

#include <cassert>

#include "device.hpp"
#include "../core/circuit.h"
#include "../math/mna.hpp"

class VoltageSource: public Device{
public:
    VoltageSource(std::string name, std::vector<std::string> nodes, double V):
                Device(name, nodes, DeviceType::VoltageSource), v(V){}

    void allocateUnknown(Circuit& circuit) override{
        branch = circuit.allocateUnknown();
    }

    int branchUnknown() const override{
        return branch;
    }

    void pattern(MNA& mna) override{
        assert(branch >= 0);

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

        rhs = &mna.rhs(branch);
    }

    void stamp() override {
        if(posBranch)    *posBranch += 1.0;
        if(negBranch)    *negBranch -= 1.0;
        if(branchPos)    *branchPos += 1.0;
        if(branchNeg)    *branchNeg -= 1.0;

        *rhs += sourceScale_ * v;
    }

    void setSourceScale(double scale) override{
        sourceScale_ = scale;
    }

private:
    double v;
    double sourceScale_ = 1.0;

    double* posBranch = nullptr;
    double* negBranch = nullptr;
    double* branchPos = nullptr;
    double* branchNeg = nullptr;

    double* rhs = nullptr;

    int branch = -1;
};
