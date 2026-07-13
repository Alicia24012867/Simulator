#pragma once

#include <cassert>

#include "device.hpp"
#include "../core/transientContext.h"

class Capacitor: public Device{
public:
    Capacitor(std::string name, std::vector<std::string> nodes, double C):
            Device(name, nodes, DeviceType::Capacitor), capacitance_(C) {
                assert(C > 0.0);
            }

    void pattern(MNA& mna) override{
        const int p = nodeIds[0];
        const int n = nodeIds[1];

        if(p >= 0){
            mna.addPattern(p, p);       
        }
        if(n >= 0){
            mna.addPattern(n, n);
        }
        if(p >= 0 && n >= 0){
            mna.addPattern(p, n);
            mna.addPattern(n, p);
        }
    }

    void bindMatrix(MNA& mna) override{
        const int p = nodeIds[0];
        const int n = nodeIds[1];

        if(p >= 0){
            aPp_ = mna.ptr(p, p);
            rhsP_ = &mna.rhs(p);
        }
        if(n >= 0){
            aNn_ = mna.ptr(n, n);
            rhsN_ = &mna.rhs(n);
        }
        if(p >=0 && n >= 0){
            aPn_ = mna.ptr(p, n);
            aNp_ = mna.ptr(n, p);
        }
    }

    void stampOperatingPoint() override{}

    void stampTransient(const TransientStampContext& ctx) override{
        const int p = nodeIds[0];
        const int n = nodeIds[1];

        const double g = capacitance_ / ctx.timeStep;
        
        const double preVp = ctx.previousSolutionVal(p);
        const double preVn = ctx.previousSolutionVal(n);
        const double i = g * (preVp - preVn);

        if(aPp_) *aPp_ += g;
        if(aNn_) *aNn_ += g;
        if(aPn_) *aPn_ -= g;
        if(aNp_) *aNp_ -= g;

        if(rhsP_) *rhsP_ += i;
        if(rhsN_) *rhsN_ -= i;
    }

private:
    double capacitance_;

    double* aPp_ = nullptr;
    double* aPn_ = nullptr;
    double* aNp_ = nullptr;
    double* aNn_ = nullptr;

    double* rhsP_ = nullptr;
    double* rhsN_ = nullptr;
};
