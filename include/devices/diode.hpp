#pragma once

#include <algorithm>
#include <cmath>

#include "device.hpp"
#include "../math/mna.hpp"
#include "../models/model.hpp"
#include "../math/limiting.hpp"

class Diode: public Device{
public:
    Diode(std::string name, std::vector<std::string> nodes, const Model* model, double area = 1.0):
            Device(name, nodes, DeviceType::Diode), model_(model), area_(area) {}

    const Model* model() const { return model_; }

    void pattern(MNA& mna) override{
        const int p = nodeIds[0];
        const int n = nodeIds[1];

        if(p >= 0) mna.addPattern(p, p);
        if(n >= 0) mna.addPattern(n, n);
        if(p >= 0 && n >= 0){
            mna.addPattern(p, n);
            mna.addPattern(n, p);
        }
    }

    void bindMatrix(MNA& mna) override{
        const int p = nodeIds[0];
        const int n = nodeIds[1];

        if(p >= 0) A11 = mna.ptr(p, p);
        if(n >= 0) A22 = mna.ptr(n, n);
        if(p >= 0 && n >= 0){
            A12 = mna.ptr(p, n);
            A21 = mna.ptr(n, p);
        }

        if(p >= 0){
            rhsP_ = &mna.rhs(p);
            solP_ = mna.solutionPtr(p);
        }
        if(n >= 0){
            rhsN_ = &mna.rhs(n);
            solN_ = mna.solutionPtr(n);
        }
    }

    void stamp() override{
        if(!model_) return;

        const auto& dc = model_->diodeDc();
        const double area = area_ > 0.0 ? area_ : 1.0;

        double vd = voltage(solP_) - voltage(solN_);
        const double nvt = dc.n * dc.vt;

        const double is = dc.is * area;

        if(hasPreviousVd_){
            vd = limitPnJunctionCombined(vd, previousVd_, nvt, is);
        }
        
        previousVd_ = vd;
        hasPreviousVd_ = true;

        const double arg = std::clamp(vd / nvt, -40.0, 40.0);
        const double evd = std::exp(arg);
        const double id = is * (evd - 1.0) + dc.gmin * vd;
        const double gd = is * evd / nvt + dc.gmin;
        const double bp = gd * vd - id;

        if(A11) *A11 += gd;
        if(A12) *A12 -= gd;
        if(A21) *A21 -= gd;
        if(A22) *A22 += gd;
        if(rhsP_) *rhsP_ += bp;
        if(rhsN_) *rhsN_ -= bp;
    }

private:
    static double voltage(const double* ptr){
        return ptr ? *ptr : 0.0;
    }

    const Model* model_;
    double area_;

    double* A11 = nullptr;
    double* A12 = nullptr;
    double* A21 = nullptr;
    double* A22 = nullptr;
    double* rhsP_ = nullptr;
    double* rhsN_ = nullptr;
    const double* solP_ = nullptr;
    const double* solN_ = nullptr;

    double previousVd_ = 0.0;
    bool hasPreviousVd_ = false;
};
