#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "device.hpp"
#include "../math/mna.hpp"
#include "../math/limiting.hpp"
#include "../models/model.hpp"

class MOSFET: public Device{
public:
    MOSFET(std::string name, std::vector<std::string> nodes, const Model* model, double w = 1.0, double l = 1.0):
            Device(name, nodes, DeviceType::MOSFET), model_(model), w_(w), l_(l) {}

    const Model* model() const { return model_; }

    bool isNonlinear() const override{
        return true;
    }

    void pattern(MNA& mna) override{
        addPattern(mna, 0, 0);
        addPattern(mna, 0, 1);
        addPattern(mna, 0, 2);
        addPattern(mna, 2, 0);
        addPattern(mna, 2, 1);
        addPattern(mna, 2, 2);
    }

    void bindMatrix(MNA& mna) override{
        for(int r: {0, 2}){
            const int row = nodeIds[r];
            if(row >= 0){
                rhs_[r] = &mna.rhs(row);
                sol_[r] = mna.solutionPtr(row);
            }

            for(int c: {0, 1, 2}){
                const int col = nodeIds[c];
                if(row >= 0 && col >= 0){
                    A_[r][c] = mna.ptr(row, col);
                }
            }
        }

        for(int c: {1, 3}){
            const int node = nodeIds[c];
            if(node >= 0){
                sol_[c] = mna.solutionPtr(node);
            }
        }
    }

    void stamp() override{
        if(!model_) return;

        const auto& dc = model_->mosDc();
        const double polarity = model_->type() == ModelType::PMOS ? -1.0 : 1.0;
        const double vd = voltage(sol_[0]);
        const double vg = voltage(sol_[1]);
        const double vs = voltage(sol_[2]);
        const double width = w_ > 0.0 ? w_ : 1.0;
        const double length = l_ > 0.0 ? l_ : 1.0;
        const double beta = dc.kp * width / length;
        const double vto = std::abs(dc.vto);
        double vgs = polarity * (vg - vs);
        double vds = polarity * (vd - vs);
        double vgd = vgs - vds;

        if(hasPreviousVoltages_){
            vgs = limitMosfetVoltage(vgs, previousVgs_, vto);
            vgd = limitMosfetVoltage(vgd, previousVgd_, vto);
            vds = vgs - vgd;
        }

        previousVgs_ = vgs;
        previousVgd_ = vgd;
        hasPreviousVoltages_ = true;

        const double overdrive = vgs - vto;

        double id = 0.0;
        double gm = 0.0;
        double gds = 0.0;

        if(overdrive > 0.0){
            const double vdsEff = std::max(vds, 0.0);
            if(vdsEff < overdrive){
                const double channel = overdrive * vdsEff - 0.5 * vdsEff * vdsEff;
                const double lambdaTerm = 1.0 + dc.lambda * vdsEff;
                id = beta * channel * lambdaTerm;
                gm = beta * vdsEff * lambdaTerm;
                gds = beta * (overdrive - vdsEff) * lambdaTerm
                    + beta * channel * dc.lambda;
            } else {
                const double channel = 0.5 * overdrive * overdrive;
                const double lambdaTerm = 1.0 + dc.lambda * vdsEff;
                id = beta * channel * lambdaTerm;
                gm = beta * overdrive * lambdaTerm;
                gds = beta * channel * dc.lambda;
            }
        }

        const double ids = polarity * id + dc.gmin * (vd - vs);
        const double gdTotal = gds + dc.gmin;
        const double v[4] = {vd, vg, vs, voltage(sol_[3])};
        double f[4] = {ids, 0.0, -ids, 0.0};
        double j[4][4] = {};

        j[0][0] += gdTotal;
        j[0][1] += gm;
        j[0][2] -= gdTotal + gm;
        j[2][0] -= gdTotal;
        j[2][1] -= gm;
        j[2][2] += gdTotal + gm;

        for(int r: {0, 2}){
            double b = -f[r];
            for(int c: {0, 1, 2}){
                if(A_[r][c]) *A_[r][c] += j[r][c];
                b += j[r][c] * v[c];
            }
            if(rhs_[r]) *rhs_[r] += b;
        }
    }

    void saveState() override{
        savedPreviousVgs_ = previousVgs_;
        savedPreviousVgd_ = previousVgd_;
        savedHasPreviousVoltages_ = hasPreviousVoltages_;
    }

    void restoreState() override{
        previousVgs_ = savedPreviousVgs_;
        previousVgd_ = savedPreviousVgd_;
        hasPreviousVoltages_ = savedHasPreviousVoltages_;
    }

private:
    static double voltage(const double* ptr){
        return ptr ? *ptr : 0.0;
    }

    void addPattern(MNA& mna, int r, int c){
        const int row = nodeIds[r];
        const int col = nodeIds[c];
        if(row >= 0 && col >= 0){
            mna.addPattern(row, col);
        }
    }

    const Model* model_;
    double w_;
    double l_;

    std::array<std::array<double*, 4>, 4> A_ = {};
    std::array<double*, 4> rhs_ = {};
    std::array<const double*, 4> sol_ = {};

    double previousVgs_ = 0.0;
    double previousVgd_ = 0.0;
    bool hasPreviousVoltages_ = false;
    double savedPreviousVgs_ = 0.0;
    double savedPreviousVgd_ = 0.0;
    bool savedHasPreviousVoltages_ = false;
};
