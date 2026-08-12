#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "circuit/circuit.h"
#include "devices/device.hpp"
#include "math/limiting.hpp"
#include "math/mna.hpp"
#include "models/model.hpp"

class BJT: public Device{
public:
    BJT(std::string name, std::vector<std::string> nodes, const Model* model, double area = 1.0):
            Device(name, nodes, DeviceType::BJT), model_(model), area_(area) {}

    const Model* model() const { return model_; }

    bool isNonlinear() const override{
        return true;
    }

    void pattern(MNA& mna) override{
        addFullPattern(mna);
        if(internalBaseNode_ >= 0){
            addResistorPattern(mna, nodeIds[1], internalBaseNode_);
        }
    }

    void allocateUnknown(Circuit& circuit) override{
        if(model_ && model_->bjtDc().rb > 0.0){
            internalBaseNode_ = circuit.allocateUnknown();
        }
    }

    void bindMatrix(MNA& mna) override{
        const std::array<int, 3> coreNodes = {
            nodeIds[0], effectiveBaseNode(), nodeIds[2]
        };
        for(int r = 0; r < 3; ++r){
            const int row = coreNodes[r];
            if(row >= 0){
                rhs_[r] = &mna.rhs(row);
                sol_[r] = mna.solutionPtr(row);
            }

            for(int c = 0; c < 3; ++c){
                const int col = coreNodes[c];
                if(row >= 0 && col >= 0){
                    A_[r][c] = mna.ptr(row, col);
                }
            }
        }

        if(internalBaseNode_ >= 0){
            bindBaseResistance(mna, nodeIds[1], internalBaseNode_);
        }
    }

    void stampOperatingPoint() override{
        if(!model_) return;

        const auto& dc = model_->bjtDc();
        const double area = area_ > 0.0 ? area_ : 1.0;
        const double polarity = model_->type() == ModelType::PNP ? -1.0 : 1.0;
        const double vc = voltage(sol_[0]);
        const double vb = voltage(sol_[1]);
        const double ve = voltage(sol_[2]);

        double vbe = polarity * (vb - ve);
        double vbc = polarity * (vb - vc);
        
        const double nvtBe = dc.nf * dc.vt;
        const double nvtBc = dc.nr * dc.vt;

        const double is = dc.is * area;

        if(hasPreviousVoltages_){
            vbe = limitPnJunctionColon(vbe, previousVbe_, nvtBe, is);
            vbc = limitPnJunctionColon(vbc, previousVbc_, nvtBc, is);
        }

        previousVbe_ = vbe;
        previousVbc_ = vbc;
        hasPreviousVoltages_ = true;

        const double argBe = std::clamp(vbe / nvtBe, -40.0, 40.0);
        const double argBc = std::clamp(vbc / nvtBc, -40.0, 40.0);
        const double ebe = std::exp(argBe);
        const double ebc = std::exp(argBc);
        const double ibe = polarity * (is / dc.bf) * (ebe - 1.0);
        const double ibc = polarity * (is / dc.br) * (ebc - 1.0);
        const double collectorDelta = (ebe - 1.0) - (ebc - 1.0);
        const double vce = polarity * (vc - ve);
        const double earlyFactor = dc.va > 0.0 ? 1.0 + vce / dc.va : 1.0;
        const double icc = polarity * is * collectorDelta * earlyFactor;
        const double gbe = is * ebe / (dc.bf * nvtBe) + dc.gmin;
        const double gbc = is * ebc / (dc.br * nvtBc) + dc.gmin;
        const double gmF = is * ebe / nvtBe;
        const double gmR = is * ebc / nvtBc;
        const double dIccDb = (gmF - gmR) * earlyFactor;
        double dIccDc = gmR * earlyFactor;
        double dIccDe = -gmF * earlyFactor;
        if(dc.va > 0.0){
            const double earlyConductance = is * collectorDelta / dc.va;
            dIccDc += earlyConductance;
            dIccDe -= earlyConductance;
        }

        std::array<double, 3> f = {0.0, 0.0, 0.0};
        std::array<std::array<double, 3>, 3> j = {};

        stampBranch(f, j, 1, 2, ibe + dc.gmin * (vb - ve), gbe);
        stampBranch(f, j, 1, 0, ibc + dc.gmin * (vb - vc), gbc);

        f[0] += icc;
        f[2] -= icc;
        j[0][1] += dIccDb;
        j[0][2] += dIccDe;
        j[0][0] += dIccDc;
        j[2][1] -= dIccDb;
        j[2][2] -= dIccDe;
        j[2][0] -= dIccDc;

        stampLinearization(f, j);
        stampBaseResistance();
    }

    void saveIterationState() override{
        savedPreviousVbe_ = previousVbe_;
        savedPreviousVbc_ = previousVbc_;
        savedHasPreviousVoltages_ = hasPreviousVoltages_;
    }

    void restoreIterationState() override{
        previousVbe_ = savedPreviousVbe_;
        previousVbc_ = savedPreviousVbc_;
        hasPreviousVoltages_ = savedHasPreviousVoltages_;
    }

private:
    using Vec3 = std::array<double, 3>;
    using Mat3 = std::array<std::array<double, 3>, 3>;

    static double voltage(const double* ptr){
        return ptr ? *ptr : 0.0;
    }

    void addFullPattern(MNA& mna){
        const std::array<int, 3> coreNodes = {
            nodeIds[0], effectiveBaseNode(), nodeIds[2]
        };
        for(int r = 0; r < 3; ++r){
            const int row = coreNodes[r];
            if(row < 0) continue;
            for(int c = 0; c < 3; ++c){
                const int col = coreNodes[c];
                if(col >= 0){
                    mna.addPattern(row, col);
                }
            }
        }
    }

    static void stampBranch(Vec3& f, Mat3& j, int p, int n, double i, double g){
        f[p] += i;
        f[n] -= i;
        j[p][p] += g;
        j[p][n] -= g;
        j[n][p] -= g;
        j[n][n] += g;
    }

    int effectiveBaseNode() const{
        return internalBaseNode_ >= 0 ? internalBaseNode_ : nodeIds[1];
    }

    static void addResistorPattern(MNA& mna, int p, int n){
        if(p >= 0) mna.addPattern(p, p);
        if(n >= 0) mna.addPattern(n, n);
        if(p >= 0 && n >= 0){
            mna.addPattern(p, n);
            mna.addPattern(n, p);
        }
    }

    void bindBaseResistance(MNA& mna, int p, int n){
        if(p >= 0){
            baseResistanceA_[0][0] = mna.ptr(p, p);
        }
        if(n >= 0){
            baseResistanceA_[1][1] = mna.ptr(n, n);
        }
        if(p >= 0 && n >= 0){
            baseResistanceA_[0][1] = mna.ptr(p, n);
            baseResistanceA_[1][0] = mna.ptr(n, p);
        }
    }

    void stampBaseResistance(){
        if(internalBaseNode_ < 0) return;

        const double conductance = 1.0 / model_->bjtDc().rb;
        if(baseResistanceA_[0][0]) *baseResistanceA_[0][0] += conductance;
        if(baseResistanceA_[0][1]) *baseResistanceA_[0][1] -= conductance;
        if(baseResistanceA_[1][0]) *baseResistanceA_[1][0] -= conductance;
        if(baseResistanceA_[1][1]) *baseResistanceA_[1][1] += conductance;
    }

    void stampLinearization(const Vec3& f, const Mat3& j){
        const Vec3 v = {voltage(sol_[0]), voltage(sol_[1]), voltage(sol_[2])};
        for(int r = 0; r < 3; ++r){
            double b = -f[r];
            for(int c = 0; c < 3; ++c){
                if(A_[r][c]) *A_[r][c] += j[r][c];
                b += j[r][c] * v[c];
            }
            if(rhs_[r]) *rhs_[r] += b;
        }
    }

    const Model* model_;
    double area_;
    std::array<std::array<double*, 3>, 3> A_ = {};
    std::array<double*, 3> rhs_ = {};
    std::array<const double*, 3> sol_ = {};

    int internalBaseNode_ = -1;
    std::array<std::array<double*, 2>, 2> baseResistanceA_ = {};

    double previousVbe_ = 0.0;
    double previousVbc_ = 0.0;
    bool hasPreviousVoltages_ = false;
    double savedPreviousVbe_ = 0.0;
    double savedPreviousVbc_ = 0.0;
    bool savedHasPreviousVoltages_ = false;
};
