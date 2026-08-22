#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

#include "circuit/circuit.h"
#include "analysis/transient_analysis.h"
#include "devices/device.hpp"
#include "devices/mos3_dc.hpp"
#include "devices/mosfet_params.hpp"
#include "math/limiting.hpp"
#include "math/mna.hpp"
#include "models/model.hpp"

class MOSFET: public Device{
public:
    using MosInstanceParams = ::MosInstanceParams;

    MOSFET(std::string name,
           std::vector<std::string> nodes,
           const Model* model):
            MOSFET(
                std::move(name),
                std::move(nodes),
                model,
                MosInstanceParams()
            ) {}

    MOSFET(std::string name,
           std::vector<std::string> nodes,
           const Model* model,
           MosInstanceParams instance):
            Device(name, nodes, DeviceType::MOSFET),
            model_(model),
            instance_(std::move(instance)) {}

    MOSFET(std::string name,
           std::vector<std::string> nodes,
           const Model* model,
           double w,
           double l):
            Device(name, nodes, DeviceType::MOSFET),
            model_(model) {
        instance_.w = w;
        instance_.l = l;
    }

    const Model* model() const { return model_; }

    const MosInstanceParams& instanceParams() const { return instance_; }

    bool isNonlinear() const override{
        return true;
    }

    bool requiresUicChargeHistoryProtection() const override{
        return model_ && model_->isMos3();
    }

    void allocateUnknown(Circuit& circuit) override{
        if(!model_ || !model_->isMos3()) return;
        if(drainSeriesResistance() > 0.0){
            internalNodes_[0] = circuit.allocateUnknown();
        }
        if(sourceSeriesResistance() > 0.0){
            internalNodes_[1] = circuit.allocateUnknown();
        }
    }

    void pattern(MNA& mna) override{
        if(model_ && model_->isMos3()){
            const auto core = coreNodes();
            addFullPattern(mna, core);
            addSeriesPattern(mna, nodeIds[0], core[0], drainSeriesResistance());
            addSeriesPattern(mna, nodeIds[2], core[2], sourceSeriesResistance());
            return;
        }
        for(int r: {0, 2}){
            for(int c = 0; c < 4; ++c){
                addPattern(mna, r, c);
            }
        }
    }

    void bindMatrix(MNA& mna) override{
        if(model_ && model_->isMos3()){
            const auto core = coreNodes();
            for(int r = 0; r < 4; ++r){
                if(core[r] >= 0){
                    rhs_[r] = &mna.rhs(core[r]);
                    sol_[r] = mna.solutionPtr(core[r]);
                }
                for(int c = 0; c < 4; ++c){
                    if(core[r] >= 0 && core[c] >= 0){
                        A_[r][c] = mna.ptr(core[r], core[c]);
                    }
                }
            }
            bindSeries(mna, 0, nodeIds[0], core[0], drainSeriesResistance());
            bindSeries(mna, 1, nodeIds[2], core[2], sourceSeriesResistance());
            return;
        }
        for(int r: {0, 2}){
            const int row = nodeIds[r];
            if(row >= 0){
                rhs_[r] = &mna.rhs(row);
                sol_[r] = mna.solutionPtr(row);
            }

            for(int c = 0; c < 4; ++c){
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

    void stampOperatingPoint() override{
        if(!model_) return;
        if(model_->isMos3()){
            stampMos3OperatingPoint();
            return;
        }

        const auto& dc = model_->mosDc();
        const double polarity = model_->type() == ModelType::PMOS ? -1.0 : 1.0;
        const double vd = voltage(sol_[0]);
        const double vg = voltage(sol_[1]);
        const double vs = voltage(sol_[2]);
        const double width = instance_.w > 0.0 ? instance_.w : 1.0;
        const double length = instance_.l > 0.0 ? instance_.l : 1.0;
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

        if(dc.rds > 0.0){
            const double conductance =
                model_->mosDrainSourceConductance(instance_.w, instance_.l);
            const double rdsCurrent = conductance * (vd - vs);
            f[0] += rdsCurrent;
            f[2] -= rdsCurrent;
            j[0][0] += conductance;
            j[0][2] -= conductance;
            j[2][0] -= conductance;
            j[2][2] += conductance;
        }

        for(int r: {0, 2}){
            double b = -f[r];
            for(int c: {0, 1, 2}){
                if(A_[r][c]) *A_[r][c] += j[r][c];
                b += j[r][c] * v[c];
            }
            if(rhs_[r]) *rhs_[r] += b;
        }
    }

    void stampTransient(const TransientStampContext& ctx) override{
        if(!model_ || !model_->isMos3()){
            stampOperatingPoint();
            return;
        }

        stampMos3OperatingPoint();
        stampMos3JunctionCharges(ctx);
        stampMos3OverlapCharges(ctx);
        stampMos3MeyerCharges(ctx);
    }

    void initializeTransientHistory(const Eigen::VectorXd& solution) override{
        if(!model_ || !model_->isMos3()) return;
        const auto core = coreNodes();
        const Mos3MeyerCapacitances capacitances =
            evaluateMeyerCapacitances(solution);
        for(int pair = 0; pair < 3; ++pair){
            const int negative = kMeyerNegativeTerminals[pair];
            meyerChargeN_[pair] = capacitanceAt(capacitances, pair) * (
                voltageAt(solution, core, 1) - voltageAt(solution, core, negative)
            );
        }
        meyerChargeNm1_ = meyerChargeN_;
        for(int junction = 0; junction < 2; ++junction){
            const int terminal = kJunctionTerminals[junction];
            junctionChargeN_[junction] =
                evaluateMos3JunctionCapacitance(solution, terminal) * (
                    voltageAt(solution, core, 3) - voltageAt(solution, core, terminal)
                );
        }
        junctionChargeNm1_ = junctionChargeN_;
    }

    void acceptTransientSolution(const Eigen::VectorXd& previous,
                                 const Eigen::VectorXd& accepted) override{
        if(!model_ || !model_->isMos3()) return;
        const auto core = coreNodes();
        const Mos3MeyerCapacitances capacitances =
            evaluateMeyerCapacitances(previous);
        std::array<double, 3> nextCharge = meyerChargeN_;
        for(int pair = 0; pair < 3; ++pair){
            const int negative = kMeyerNegativeTerminals[pair];
            const double voltageChange =
                voltageAt(accepted, core, 1) - voltageAt(accepted, core, negative) -
                voltageAt(previous, core, 1) + voltageAt(previous, core, negative);
            nextCharge[pair] += capacitanceAt(capacitances, pair) * voltageChange;
        }
        meyerChargeNm1_ = meyerChargeN_;
        meyerChargeN_ = nextCharge;

        std::array<double, 2> nextJunctionCharge = junctionChargeN_;
        for(int junction = 0; junction < 2; ++junction){
            const int terminal = kJunctionTerminals[junction];
            const double voltageChange =
                voltageAt(accepted, core, 3) - voltageAt(accepted, core, terminal) -
                voltageAt(previous, core, 3) + voltageAt(previous, core, terminal);
            nextJunctionCharge[junction] +=
                evaluateMos3JunctionCapacitance(previous, terminal) * voltageChange;
        }
        junctionChargeNm1_ = junctionChargeN_;
        junctionChargeN_ = nextJunctionCharge;
    }

    void saveIterationState() override{
        savedPreviousVgs_ = previousVgs_;
        savedPreviousVgd_ = previousVgd_;
        savedHasPreviousVoltages_ = hasPreviousVoltages_;
        savedMeyerChargeN_ = meyerChargeN_;
        savedMeyerChargeNm1_ = meyerChargeNm1_;
        savedJunctionChargeN_ = junctionChargeN_;
        savedJunctionChargeNm1_ = junctionChargeNm1_;
    }

    void restoreIterationState() override{
        previousVgs_ = savedPreviousVgs_;
        previousVgd_ = savedPreviousVgd_;
        hasPreviousVoltages_ = savedHasPreviousVoltages_;
        meyerChargeN_ = savedMeyerChargeN_;
        meyerChargeNm1_ = savedMeyerChargeNm1_;
        junctionChargeN_ = savedJunctionChargeN_;
        junctionChargeNm1_ = savedJunctionChargeNm1_;
    }

private:
    using Vec4 = std::array<double, 4>;
    using Mat4 = std::array<std::array<double, 4>, 4>;

    static double voltage(const double* ptr){
        return ptr ? *ptr : 0.0;
    }

    std::array<int, 4> coreNodes() const{
        return {
            internalNodes_[0] >= 0 ? internalNodes_[0] : nodeIds[0],
            nodeIds[1],
            internalNodes_[1] >= 0 ? internalNodes_[1] : nodeIds[2],
            nodeIds[3]
        };
    }

    static double valueOrZero(const std::optional<double>& value){
        return value ? *value : 0.0;
    }

    double seriesResistance(bool drain) const{
        if(!model_ || !model_->isMos3()) return 0.0;
        const auto& card = model_->mos3();
        const double explicitResistance = valueOrZero(drain ? card.rd : card.rs);
        if(explicitResistance > 0.0){
            return explicitResistance / std::max(instance_.m, 1e-30);
        }
        const double squares = drain ? instance_.nrd : instance_.nrs;
        const double sheetResistance = valueOrZero(card.rsh);
        if(sheetResistance > 0.0 && squares > 0.0){
            return sheetResistance * squares / std::max(instance_.m, 1e-30);
        }
        return 0.0;
    }

    double drainSeriesResistance() const { return seriesResistance(true); }
    double sourceSeriesResistance() const { return seriesResistance(false); }

    void addPattern(MNA& mna, int r, int c){
        const int row = nodeIds[r];
        const int col = nodeIds[c];
        if(row >= 0 && col >= 0){
            mna.addPattern(row, col);
        }
    }

    static void addFullPattern(MNA& mna, const std::array<int, 4>& nodes){
        for(int row: nodes){
            if(row < 0) continue;
            for(int col: nodes){
                if(col >= 0) mna.addPattern(row, col);
            }
        }
    }

    static void addSeriesPattern(MNA& mna, int external, int internal,
                                 double resistance){
        if(resistance <= 0.0) return;
        if(external >= 0) mna.addPattern(external, external);
        if(internal >= 0) mna.addPattern(internal, internal);
        if(external >= 0 && internal >= 0){
            mna.addPattern(external, internal);
            mna.addPattern(internal, external);
        }
    }

    void bindSeries(MNA& mna, int terminal, int external, int internal,
                    double resistance){
        if(resistance <= 0.0) return;
        if(external >= 0) seriesA_[terminal][0][0] = mna.ptr(external, external);
        if(internal >= 0) seriesA_[terminal][1][1] = mna.ptr(internal, internal);
        if(external >= 0 && internal >= 0){
            seriesA_[terminal][0][1] = mna.ptr(external, internal);
            seriesA_[terminal][1][0] = mna.ptr(internal, external);
        }
    }

    void stampSeries(int terminal, double resistance){
        if(resistance <= 0.0) return;
        const double g = 1.0 / resistance;
        if(seriesA_[terminal][0][0]) *seriesA_[terminal][0][0] += g;
        if(seriesA_[terminal][0][1]) *seriesA_[terminal][0][1] -= g;
        if(seriesA_[terminal][1][0]) *seriesA_[terminal][1][0] -= g;
        if(seriesA_[terminal][1][1]) *seriesA_[terminal][1][1] += g;
    }

    void stampSeriesResistors(){
        stampSeries(0, drainSeriesResistance());
        stampSeries(1, sourceSeriesResistance());
    }

    static void stampBranch(Vec4& f, Mat4& j, int p, int n, double i,
                            double g){
        f[p] += i;
        f[n] -= i;
        j[p][p] += g;
        j[p][n] -= g;
        j[n][p] -= g;
        j[n][n] += g;
    }

    void stampLinearization(const Vec4& f, const Mat4& j){
        const Vec4 v = {
            voltage(sol_[0]), voltage(sol_[1]), voltage(sol_[2]), voltage(sol_[3])
        };
        for(int r = 0; r < 4; ++r){
            double b = -f[r];
            for(int c = 0; c < 4; ++c){
                if(A_[r][c]) *A_[r][c] += j[r][c];
                b += j[r][c] * v[c];
            }
            if(rhs_[r]) *rhs_[r] += b;
        }
    }

    void stampMos3OperatingPoint(){
        const auto& card = model_->mos3();
        const Vec4 v = {
            voltage(sol_[0]), voltage(sol_[1]), voltage(sol_[2]), voltage(sol_[3])
        };
        const Mos3DcResult result = instance_.off
            ? Mos3DcResult{}
            : evaluateMos3Dc(v[0], v[1], v[2], v[3], *model_, instance_);

        Vec4 f = {result.ids, 0.0, -result.ids, 0.0};
        Mat4 j = {};
        j[0][0] = result.gds;
        j[0][1] = result.gm;
        j[0][3] = result.gmb;
        j[0][2] = -(result.gds + result.gm + result.gmb);
        for(int c = 0; c < 4; ++c){
            j[2][c] = -j[0][c];
        }

        const double polarity = model_->type() == ModelType::PMOS ? -1.0 : 1.0;
        const double thermalVoltage = 0.02585;
        const auto stampJunction = [&](int terminal, double area){
            const double isDensity = valueOrZero(card.js);
            const double is = isDensity > 0.0 && area > 0.0
                ? isDensity * area * std::max(instance_.m, 1e-30)
                : valueOrZero(card.is) * std::max(instance_.m, 1e-30);
            if(is <= 0.0) return;
            const double junctionVoltage = polarity * (v[3] - v[terminal]);
            const double exponential = std::exp(
                std::clamp(junctionVoltage / thermalVoltage, -40.0, 40.0)
            );
            const double current = polarity * is * (exponential - 1.0);
            const double conductance = is * exponential / thermalVoltage;
            stampBranch(f, j, 3, terminal, current, conductance);
        };
        stampJunction(0, instance_.ad);
        stampJunction(2, instance_.as);

        stampLinearization(f, j);
        stampSeriesResistors();
    }

    static double evaluateJunctionCapacitance(double voltage,
                                              double bottomCapacitance,
                                              double sidewallCapacitance,
                                              double potential,
                                              double bottomGrading,
                                              double sidewallGrading,
                                              double forwardBiasCoeff){
        if((bottomCapacitance <= 0.0 && sidewallCapacitance <= 0.0) ||
           potential <= 0.0){
            return 0.0;
        }

        const double fc = std::clamp(forwardBiasCoeff, 0.0, 0.99);
        const double depletionVoltage = fc * potential;
        const auto depletion = [&](double capacitance, double grading){
            if(capacitance <= 0.0) return 0.0;
            const double arg = std::max(1.0 - voltage / potential, 1.0e-12);
            return capacitance * std::pow(arg, -grading);
        };

        if(voltage < depletionVoltage){
            return depletion(bottomCapacitance, bottomGrading) +
                depletion(sidewallCapacitance, sidewallGrading);
        }

        const double arg = std::max(1.0 - fc, 1.0e-12);
        const auto forward = [&](double capacitance, double grading){
            if(capacitance <= 0.0) return 0.0;
            const double scale = std::pow(arg, -grading);
            const double f2 = capacitance * (1.0 - fc * (1.0 + grading)) *
                scale / arg;
            const double f3 = capacitance * grading * scale / arg / potential;
            return f2 + voltage * f3;
        };
        return forward(bottomCapacitance, bottomGrading) +
            forward(sidewallCapacitance, sidewallGrading);
    }

    void stampMos3JunctionCharges(const TransientStampContext& ctx){
        const auto core = coreNodes();
        for(int junction = 0; junction < 2; ++junction){
            const int terminal = kJunctionTerminals[junction];
            stampChargeCompanion(
                ctx, core, 3, terminal,
                evaluateMos3JunctionCapacitance(ctx.previousSolution, terminal),
                junctionChargeN_[junction], junctionChargeNm1_[junction]
            );
        }
    }

    void stampMos3OverlapCharges(const TransientStampContext& ctx){
        const auto& card = model_->mos3();
        const double multiplicity = std::max(instance_.m, 1e-30);
        const double effectiveLength = instance_.l -
            2.0 * valueOrZero(card.ld) + valueOrZero(card.xl);
        const double effectiveWidth = instance_.w -
            2.0 * valueOrZero(card.wd) + valueOrZero(card.xw);
        if(effectiveLength <= 0.0 || effectiveWidth <= 0.0) return;

        const auto core = coreNodes();
        stampTransientCapacitor(
            ctx, core, 1, 2, valueOrZero(card.cgso) * multiplicity * effectiveWidth
        );
        stampTransientCapacitor(
            ctx, core, 1, 0, valueOrZero(card.cgdo) * multiplicity * effectiveWidth
        );
        stampTransientCapacitor(
            ctx, core, 1, 3, valueOrZero(card.cgbo) * multiplicity * effectiveLength
        );
    }

    void stampMos3MeyerCharges(const TransientStampContext& ctx){
        const auto core = coreNodes();
        const Mos3MeyerCapacitances capacitances =
            evaluateMeyerCapacitances(ctx.previousSolution);
        for(int pair = 0; pair < 3; ++pair){
            stampChargeCompanion(
                ctx, core, 1, kMeyerNegativeTerminals[pair],
                capacitanceAt(capacitances, pair), meyerChargeN_[pair],
                meyerChargeNm1_[pair]
            );
        }
    }

    static constexpr std::array<int, 3> kMeyerNegativeTerminals = {2, 0, 3};
    static constexpr std::array<int, 2> kJunctionTerminals = {0, 2};

    double evaluateMos3JunctionCapacitance(const Eigen::VectorXd& solution,
                                           int terminal) const{
        const auto& card = model_->mos3();
        const double multiplicity = std::max(instance_.m, 1e-30);
        const double potential = valueOrZero(card.pb) > 0.0
            ? valueOrZero(card.pb) : 0.8;
        const double bottomGrading = card.mj ? *card.mj : 0.5;
        const double sidewallGrading = card.mjsw ? *card.mjsw : 0.33;
        const double forwardBiasCoeff = card.fc ? *card.fc : 0.5;
        const bool drain = terminal == 0;
        const double area = drain ? instance_.ad : instance_.as;
        const double perimeter = drain ? instance_.pd : instance_.ps;
        const std::optional<double>& explicitCapacitance =
            drain ? card.cbd : card.cbs;
        const double bottomCapacitance = explicitCapacitance
            ? *explicitCapacitance * multiplicity
            : valueOrZero(card.cj) * std::max(area, 0.0) * multiplicity;
        const double sidewallCapacitance = valueOrZero(card.cjsw) *
            std::max(perimeter, 0.0) * multiplicity;
        const auto core = coreNodes();
        const double polarity = model_->type() == ModelType::PMOS ? -1.0 : 1.0;
        const double junctionVoltage = polarity * (
            voltageAt(solution, core, 3) - voltageAt(solution, core, terminal)
        );
        return evaluateJunctionCapacitance(
            junctionVoltage, bottomCapacitance, sidewallCapacitance, potential,
            bottomGrading, sidewallGrading, forwardBiasCoeff
        );
    }

    static double voltageAt(const Eigen::VectorXd& solution,
                            const std::array<int, 4>& core,
                            int terminal){
        return core[terminal] >= 0 ? solution[core[terminal]] : 0.0;
    }

    Mos3MeyerCapacitances evaluateMeyerCapacitances(
        const Eigen::VectorXd& solution
    ) const{
        const auto core = coreNodes();
        return evaluateMos3MeyerCapacitances(
            voltageAt(solution, core, 0), voltageAt(solution, core, 1),
            voltageAt(solution, core, 2), voltageAt(solution, core, 3),
            *model_, instance_
        );
    }

    static double capacitanceAt(const Mos3MeyerCapacitances& capacitances,
                                int pair){
        switch(pair){
            case 0: return capacitances.cgs;
            case 1: return capacitances.cgd;
            default: return capacitances.cgb;
        }
    }

    void stampChargeCompanion(const TransientStampContext& ctx,
                              const std::array<int, 4>& core,
                              int positive,
                              int negative,
                              double capacitance,
                              double previousCharge,
                              double olderCharge){
        if(capacitance <= 0.0) return;
        const double conductance = ctx.derivative.alpha0 * capacitance;
        const double previousVoltage =
            voltageAt(ctx.previousSolution, core, positive) -
            voltageAt(ctx.previousSolution, core, negative);
        const double history =
            (ctx.derivative.alpha0 + ctx.derivative.alpha1) * previousCharge +
            ctx.derivative.alpha2 * olderCharge -
            ctx.derivative.alpha0 * capacitance * previousVoltage;

        if(A_[positive][positive]) *A_[positive][positive] += conductance;
        if(A_[positive][negative]) *A_[positive][negative] -= conductance;
        if(A_[negative][positive]) *A_[negative][positive] -= conductance;
        if(A_[negative][negative]) *A_[negative][negative] += conductance;
        if(rhs_[positive]) *rhs_[positive] -= history;
        if(rhs_[negative]) *rhs_[negative] += history;
    }

    void stampTransientCapacitor(const TransientStampContext& ctx,
                                 const std::array<int, 4>& core,
                                 int positive,
                                 int negative,
                                 double capacitance){
        if(capacitance <= 0.0) return;
        const double conductance = ctx.derivative.alpha0 * capacitance;
        const double history = capacitance *
            ctx.historyDerivativeDifference(core[positive], core[negative]);

        if(A_[positive][positive]) *A_[positive][positive] += conductance;
        if(A_[positive][negative]) *A_[positive][negative] -= conductance;
        if(A_[negative][positive]) *A_[negative][positive] -= conductance;
        if(A_[negative][negative]) *A_[negative][negative] += conductance;
        if(rhs_[positive]) *rhs_[positive] -= history;
        if(rhs_[negative]) *rhs_[negative] += history;
    }

    const Model* model_;
    MosInstanceParams instance_;

    std::array<int, 2> internalNodes_ = {-1, -1};
    std::array<std::array<double*, 4>, 4> A_ = {};
    std::array<double*, 4> rhs_ = {};
    std::array<const double*, 4> sol_ = {};
    std::array<std::array<std::array<double*, 2>, 2>, 2> seriesA_ = {};

    double previousVgs_ = 0.0;
    double previousVgd_ = 0.0;
    bool hasPreviousVoltages_ = false;
    double savedPreviousVgs_ = 0.0;
    double savedPreviousVgd_ = 0.0;
    bool savedHasPreviousVoltages_ = false;
    std::array<double, 3> meyerChargeN_ = {};
    std::array<double, 3> meyerChargeNm1_ = {};
    std::array<double, 3> savedMeyerChargeN_ = {};
    std::array<double, 3> savedMeyerChargeNm1_ = {};
    std::array<double, 2> junctionChargeN_ = {};
    std::array<double, 2> junctionChargeNm1_ = {};
    std::array<double, 2> savedJunctionChargeN_ = {};
    std::array<double, 2> savedJunctionChargeNm1_ = {};
};
