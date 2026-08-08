#pragma once

#include <cassert>

#include "device.hpp"
#include "../math/mna.hpp"
#include "../analysis/transientAnalysis.h"

enum class PtaPlacementKind{
    CurrentSourceParallelCap,
    TransistorNodeCap,
    VoltageSourceSeriesInductor
};

struct PendingPtaPlacement{
    PtaPlacementKind kind;
    const Device* owner;
    int terminal = -1;
};

class PseudoDevice{
public:
    virtual void pattern(MNA& mna) = 0;

    virtual void bindMatrix(MNA& mna) = 0;

    virtual void stampPseudo(const TransientStampContext& ctx) = 0;

    void setValue(double newValue){
        assert(newValue > 0.0);
        value = newValue;
    }

    virtual ~PseudoDevice() = default;

protected:
    double value;
};

class PseudoCapacitor: public PseudoDevice{
public:
    PseudoCapacitor(int pNode, int nNode, double capacitance):
        p(pNode), n(nNode){
            assert(capacitance > 0.0);
            value = capacitance;
        }

    void pattern(MNA& mna) override{
        if(p >= 0){
            mna.addPattern(p, p);
        }

        if(n >= 0){
            mna.addPattern(n, n);
        }

        if(p >= 0 && n >=0){
            mna.addPattern(p, n);
            mna.addPattern(n, p);
        }
    }

    void bindMatrix(MNA& mna) override{
        if(p >= 0){
            pp = mna.ptr(p, p);
            rhsP = &mna.rhs(p);
        }

        if(n >= 0){
            nn = mna.ptr(n, n);
            rhsN = &mna.rhs(n);
        }

        if(p >=0 && n >= 0){
            pn = mna.ptr(p, n);
            np = mna.ptr(n, p);
        }
    }

    void stampPseudo(const TransientStampContext& ctx) override{
        const double g = ctx.derivative.alpha0 * value;
        const double history = value * 
            ctx.historyDerivativeDifference(p, n);

        if(pp)  *pp += g;
        if(nn)  *nn += g;
        if(pn)  *pn -= g;
        if(np)  *np -= g;

        if(rhsP)    *rhsP -= history;
        if(rhsN)    *rhsN += history;
    }

private:
    int p;
    int n;

    double* pp = nullptr;
    double* pn = nullptr;
    double* np = nullptr;
    double* nn = nullptr;

    double* rhsP = nullptr;
    double* rhsN = nullptr;
};

class PseudoInductor: public PseudoDevice{
public:
    PseudoInductor(int branchIdx, double inductance): branch(branchIdx){
        assert(inductance > 0.0);
        value = inductance;
    }

    void pattern(MNA& mna) override{
        mna.addPattern(branch, branch);
    }

    void bindMatrix(MNA& mna) override{
        bb = mna.ptr(branch, branch);
        rhsB = &mna.rhs(branch);
    }

    void stampPseudo(const TransientStampContext& ctx) override{
        const double r = value * ctx.derivative.alpha0;
        
        const double history = value * 
            ctx.historyDerivativeVal(branch);

        *bb -= r;
        *rhsB += history;
    }

private:
    int branch;

    double* bb = nullptr;
    
    double* rhsB = nullptr;
};