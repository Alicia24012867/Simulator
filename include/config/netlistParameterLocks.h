#pragma once

namespace simulator::config {

// Fields explicitly set by a netlist control card.  Config-file and command-
// line layers must leave these values unchanged, because a deck is the
// authoritative description of the analysis it requests.
struct PtaParameterLocks {
    bool mode = false;
    bool initialStep = false;
    bool minimumStep = false;
    bool maximumStep = false;
    bool derivativeTolerance = false;
    bool dcResidualTolerance = false;
    bool compoundTimeConstant = false;
    bool sourceRampTime = false;
    bool initialBjtVbe = false;
};

struct TransientParameterLocks {
    bool enabled = false;
    bool outputInterval = false;
    bool stopTime = false;
    bool outputStartTime = false;
    bool maximumStep = false;
    bool useInitialConditions = false;
};

struct NetlistAnalysisParameterLocks {
    PtaParameterLocks pta;
    TransientParameterLocks transient;
};

}  // namespace simulator::config
