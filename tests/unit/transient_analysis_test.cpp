#include "analysis/transientAnalysis.h"
#include "analysis/ptaAnalysis.h"
#include "circuit/circuit.h"
#include "devices/device.hpp"
#include "devices/pseudoDevice.hpp"
#include "math/mna.hpp"
#include "math/newtonStep.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

// This deliberately linear stamp is marked nonlinear so the PTA solve takes
// the Newton path.  A small pseudo-capacitance makes the first Newton target
// exceed the iteration budget; one global capacitance increase restores it.
class PtaRecoveryDevice: public Device {
public:
    PtaRecoveryDevice(
        std::string name,
        std::vector<std::string> nodes,
        double conductance,
        double current
    ):
        Device(std::move(name), std::move(nodes), DeviceType::BJT),
        conductance_(conductance),
        current_(current) {}

    bool isNonlinear() const override{
        return true;
    }

    void pattern(MNA& mna) override{
        if(nodeIds[0] >= 0){
            mna.addPattern(nodeIds[0], nodeIds[0]);
        }
    }

    void bindMatrix(MNA& mna) override{
        if(nodeIds[0] >= 0){
            diagonal_ = mna.ptr(nodeIds[0], nodeIds[0]);
            rhs_ = &mna.rhs(nodeIds[0]);
        }
    }

    void stampOperatingPoint() override{
        *diagonal_ += conductance_;
        *rhs_ += current_;
    }

private:
    double conductance_;
    double current_;
    double* diagonal_ = nullptr;
    double* rhs_ = nullptr;
};

class CircuitPtaTestAccess {
public:
    static void addNodeCapacitor(
        Circuit& circuit,
        int node,
        double capacitance,
        double previousDelta,
        bool hasPreviousDelta
    ){
        auto capacitor = std::make_unique<PseudoCapacitor>(
            node,
            -1,
            capacitance
        );
        PseudoCapacitor* rawCapacitor = capacitor.get();

        circuit.pseudoDevices_.push_back(std::move(capacitor));
        circuit.ptaNodeCaps_.push_back({
            node,
            rawCapacitor,
            capacitance,
            previousDelta,
            hasPreviousDelta
        });
    }

    static bool growAll(
        Circuit& circuit,
        const PtaAnalysisConfig& config
    ){
        return circuit.growAllPtaNodeCapacitances(config);
    }

    static void updateAfterAcceptedStep(
        Circuit& circuit,
        const Eigen::VectorXd& currentSolution,
        const Eigen::VectorXd& previousSolution,
        const PtaAnalysisConfig& config
    ){
        circuit.updatePtaNodeCapacitancesAfterAcceptedStep(
            currentSolution,
            previousSolution,
            config
        );
    }

    static double capacitance(const Circuit& circuit, std::size_t index){
        return circuit.ptaNodeCaps_.at(index).capacitance;
    }

    static double previousDelta(const Circuit& circuit, std::size_t index){
        return circuit.ptaNodeCaps_.at(index).previousDelta;
    }

    static bool hasPreviousDelta(const Circuit& circuit, std::size_t index){
        return circuit.ptaNodeCaps_.at(index).hasPreviousDelta;
    }

    static int ptaCapacitanceGrowths(const Circuit& circuit){
        return circuit.operatingPointStats_.ptaCapacitanceGrowths;
    }

    static int ptaCapacitanceReductions(const Circuit& circuit){
        return circuit.operatingPointStats_.ptaCapacitanceReductions;
    }

    static int ptaMinimumStepRecoveries(const Circuit& circuit){
        return circuit.operatingPointStats_.ptaMinimumStepRecoveries;
    }
};

namespace {

int checkCount = 0;
int failureCount = 0;

Eigen::VectorXd makeVector(std::initializer_list<double> values){
    Eigen::VectorXd result(static_cast<Eigen::Index>(values.size()));
    Eigen::Index index = 0;
    for(double value : values){
        result[index++] = value;
    }
    return result;
}

void expect(bool condition, const std::string& description){
    ++checkCount;
    if(condition){
        return;
    }

    ++failureCount;
    std::cerr << "FAIL: " << description << '\n';
}

template<class Callback>
void expectInvalidArgument(Callback&& callback,
                           const std::string& description){
    bool threwInvalidArgument = false;

    try {
        callback();
    } catch(const std::invalid_argument&) {
        threwInvalidArgument = true;
    } catch(...) {
    }

    expect(threwInvalidArgument, description);
}

void expectNear(
    double actual,
    double expected,
    const std::string& description
){
    const double tolerance = 32.0 * std::numeric_limits<double>::epsilon()
        * std::max({1.0, std::abs(actual), std::abs(expected)});
    expect(
        std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
        description
    );
}

void expectDefaultInvalid(
    const TransientErrorEstimate& estimate,
    const std::string& description
){
    expect(!estimate.valid, description + ": invalid");
}

TransientIntegrator makeIntegratorWithHistory(
    const Eigen::VectorXd& initialSolution,
    const Eigen::VectorXd& acceptedSolution,
    double acceptedTime = 1.0
){
    TransientIntegrator integrator;
    integrator.Initialize(0.0, initialSolution);
    integrator.accept(acceptedTime, acceptedSolution);
    return integrator;
}

TransientSolverOptions makeAbsoluteOnlyOptions(){
    TransientSolverOptions options;
    options.relativeTolerance = 0.0;
    options.voltageAbsoluteTolerance = 1.0;
    options.currentAbsoluteTolerance = 1.0;
    return options;
}

void testSolverOptionsValidation(){
    constexpr double maximumIntegrationStep = 1.0e-6;
    TransientSolverOptions options;

    expect(
        options.validFor(maximumIntegrationStep),
        "default solver options are valid"
    );

    options.relativeTolerance = 0.0;
    options.safetyFactor = 1.0;
    options.maximumScale = 1.0;
    options.maximumRejects = 0;
    expect(
        options.validFor(maximumIntegrationStep),
        "inclusive solver-option boundaries are valid"
    );

    options = TransientSolverOptions{};
    expect(!options.validFor(0.0), "zero maximum integration step is invalid");
    expect(
        !options.validFor(std::numeric_limits<double>::infinity()),
        "infinite maximum integration step is invalid"
    );

    options = TransientSolverOptions{};
    options.relativeTolerance = -1.0;
    expect(!options.validFor(maximumIntegrationStep), "negative relative tolerance is invalid");

    options = TransientSolverOptions{};
    options.voltageAbsoluteTolerance = 0.0;
    expect(!options.validFor(maximumIntegrationStep), "zero voltage tolerance is invalid");

    options = TransientSolverOptions{};
    options.currentAbsoluteTolerance = std::numeric_limits<double>::quiet_NaN();
    expect(!options.validFor(maximumIntegrationStep), "NaN current tolerance is invalid");

    options = TransientSolverOptions{};
    options.minimumStep = 0.0;
    expect(!options.validFor(maximumIntegrationStep), "zero minimum step is invalid");

    options = TransientSolverOptions{};
    options.minimumStep = 2.0 * maximumIntegrationStep;
    expect(
        !options.validFor(maximumIntegrationStep),
        "minimum step above maximum integration step is invalid"
    );

    options = TransientSolverOptions{};
    options.safetyFactor = 1.1;
    expect(!options.validFor(maximumIntegrationStep), "safety factor above one is invalid");

    options = TransientSolverOptions{};
    options.minimumScale = 1.0;
    expect(!options.validFor(maximumIntegrationStep), "minimum scale of one is invalid");

    options = TransientSolverOptions{};
    options.maximumScale = 0.9;
    expect(!options.validFor(maximumIntegrationStep), "maximum scale below one is invalid");

    options = TransientSolverOptions{};
    options.convergenceFailureScale = 1.0;
    expect(
        !options.validFor(maximumIntegrationStep),
        "convergence failure scale of one is invalid"
    );

    options = TransientSolverOptions{};
    options.maximumRejects = -1;
    expect(!options.validFor(maximumIntegrationStep), "negative reject limit is invalid");
}

void testPtaConfigValidation(){
    PtaAnalysisConfig config;

    try {
        config.validate();
        expect(true, "default PTA configuration is valid");
    } catch(...) {
        expect(false, "default PTA configuration is valid");
    }

    config = PtaAnalysisConfig{};
    config.voltageSourceInductance = 0.0;
    expectInvalidArgument(
        [&config] { config.validate(); },
        "zero pseudo-inductance is invalid"
    );

    config = PtaAnalysisConfig{};
    config.minimumNodeCapacitance =
        2.0 * config.initialNodeCapacitance;
    expectInvalidArgument(
        [&config] { config.validate(); },
        "node-capacitance bounds must contain the initial value"
    );

    config = PtaAnalysisConfig{};
    config.mediumOscillationScale = config.smallOscillationScale;
    expectInvalidArgument(
        [&config] { config.validate(); },
        "oscillation scales must be strictly ordered"
    );

    config = PtaAnalysisConfig{};
    config.mediumOscillationRatio = config.heavyOscillationRatio;
    expectInvalidArgument(
        [&config] { config.validate(); },
        "oscillation ratios must be strictly ordered"
    );

    config = PtaAnalysisConfig{};
    config.successfulStepScale = 1.0;
    expectInvalidArgument(
        [&config] { config.validate(); },
        "successful-step scale must be greater than one"
    );

    config = PtaAnalysisConfig{};
    config.derivativeRelativeTolerance = -1.0;
    expectInvalidArgument(
        [&config] { config.validate(); },
        "negative derivative relative tolerance is invalid"
    );
}

void testPtaNormalizedDerivative(){
    PtaAnalysisConfig config;
    config.derivativeRelativeTolerance = 0.1;
    config.derivativeVoltageAbsoluteTolerance = 1.0;
    config.derivativeCurrentAbsoluteTolerance = 1.0e-2;

    const PtaDerivativeEstimate estimate = estimatePtaNormalizedDerivative(
        makeVector({1.0, 1.0e-2}),
        makeVector({10.0, 1.0e-1}),
        makeVector({8.0, 5.0e-2}),
        1,
        2.0,
        config
    );
    expect(estimate.valid, "scaled PTA derivative estimate is valid");
    expectNear(
        estimate.normalizedDerivative,
        1.0,
        "PTA derivative uses time step and voltage/current scales"
    );

    const PtaDerivativeEstimate smallerStepEstimate =
        estimatePtaNormalizedDerivative(
            makeVector({1.0, 1.0e-2}),
            makeVector({10.0, 1.0e-1}),
            makeVector({8.0, 5.0e-2}),
            1,
            0.5,
            config
        );
    expect(smallerStepEstimate.valid, "short-step PTA derivative is valid");
    expectNear(
        smallerStepEstimate.normalizedDerivative,
        0.25,
        "PTA derivative normalization scales with pseudo-time step"
    );
}

void testPtaNodeCapacitanceGrowth(){
    PtaAnalysisConfig config;
    config.capacitanceGrowScale = 2.0;
    config.maximumNodeCapacitance = 1.0e-3;

    Circuit circuit;
    CircuitPtaTestAccess::addNodeCapacitor(
        circuit,
        0,
        1.0e-12,
        0.25,
        true
    );
    CircuitPtaTestAccess::addNodeCapacitor(
        circuit,
        1,
        7.5e-4,
        -0.5,
        true
    );

    expect(
        CircuitPtaTestAccess::growAll(circuit, config),
        "PTA growth reports a changed node capacitance"
    );
    expectNear(
        CircuitPtaTestAccess::capacitance(circuit, 0),
        2.0e-12,
        "PTA growth scales a node capacitance"
    );
    expectNear(
        CircuitPtaTestAccess::capacitance(circuit, 1),
        config.maximumNodeCapacitance,
        "PTA growth clamps a node capacitance at its maximum"
    );
    expectNear(
        CircuitPtaTestAccess::previousDelta(circuit, 0),
        0.0,
        "PTA growth clears the first node oscillation delta"
    );
    expect(
        !CircuitPtaTestAccess::hasPreviousDelta(circuit, 0),
        "PTA growth clears the first node oscillation history flag"
    );
    expectNear(
        CircuitPtaTestAccess::previousDelta(circuit, 1),
        0.0,
        "PTA growth clears the capped node oscillation delta"
    );
    expect(
        !CircuitPtaTestAccess::hasPreviousDelta(circuit, 1),
        "PTA growth clears the capped node oscillation history flag"
    );

    Circuit saturatedCircuit;
    CircuitPtaTestAccess::addNodeCapacitor(
        saturatedCircuit,
        0,
        config.maximumNodeCapacitance,
        0.0,
        false
    );
    expect(
        !CircuitPtaTestAccess::growAll(saturatedCircuit, config),
        "PTA growth fails when every node capacitance is saturated"
    );
}

void testPtaNodeCapacitanceOscillationAdaptation(){
    PtaAnalysisConfig config;
    config.minimumNodeCapacitance = 3.0e-2;
    config.smallOscillationScale = 0.9;
    config.mediumOscillationScale = 0.7;
    config.heavyOscillationScale = 0.5;
    config.mediumOscillationRatio = 0.5;
    config.heavyOscillationRatio = 1.0;

    Circuit circuit;
    CircuitPtaTestAccess::addNodeCapacitor(
        circuit,
        0,
        1.0e-1,
        0.0,
        false
    );

    CircuitPtaTestAccess::updateAfterAcceptedStep(
        circuit,
        makeVector({1.1}),
        makeVector({1.0}),
        config
    );
    expectNear(
        CircuitPtaTestAccess::capacitance(circuit, 0),
        1.0e-1,
        "first PTA node delta does not change capacitance"
    );
    expectNear(
        CircuitPtaTestAccess::previousDelta(circuit, 0),
        1.0e-1,
        "first PTA node delta is recorded"
    );
    expect(
        CircuitPtaTestAccess::hasPreviousDelta(circuit, 0),
        "first PTA node delta enables oscillation history"
    );

    CircuitPtaTestAccess::updateAfterAcceptedStep(
        circuit,
        makeVector({1.2}),
        makeVector({1.1}),
        config
    );
    expectNear(
        CircuitPtaTestAccess::capacitance(circuit, 0),
        1.0e-1,
        "same-sign PTA node delta does not change capacitance"
    );

    CircuitPtaTestAccess::updateAfterAcceptedStep(
        circuit,
        makeVector({0.96}),
        makeVector({1.0}),
        config
    );
    expectNear(
        CircuitPtaTestAccess::capacitance(circuit, 0),
        9.0e-2,
        "small PTA oscillation applies the small capacitance scale"
    );

    CircuitPtaTestAccess::updateAfterAcceptedStep(
        circuit,
        makeVector({1.03}),
        makeVector({1.0}),
        config
    );
    expectNear(
        CircuitPtaTestAccess::capacitance(circuit, 0),
        6.3e-2,
        "medium PTA oscillation applies the medium capacitance scale"
    );

    CircuitPtaTestAccess::updateAfterAcceptedStep(
        circuit,
        makeVector({0.94}),
        makeVector({1.0}),
        config
    );
    expectNear(
        CircuitPtaTestAccess::capacitance(circuit, 0),
        3.15e-2,
        "heavy PTA oscillation applies the heavy capacitance scale"
    );

    CircuitPtaTestAccess::updateAfterAcceptedStep(
        circuit,
        makeVector({1.12}),
        makeVector({1.0}),
        config
    );
    expectNear(
        CircuitPtaTestAccess::capacitance(circuit, 0),
        config.minimumNodeCapacitance,
        "PTA oscillation adaptation clamps capacitance at its minimum"
    );
}

void testPtaMinimumStepCapacitanceRecovery(){
    PtaAnalysisConfig config;
    config.mode = PtaMode::Force;
    config.initialStep = 1.0;
    config.minimumStep = 1.0;
    config.maximumStep = 1024.0;
    config.maximumSteps = 100;
    config.derivativeTolerance = 1.0;
    config.dcResidualTolerance = 1.0e-6;
    config.initialNodeCapacitance = 1.0e-6;
    config.minimumNodeCapacitance = 1.0e-6;
    config.maximumNodeCapacitance = 1.0;
    config.capacitanceGrowScale = 1.0e6;
    config.successfulStepScale = 2.0;

    Circuit circuit;
    circuit.addDevice<PtaRecoveryDevice>(
        "XPTA",
        std::vector<std::string>{"node", "0", "0"},
        1.0,
        1000.0
    );

    expect(circuit.build(config), "PTA recovery fixture builds");
    expect(
        circuit.solveAdaptivePta(config),
        "PTA recovers after a minimum-step capacitance increase"
    );
    expect(
        CircuitPtaTestAccess::ptaMinimumStepRecoveries(circuit) > 0,
        "PTA recovery fixture reaches the minimum-step recovery path"
    );
    expect(
        CircuitPtaTestAccess::ptaCapacitanceGrowths(circuit) > 0,
        "PTA recovery fixture records a capacitance growth"
    );
    expect(
        CircuitPtaTestAccess::ptaCapacitanceReductions(circuit) > 0,
        "PTA recovery fixture records a node-capacitance reduction"
    );
}

void testRequiresBdf2History(){
    TransientIntegrator integrator;
    integrator.Initialize(0.0, makeVector({0.0}));

    const TransientErrorEstimate estimate = integrator.estimateError(
        1.0,
        makeVector({0.0}),
        1,
        TransientSolverOptions{}
    );

    expectDefaultInvalid(estimate, "BE startup has no BDF2 estimate");
}

void testZeroErrorUsesMaximumScale(){
    const TransientIntegrator integrator = makeIntegratorWithHistory(
        makeVector({1.0, 1.0}),
        makeVector({2.0, 3.0})
    );
    TransientSolverOptions options = makeAbsoluteOnlyOptions();

    const TransientErrorEstimate estimate = integrator.estimateError(
        2.0,
        makeVector({3.0, 5.0}),
        1,
        options
    );

    expect(estimate.valid, "zero-error BDF2 estimate is valid");
    expectNear(estimate.normalizedError, 0.0, "zero normalized error");
    expectNear(
        estimate.suggestedScale,
        options.maximumScale,
        "zero error uses maximum scale"
    );
}

void testVoltageAndCurrentAbsoluteTolerances(){
    const TransientIntegrator integrator = makeIntegratorWithHistory(
        makeVector({0.0, 0.0}),
        makeVector({0.0, 0.0})
    );
    TransientSolverOptions options = makeAbsoluteOnlyOptions();
    options.voltageAbsoluteTolerance = 2.0;
    options.currentAbsoluteTolerance = 0.5;

    const TransientErrorEstimate estimate = integrator.estimateError(
        2.0,
        makeVector({1.0, 1.0}),
        1,
        options
    );

    expect(estimate.valid, "mixed voltage/current estimate is valid");
    expectNear(
        estimate.normalizedError,
        2.0,
        "current unknown uses current absolute tolerance"
    );
    expectNear(
        estimate.suggestedScale,
        options.safetyFactor / std::sqrt(2.0),
        "mixed tolerance scale"
    );

    const TransientErrorEstimate voltageOnlyEstimate = integrator.estimateError(
        2.0,
        makeVector({1.0, 0.0}),
        1,
        options
    );
    expectNear(
        voltageOnlyEstimate.normalizedError,
        0.5,
        "voltage unknown uses voltage absolute tolerance"
    );
}

void testRelativeWeight(){
    const TransientIntegrator integrator = makeIntegratorWithHistory(
        makeVector({1.0}),
        makeVector({2.0})
    );
    TransientSolverOptions options = makeAbsoluteOnlyOptions();
    options.relativeTolerance = 0.5;

    const TransientErrorEstimate estimate = integrator.estimateError(
        2.0,
        makeVector({4.0}),
        1,
        options
    );

    const double expectedError = 1.0 / 3.0;
    expect(estimate.valid, "relative-weight estimate is valid");
    expectNear(
        estimate.normalizedError,
        expectedError,
        "relative weight uses max(corrected, accepted)"
    );
    expectNear(
        estimate.suggestedScale,
        options.safetyFactor / std::sqrt(expectedError),
        "relative-weight suggested scale"
    );
}

void testScaleBoundaries(){
    const TransientIntegrator integrator = makeIntegratorWithHistory(
        makeVector({0.0}),
        makeVector({0.0})
    );
    const TransientSolverOptions options = makeAbsoluteOnlyOptions();

    const TransientErrorEstimate unitError = integrator.estimateError(
        2.0,
        makeVector({1.0}),
        1,
        options
    );
    expectNear(unitError.normalizedError, 1.0, "unit normalized error");
    expectNear(
        unitError.suggestedScale,
        options.safetyFactor,
        "unit error uses safety factor"
    );

    const TransientErrorEstimate smallError = integrator.estimateError(
        2.0,
        makeVector({0.01}),
        1,
        options
    );
    expectNear(smallError.normalizedError, 0.01, "small normalized error");
    expectNear(
        smallError.suggestedScale,
        options.maximumScale,
        "small error clamps to maximum scale"
    );

    const TransientErrorEstimate largeError = integrator.estimateError(
        2.0,
        makeVector({100.0}),
        1,
        options
    );
    expectNear(largeError.normalizedError, 100.0, "large normalized error");
    expectNear(
        largeError.suggestedScale,
        options.minimumScale,
        "large error clamps to minimum scale"
    );
}

void testVariableStepPredictor(){
    const TransientIntegrator integrator = makeIntegratorWithHistory(
        makeVector({0.0}),
        makeVector({4.0}),
        2.0
    );
    TransientSolverOptions options = makeAbsoluteOnlyOptions();
    options.voltageAbsoluteTolerance = 2.0;

    const TransientErrorEstimate estimate = integrator.estimateError(
        3.0,
        makeVector({7.0}),
        1,
        options
    );

    expect(estimate.valid, "variable-step estimate is valid");
    expectNear(
        estimate.normalizedError,
        0.5,
        "variable-step ratio is used by predictor"
    );
    expectNear(
        estimate.suggestedScale,
        options.safetyFactor / std::sqrt(0.5),
        "variable-step suggested scale"
    );
}

void testMaximumBdf2StepRatio(){
    const TransientIntegrator integrator = makeIntegratorWithHistory(
        makeVector({0.0}),
        makeVector({1.0})
    );
    const TransientSolverOptions options = makeAbsoluteOnlyOptions();

    const TransientErrorEstimate boundaryEstimate = integrator.estimateError(
        3.0,
        makeVector({4.0}),
        1,
        options
    );
    expect(boundaryEstimate.valid, "BDF2 ratio 2 boundary is valid");
    expectNear(
        boundaryEstimate.normalizedError,
        1.0,
        "BDF2 ratio 2 predictor"
    );

    const double aboveBoundary = std::nextafter(
        3.0,
        std::numeric_limits<double>::infinity()
    );
    const TransientErrorEstimate fallbackEstimate = integrator.estimateError(
        aboveBoundary,
        makeVector({4.0}),
        1,
        options
    );
    expectDefaultInvalid(
        fallbackEstimate,
        "step ratio above 2 falls back to BE"
    );
}

void testInvalidInputs(){
    const TransientIntegrator integrator = makeIntegratorWithHistory(
        makeVector({0.0}),
        makeVector({0.0})
    );
    const TransientSolverOptions validOptions = makeAbsoluteOnlyOptions();

    expectDefaultInvalid(
        integrator.estimateError(
            2.0, makeVector({0.0, 0.0}), 1, validOptions
        ),
        "solution size mismatch"
    );
    expectDefaultInvalid(
        integrator.estimateError(2.0, makeVector({0.0}), -1, validOptions),
        "negative voltage unknown count"
    );
    expectDefaultInvalid(
        integrator.estimateError(2.0, makeVector({0.0}), 2, validOptions),
        "voltage unknown count exceeds solution size"
    );
    expectDefaultInvalid(
        integrator.estimateError(
            2.0,
            makeVector({std::numeric_limits<double>::quiet_NaN()}),
            1,
            validOptions
        ),
        "NaN corrected solution"
    );
    expectDefaultInvalid(
        integrator.estimateError(
            2.0,
            makeVector({std::numeric_limits<double>::infinity()}),
            1,
            validOptions
        ),
        "infinite corrected solution"
    );
    expectDefaultInvalid(
        integrator.estimateError(
            std::numeric_limits<double>::quiet_NaN(),
            makeVector({0.0}),
            1,
            validOptions
        ),
        "NaN target time"
    );
    expectDefaultInvalid(
        integrator.estimateError(1.0, makeVector({0.0}), 1, validOptions),
        "target time does not advance"
    );

    const TransientIntegrator emptyIntegrator = makeIntegratorWithHistory(
        Eigen::VectorXd{},
        Eigen::VectorXd{}
    );
    expectDefaultInvalid(
        emptyIntegrator.estimateError(
            2.0, Eigen::VectorXd{}, 0, validOptions
        ),
        "empty solution"
    );

    TransientSolverOptions invalidOptions = validOptions;
    invalidOptions.relativeTolerance = -1.0;
    expectDefaultInvalid(
        integrator.estimateError(2.0, makeVector({0.0}), 1, invalidOptions),
        "negative relative tolerance"
    );

    invalidOptions = validOptions;
    invalidOptions.voltageAbsoluteTolerance = 0.0;
    expectDefaultInvalid(
        integrator.estimateError(2.0, makeVector({0.0}), 1, invalidOptions),
        "zero voltage absolute tolerance"
    );

    invalidOptions = validOptions;
    invalidOptions.currentAbsoluteTolerance = 0.0;
    expectDefaultInvalid(
        integrator.estimateError(2.0, makeVector({0.0}), 1, invalidOptions),
        "zero current absolute tolerance"
    );

    invalidOptions = validOptions;
    invalidOptions.safetyFactor = std::numeric_limits<double>::infinity();
    expectDefaultInvalid(
        integrator.estimateError(2.0, makeVector({0.0}), 1, invalidOptions),
        "infinite safety factor"
    );

    invalidOptions = validOptions;
    invalidOptions.minimumScale = 0.0;
    expectDefaultInvalid(
        integrator.estimateError(2.0, makeVector({0.0}), 1, invalidOptions),
        "zero minimum scale"
    );

    invalidOptions = validOptions;
    invalidOptions.maximumScale = invalidOptions.minimumScale / 2.0;
    expectDefaultInvalid(
        integrator.estimateError(2.0, makeVector({0.0}), 1, invalidOptions),
        "maximum scale below minimum scale"
    );

    const TransientIntegrator nonFiniteHistory = makeIntegratorWithHistory(
        makeVector({0.0}),
        makeVector({std::numeric_limits<double>::infinity()})
    );
    expectDefaultInvalid(
        nonFiniteHistory.estimateError(
            2.0, makeVector({0.0}), 1, validOptions
        ),
        "non-finite accepted history"
    );

    const double maximumFinite = std::numeric_limits<double>::max();
    const TransientIntegrator overflowingWeight = makeIntegratorWithHistory(
        makeVector({maximumFinite}),
        makeVector({maximumFinite})
    );
    TransientSolverOptions overflowingWeightOptions = validOptions;
    overflowingWeightOptions.relativeTolerance = 2.0;
    expectDefaultInvalid(
        overflowingWeight.estimateError(
            2.0,
            makeVector({maximumFinite}),
            1,
            overflowingWeightOptions
        ),
        "overflowing error weight"
    );
}

// Characterization test for the current O(h^2) linear-predictor proxy.
// Replace this expectation if estimateError becomes a strict BDF2 LTE estimate.
void testQuadraticSolutionCharacterizesProxy(){
    const TransientIntegrator integrator = makeIntegratorWithHistory(
        makeVector({0.0}),
        makeVector({1.0})
    );
    const TransientSolverOptions options = makeAbsoluteOnlyOptions();

    const TransientErrorEstimate estimate = integrator.estimateError(
        2.0,
        makeVector({4.0}),
        1,
        options
    );

    expect(estimate.valid, "quadratic proxy estimate is valid");
    expectNear(
        estimate.normalizedError,
        2.0,
        "quadratic solution records linear-predictor defect"
    );
    expectNear(
        estimate.suggestedScale,
        options.safetyFactor / std::sqrt(2.0),
        "quadratic proxy uses second-order scale exponent"
    );
}

void testTransientStepController(){
    TransientSolverOptions options = makeAbsoluteOnlyOptions();
    options.minimumStep = 0.1;
    options.maximumRejects = 2;
    options.convergenceFailureScale = 0.25;

    TransientStepControlInput input;
    input.converged = true;
    input.requiresBdf2 = true;
    input.integrationOrder = 2;
    input.errorEstimateValid = true;
    input.normalizedError = 0.25;
    input.suggestedStepScale = 1.5;
    input.currentTime = 1.0;
    input.attemptedStep = 0.5;
    input.hardStepLimit = 1.0;

    TransientStepControlDecision decision = decideTransientStep(
        input,
        options
    );

    expect(
        decision.action == TransientStepAction::Accept,
        "accepted BDF2 attempt is committed"
    );
    expectNear(
        decision.nextStep,
        0.75,
        "accepted BDF2 attempt returns suggested next step"
    );

    input.normalizedError = 4.0;
    input.suggestedStepScale = 0.25;
    decision = decideTransientStep(input, options);

    expect(
        decision.action == TransientStepAction::RetryError,
        "large BDF2 error is retried"
    );
    expectNear(
        decision.nextStep,
        0.125,
        "error retry uses suggested scale"
    );

    input.converged = false;
    decision = decideTransientStep(input, options);

    expect(
        decision.action == TransientStepAction::RetryConvergence,
        "Newton failure is retried"
    );
    expectNear(
        decision.nextStep,
        0.125,
        "Newton failure uses convergence-failure scale"
    );

    input.converged = true;
    input.errorEstimateValid = false;
    decision = decideTransientStep(input, options);

    expect(
        decision.action == TransientStepAction::FailInvalidEstimate,
        "invalid required estimate is never accepted"
    );

    input.errorEstimateValid = true;
    input.integrationOrder = 1;
    decision = decideTransientStep(input, options);

    expect(
        decision.action == TransientStepAction::FailInvalidEstimate,
        "BDF2 history cannot silently fall back to BE"
    );

    input.requiresBdf2 = false;
    input.integrationOrder = 1;
    input.normalizedError = 0.0;
    input.suggestedStepScale = 1.0;
    decision = decideTransientStep(input, options);

    expect(
        decision.action == TransientStepAction::Accept,
        "BE step-doubling result is accepted with a valid estimate"
    );

    input.requiresBdf2 = true;
    input.integrationOrder = 2;
    input.normalizedError = 4.0;
    input.suggestedStepScale = 0.25;
    input.rejectedAttempts = options.maximumRejects;
    decision = decideTransientStep(input, options);

    expect(
        decision.action == TransientStepAction::FailRejectLimit,
        "reject limit prevents an additional retry"
    );

    input.rejectedAttempts = 0;
    input.attemptedStep = 0.11;
    input.suggestedStepScale = 0.2;
    decision = decideTransientStep(input, options);

    expect(
        decision.action == TransientStepAction::RetryError,
        "one final retry at the minimum step is allowed"
    );
    expectNear(
        decision.nextStep,
        0.1,
        "error retry is clamped to the minimum step"
    );

    input.attemptedStep = 0.1;
    decision = decideTransientStep(input, options);

    expect(
        decision.action == TransientStepAction::FailMinimumStep,
        "minimum step cannot be retried indefinitely"
    );
}

void testStepDoublingDifferenceEstimate(){
    const TransientSolverOptions options = makeAbsoluteOnlyOptions();

    const TransientErrorEstimate estimate =
        estimateTransientSolutionDifference(
            makeVector({0.0}),
            makeVector({2.0}),
            makeVector({1.0}),
            1,
            options
        );

    expect(estimate.valid, "step-doubling estimate is valid");
    expectNear(
        estimate.normalizedError,
        1.0,
        "step-doubling compares fine and coarse endpoints"
    );
    expectNear(
        estimate.suggestedScale,
        options.safetyFactor,
        "unit step-doubling error uses safety factor"
    );

    expectDefaultInvalid(
        estimateTransientSolutionDifference(
            makeVector({0.0}),
            makeVector({std::numeric_limits<double>::quiet_NaN()}),
            makeVector({1.0}),
            1,
            options
        ),
        "non-finite fine endpoint invalidates step-doubling estimate"
    );
}

void testRestartForcesBackwardEuler(){
    TransientIntegrator integrator = makeIntegratorWithHistory(
        makeVector({0.0}),
        makeVector({1.0})
    );

    integrator.restartFrom(1.0, makeVector({1.0}));

    expect(
        integrator.coefficients(1.5).order == 1,
        "restart clears history and forces BE"
    );
    expect(
        !integrator.canUseBdf2(1.5),
        "restarted integrator cannot select BDF2"
    );
}

void testNewtonStepLimiting(){
    Eigen::VectorXd previous = makeVector({1.0, -2.0});
    Eigen::VectorXd current = makeVector({4.0, -2.5});

    const NewtonStepResult limited = limitNewtonStep(
        current,
        previous,
        1.0
    );
    expect(limited.limited, "large Newton step is limited");
    expectNear(limited.delta, 1.0, "limited Newton step reports bound");
    expectNear(current[0], 2.0, "limited Newton step updates first value");
    expectNear(
        current[1],
        -2.0 - (0.5 / 3.0),
        "limited Newton step preserves update direction"
    );

    current = makeVector({1.2, -2.1});
    const NewtonStepResult unchanged = limitNewtonStep(
        current,
        previous,
        1.0
    );
    expect(!unchanged.limited, "small Newton step is unchanged");
    expectNear(unchanged.delta, 0.2, "small Newton step reports raw delta");

    current = makeVector({std::numeric_limits<double>::quiet_NaN(), -2.0});
    const NewtonStepResult invalid = limitNewtonStep(
        current,
        previous,
        1.0
    );
    expect(!std::isfinite(invalid.delta), "non-finite Newton step is rejected");
}

} // namespace

int main(){
    testSolverOptionsValidation();
    testPtaConfigValidation();
    testPtaNormalizedDerivative();
    testPtaNodeCapacitanceGrowth();
    testPtaNodeCapacitanceOscillationAdaptation();
    testPtaMinimumStepCapacitanceRecovery();
    testRequiresBdf2History();
    testZeroErrorUsesMaximumScale();
    testVoltageAndCurrentAbsoluteTolerances();
    testRelativeWeight();
    testScaleBoundaries();
    testVariableStepPredictor();
    testMaximumBdf2StepRatio();
    testInvalidInputs();
    testQuadraticSolutionCharacterizesProxy();
    testTransientStepController();
    testStepDoublingDifferenceEstimate();
    testRestartForcesBackwardEuler();
    testNewtonStepLimiting();

    if(failureCount != 0){
        std::cerr << failureCount << " of " << checkCount
                  << " transient analysis checks failed\n";
        return 1;
    }

    std::cout << "Transient analysis unit tests: " << checkCount
              << "/" << checkCount << " checks passed\n";
    return 0;
}
