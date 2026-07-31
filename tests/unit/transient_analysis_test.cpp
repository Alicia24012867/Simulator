#include "analysis/transientAnalysis.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string>

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

} // namespace

int main(){
    testSolverOptionsValidation();
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

    if(failureCount != 0){
        std::cerr << failureCount << " of " << checkCount
                  << " transient analysis checks failed\n";
        return 1;
    }

    std::cout << "Transient analysis unit tests: " << checkCount
              << "/" << checkCount << " checks passed\n";
    return 0;
}
