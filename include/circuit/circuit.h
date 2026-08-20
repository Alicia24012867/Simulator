#pragma once
#include <Eigen/Core>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class Device;
class PseudoDevice;
class PseudoCapacitor;
class MNA;
class Model;
class NodeMap;
class SpiceOutputAccess;
class CircuitPtaTestAccess;
class TransientIntegrator;

struct TransientAnalysisConfig;
struct TransientStampContext;
struct TransientSolverOptions;
struct PendingPtaPlacement;
struct PtaAnalysisConfig;
struct PtaDiagnostics;
struct NewtonSolverOptions;
struct OperatingPointSolverOptions;
class Circuit{
public:
    Circuit();
    ~Circuit();

    template<class T, class... Args>
    void addDevice(Args&&... args){
        devices_.emplace_back(
            std::make_unique<T>(
                std::forward<Args>(args)...
            )
        );
    }

    const Model* addModel(std::unique_ptr<Model> model);

    const Model* findModel(const std::string& name) const;

    bool build(const PtaAnalysisConfig& config);

    int allocateUnknown();

    bool solve(); // Backward-compatible OP entry point.

    bool solveOperatingPoint();

    bool solveOperatingPoint(
        const OperatingPointSolverOptions& options
    );

    bool solveTransient(const TransientAnalysisConfig& config);

    bool solveTransient(
        const TransientAnalysisConfig& config,
        const OperatingPointSolverOptions& operatingPointOptions
    );

    bool solveAdaptivePta(const PtaAnalysisConfig& config);

    PtaDiagnostics ptaDiagnostics() const;

private:
    friend class SpiceOutputAccess;
    friend class CircuitPtaTestAccess;

    using AssembleCallback = std::function<void()>;

    struct OperatingPointStats {
        bool converged = false;
        int iterations = 0;
        int maxIterations = 0;
        int dampedSteps = 0;
        int sourceSteps = 0;
        int failedSourceSteps = 0;
        int ptaCapacitanceGrowths = 0;
        int ptaCapacitanceReductions = 0;
        int ptaMinimumStepRecoveries = 0;
        bool ptaAttempted = false;
        bool hasPtaConvergenceMetrics = false;
        double finalDelta = 0.0;
        double tolerance = 0.0;
        double cpuSeconds = 0.0;
        double sourceScale = 0.0;
        double minSourceStep = 0.0;
        double ptaNormalizedDerivative = 0.0;
        double ptaNormalizedDcResidual = 0.0;
    };

    struct NewtonStats {
        int iterations = 0;
        int dampedSteps = 0;
        double finalDelta = 0.0;
    };

    struct TransientStepAttempt{
        int integrationOrder = 1;
        bool errorEstimateValid = false;
        double normalizedError = 0.0;
        double suggestedStepScale = 1.0;
        bool converged = false;
        NewtonStats newtonStats;
        Eigen::VectorXd solution;
    };

    struct TransientStats {
        bool converged = false;
        int timeSteps = 0;
        int outputPoints = 0;
        int iterations = 0;
        int maxIterations = 0;
        int dampedSteps = 0;
        double finalTime = 0.0;
        double finalDelta = 0.0;
        double tolerance = 0.0;
        double initializationCpuSeconds = 0.0;
        double cpuSeconds = 0.0;

        int attemptedSteps = 0;
        int rejectedSteps = 0;
        int convergenceRejectedSteps = 0;
        int errorRejectedSteps = 0;
        int invalidEstimateFailures = 0;
    };

    struct TransientSample {
        double time = 0.0;
        Eigen::VectorXd solution;
    };

    struct PtaNodeCapState{
        int node = -1;
        PseudoCapacitor* capacitor = nullptr;
        double capacitance = 0.0;
        double previousDelta = 0.0;
        bool hasPreviousDelta = false;
    };

    TransientStepAttempt tryTransientStep(
        const TransientIntegrator& integrator,
        double targetTime,
        const TransientSolverOptions& options
    );

    void addTransientStats(const NewtonStats& stats);

    void restoreTransientCheckpoint( 
        const Eigen::VectorXd& acceptedSolution
    );

    bool solveNewtonSystem(const AssembleCallback& assemble,
                           NewtonStats& stats,
                           const NewtonSolverOptions& options);

    bool solveLinearSystem(const AssembleCallback& assemble,
                           NewtonStats& stats);

    bool solveOperatingPointWithSourceStepping(
        const AssembleCallback& assemble,
        const OperatingPointSolverOptions& options
    );

    void addNewtonStats(const NewtonStats& stats);

    void cacheOperatingPointDeviceRoles();

    void assembleOperatingPointSystem();

    void assembleTransientSystem(const TransientStampContext& ctx);

    void assemblePtaSystem(const TransientStampContext& ctx);

    bool hasNonlinearDevices() const;

    void setOperatingPointSourceScale(double scale);

    void initializePtaStates(const PtaAnalysisConfig& config, double time);

    void saveNonlinearIterationStates();

    void restoreNonlinearIterationStates();

    void recordTransientSample(double time);

    void collectPendingPtaPlacements(const PtaAnalysisConfig& config);

    void materializePseudoDevices(const PtaAnalysisConfig& config);

    bool growAllPtaNodeCapacitances(const PtaAnalysisConfig& config);

    void updatePtaNodeCapacitancesAfterAcceptedStep(
        const Eigen::VectorXd& currentSolution,
        const Eigen::VectorXd& previousSolution,
        const PtaAnalysisConfig& config
    );

    std::unique_ptr<MNA> mna_;

    std::vector<std::unique_ptr<Device>> devices_;

    std::vector<Device*> sourceSteppingDevices_;

    std::vector<Device*> iterationStateDevices_;

    std::vector<std::unique_ptr<PseudoDevice>> pseudoDevices_;

    std::unordered_map<std::string, std::unique_ptr<Model>> models_;

    int nextUnknown_ = 0;

    bool hasNonlinearDevices_ = false;

    double operatingPointSourceScale_ = 1.0;

    std::unique_ptr<NodeMap> nodeMap_;

    OperatingPointStats operatingPointStats_;

    TransientStats transientStats_;

    std::vector<TransientSample> transientSamples_;

    std::vector<PendingPtaPlacement> pendingPtaPlacements_;

    std::vector<PtaNodeCapState> ptaNodeCaps_;
};
