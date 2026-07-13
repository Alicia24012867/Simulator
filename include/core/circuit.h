#pragma once
#include <Eigen/Core>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class Device;
class MNA;
class Model;
class NodeMap;
class SpiceOutputAccess;
struct TransientAnalysisConfig;
struct TransientStampContext;

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

    bool build();

    int allocateUnknown();

    bool solve(); // Backward-compatible OP entry point.

    bool solveOperatingPoint();

    bool solveTransient(const TransientAnalysisConfig& config);

private:
    friend class SpiceOutputAccess;

    using AssembleCallback = std::function<void()>;

    struct OperatingPointStats {
        bool converged = false;
        int iterations = 0;
        int maxIterations = 0;
        int dampedSteps = 0;
        int sourceSteps = 0;
        int failedSourceSteps = 0;
        double finalDelta = 0.0;
        double tolerance = 0.0;
        double cpuSeconds = 0.0;
        double sourceScale = 0.0;
        double minSourceStep = 0.0;
    };

    struct NewtonStats {
        int iterations = 0;
        int dampedSteps = 0;
        double finalDelta = 0.0;
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
    };

    struct TransientSample {
        double time = 0.0;
        Eigen::VectorXd solution;
    };

    bool solveNewtonSystem(const AssembleCallback& assemble,
                           NewtonStats& stats);

    bool solveLinearSystem(const AssembleCallback& assemble,
                           NewtonStats& stats);

    bool solveOperatingPointWithSourceStepping(
        const AssembleCallback& assemble);

    void addNewtonStats(const NewtonStats& stats);

    void cacheOperatingPointDeviceRoles();

    void assembleOperatingPointSystem();

    void assembleTransientSystem(const TransientStampContext& ctx);

    bool hasNonlinearDevices() const;

    void setOperatingPointSourceScale(double scale);

    void saveNonlinearIterationStates();

    void restoreNonlinearIterationStates();

    void recordTransientSample(double time);

    std::unique_ptr<MNA> mna_;

    std::vector<std::unique_ptr<Device>> devices_;

    std::vector<Device*> sourceSteppingDevices_;

    std::vector<Device*> iterationStateDevices_;

    std::unordered_map<std::string, std::unique_ptr<Model>> models_;

    int nextUnknown_ = 0;

    bool hasNonlinearDevices_ = false;

    double operatingPointSourceScale_ = 1.0;

    std::unique_ptr<NodeMap> nodeMap_;

    OperatingPointStats operatingPointStats_;

    TransientStats transientStats_;

    std::vector<TransientSample> transientSamples_;
};
