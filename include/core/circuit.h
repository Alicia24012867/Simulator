#pragma once
#include <memory>
#include <iosfwd>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class Device;
class MNA;
class Model;
class NodeMap;

class Circuit{
public:
    Circuit();
    ~Circuit();

    template<class T, class... Args>
    void addDevice(Args&&... args){
        devices.emplace_back(
            std::make_unique<T>(
                std::forward<Args>(args)...
            )
        );
    }

    const Model* addModel(std::unique_ptr<Model> model);

    const Model* findModel(const std::string& name) const;

    bool build();

    int allocateUnknown();

    bool solve();

    void printOperatingPoint(std::ostream& os) const;

private:
    struct SolveStats {
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

    bool solveNewton(NewtonStats& stats);

    bool solveLinear(NewtonStats& stats);

    bool solveWithDynamicSourceStepping();

    void addNewtonStats(const NewtonStats& stats);

    void cacheDeviceRoles();

    void assembleSystem();

    bool hasNonlinearDevices() const;

    void setSourceScale(double scale);

    void saveDeviceStates();

    void restoreDeviceStates();

    std::unique_ptr<MNA> mna;

    std::vector<std::unique_ptr<Device>> devices;

    std::vector<Device*> sourceDevices;

    std::vector<Device*> statefulDevices;

    std::unordered_map<std::string, std::unique_ptr<Model>> models;

    int nextUnknown = 0;

    bool hasNonlinearDevices_ = false;

    double currentSourceScale_ = 1.0;

    std::unique_ptr<NodeMap> nodeMap;

    SolveStats solveStats;
};
