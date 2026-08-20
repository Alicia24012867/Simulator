#pragma once

#include <optional>

namespace simulator::config {
struct LoadedConfig;

enum class PtaModeOverride {
    Disabled,
    Force,
    Fallback
};

template<class T>
struct NullableOverride {
    bool specified {false};
    std::optional<T> value;
};

struct PtaOverrides {
    std::optional<PtaModeOverride> mode;

    std::optional<double> initialStep;
    std::optional<double> minimumStep;
    std::optional<double> maximumStep;
    std::optional<int> maximumSteps;

    std::optional<double> derivativeTolerance;
    std::optional<double> derivativeRelativeTolerance;
    std::optional<double> derivativeVoltageAbsoluteTolerance;
    std::optional<double> derivativeCurrentAbsoluteTolerance;

    std::optional<double> dcResidualTolerance;
    std::optional<double> dcResidualRelativeTolerance;
    std::optional<double> dcVoltageAbsoluteTolerance;
    std::optional<double> dcCurrentAbsoluteTolerance;

    std::optional<double> initialNodeCapacitance;
    std::optional<double> minimumNodeCapacitance;
    std::optional<double> maximumNodeCapacitance;
    std::optional<double> currentSourceCapacitance;
    std::optional<double> voltageSourceInductance;

    std::optional<double> compoundTimeConstant;
    std::optional<double> compoundInitialResistance;
    std::optional<double> compoundInitialConductance;
    std::optional<double> sourceRampTime;
    NullableOverride<double> initialBjtVbe;

    std::optional<double> failedStepScale;
    std::optional<double> successfulStepScale;
    std::optional<double> capacitanceGrowScale;
    std::optional<double> smallOscillationScale;
    std::optional<double> mediumOscillationScale;
    std::optional<double> heavyOscillationScale;
    std::optional<double> mediumOscillationRatio;
    std::optional<double> heavyOscillationRatio;

    std::optional<bool> includeMosBulk;
    std::optional<bool> includeDiodes;
};

struct TransientSolverOverrides {
    std::optional<double> relativeTolerance;
    std::optional<double> voltageAbsoluteTolerance;
    std::optional<double> currentAbsoluteTolerance;
    std::optional<double> minimumStep;

    std::optional<double> safetyFactor;
    std::optional<double> minimumScale;
    std::optional<double> maximumScale;
    std::optional<double> convergenceFailureScale;
    std::optional<int> maximumRejects;
};

struct TransientOverrides {
    std::optional<bool> enabled;
    std::optional<double> outputInterval;
    std::optional<double> stopTime;
    std::optional<double> outputStartTime;
    std::optional<double> maximumStep;
    std::optional<bool> useInitialConditions;
    std::optional<TransientSolverOverrides> solver;
};

struct ConfigOverrides {
    std::optional<int> schemaVersion;
    std::optional<PtaOverrides> pta;
    std::optional<TransientOverrides> transient;
};

/**
 * 将已加载的 json 文档严格解析为覆盖层。
 *
 * 该函数不修改 PtaAnalysisConfig、TransientAnalysisConfig 或 AnalysisPlan。
 * 找不到配置文件时，返回完全为空的 ConfigOverrides。
 *
 * json schema、字段类型、未知字段和数值格式错误时 throw std::runtime_error。
 */
[[nodiscard]] ConfigOverrides parseConfigOverrides(
    const LoadedConfig& loadedConfig
);

}  // namespace simulator::config
