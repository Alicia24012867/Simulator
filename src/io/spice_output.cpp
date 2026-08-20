#include "io/spice_output.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "analysis/analysis_plan.h"
#include "analysis/pta_analysis.h"
#include "circuit/circuit.h"
#include "circuit/node_map.h"
#include "devices/device.hpp"
#include "math/mna.hpp"
#include "utils/string_utils.hpp"

class SpiceOutputAccess {
public:
    static const auto& devices(const Circuit& circuit){
        return circuit.devices_;
    }

    static const NodeMap& nodeMap(const Circuit& circuit){
        return *circuit.nodeMap_;
    }

    static const MNA& mna(const Circuit& circuit){
        return *circuit.mna_;
    }

    static const auto& transientSamples(const Circuit& circuit){
        return circuit.transientSamples_;
    }
};

namespace {
constexpr int kIndexWidth = 8;
constexpr int kValueWidth = 20;

struct ResolvedVariable {
    PrintQuantity quantity = PrintQuantity::Voltage;
    std::string label;
    std::string rawLabel;
    std::string rawType;
    int index = -1;
    int referenceIndex = -1;
};

int branchIndexOf(const Circuit& circuit, const std::string& deviceName){
    for(const auto& device: SpiceOutputAccess::devices(circuit)){
        if(equal_ignore_case(device->getName(), deviceName)){
            const int branch = device->branchUnknown();
            if(branch < 0){
                throw std::runtime_error(
                    "Current output is unavailable for device " + deviceName +
                    "; only devices with a branch unknown are supported"
                );
            }
            return branch;
        }
    }
    throw std::runtime_error("Unknown output device: " + deviceName);
}

ResolvedVariable resolveVariable(const Circuit& circuit,
                                 const PrintVariable& variable){
    ResolvedVariable resolved;
    resolved.quantity = variable.quantity;

    if(variable.quantity == PrintQuantity::Voltage){
        resolved.index = SpiceOutputAccess::nodeMap(circuit).idxOf(variable.name);
        resolved.referenceIndex =
            SpiceOutputAccess::nodeMap(circuit).idxOf(variable.reference);
        resolved.label = "v(" + to_lower_copy(variable.name);
        if(variable.reference != "0" &&
           !equal_ignore_case(variable.reference, "gnd")){
            resolved.label += ',' + to_lower_copy(variable.reference);
        }
        resolved.label += ')';
        resolved.rawLabel = resolved.label;
        resolved.rawType = "voltage";
        return resolved;
    }

    resolved.index = branchIndexOf(circuit, variable.name);
    resolved.label = to_lower_copy(variable.name) + "#branch";
    resolved.rawLabel = "i(" + to_lower_copy(variable.name) + ')';
    resolved.rawType = "current";
    return resolved;
}

std::vector<ResolvedVariable> defaultVariables(const Circuit& circuit){
    std::vector<ResolvedVariable> resolved;

    const auto& nodeNames =
        SpiceOutputAccess::nodeMap(circuit).nodeNameByIdx();
    resolved.reserve(
        nodeNames.size() + SpiceOutputAccess::devices(circuit).size()
    );
    for(std::size_t i = 0; i < nodeNames.size(); ++i){
        resolved.push_back({
            PrintQuantity::Voltage,
            "v(" + to_lower_copy(nodeNames[i]) + ')',
            "v(" + to_lower_copy(nodeNames[i]) + ')',
            "voltage",
            static_cast<int>(i),
            -1
        });
    }

    for(const auto& device: SpiceOutputAccess::devices(circuit)){
        const int branch = device->branchUnknown();
        if(branch >= 0){
            resolved.push_back({
                PrintQuantity::BranchCurrent,
                to_lower_copy(device->getName()) + "#branch",
                "i(" + to_lower_copy(device->getName()) + ')',
                "current",
                branch,
                -1
            });
        }
    }
    return resolved;
}

std::vector<ResolvedVariable> resolveVariables(
    const Circuit& circuit,
    bool printRequested,
    const std::vector<PrintVariable>& requested)
{
    if(!printRequested){
        return defaultVariables(circuit);
    }

    std::vector<ResolvedVariable> resolved;
    resolved.reserve(requested.size());
    for(const auto& variable: requested){
        resolved.push_back(resolveVariable(circuit, variable));
    }
    return resolved;
}

double variableValue(const ResolvedVariable& variable,
                     const Eigen::VectorXd& solution){
    if(variable.quantity == PrintQuantity::BranchCurrent){
        return solution[variable.index];
    }

    const double positive = variable.index < 0
        ? 0.0
        : solution[variable.index];
    const double negative = variable.referenceIndex < 0
        ? 0.0
        : solution[variable.referenceIndex];
    return positive - negative;
}

void ensureFinite(double value){
    if(!std::isfinite(value)){
        throw std::runtime_error("Cannot write NaN or infinity to SPICE output");
    }
}

std::string currentSpiceDate(){
    const std::time_t now = std::time(nullptr);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &now);
#else
    localtime_r(&now, &localTime);
#endif

    std::ostringstream formatted;
    formatted.imbue(std::locale::classic());
    formatted << std::put_time(&localTime, "%a %b %d %H:%M:%S %Y");
    return formatted.str();
}

void writeTableHeader(std::ostream& os,
                      const std::string& title,
                      const std::string& analysisName,
                      std::size_t pointCount,
                      bool includeTime,
                      const std::vector<ResolvedVariable>& variables){
    const std::size_t columnCount = variables.size() + (includeTime ? 1 : 0);
    const std::size_t separatorWidth = std::max<std::size_t>(
        80,
        kIndexWidth + kValueWidth * columnCount
    );

    os << "Circuit: " << title << "\n\n";
    os << analysisName << '\n';
    os << "No. of Data Rows : " << pointCount << "\n\n";
    os << std::string(separatorWidth, '-') << '\n';
    os << std::left;
    if(columnCount == 0){
        os << "Index";
    } else {
        os << std::setw(kIndexWidth) << "Index";
    }
    if(includeTime){
        os << ' ';
        if(variables.empty()){
            os << "time";
        } else {
            os << std::setw(kValueWidth - 1) << "time";
        }
    }
    for(std::size_t i = 0; i < variables.size(); ++i){
        os << ' ';
        if(i + 1 == variables.size()){
            os << variables[i].label;
        } else {
            os << std::setw(kValueWidth - 1) << variables[i].label;
        }
    }
    os << '\n' << std::string(separatorWidth, '-') << '\n';
}

void writeTableRow(std::ostream& os,
                   std::size_t index,
                   const double* time,
                   const Eigen::VectorXd& solution,
                   const std::vector<ResolvedVariable>& variables){
    os << std::right << std::setw(kIndexWidth) << index;
    os << std::scientific << std::setprecision(10);
    if(time){
        ensureFinite(*time);
        os << ' ' << std::setw(kValueWidth - 1) << *time;
    }
    for(const auto& variable: variables){
        const double value = variableValue(variable, solution);
        ensureFinite(value);
        os << ' ' << std::setw(kValueWidth - 1) << value;
    }
    os << '\n';
}


void writeRawHeader(std::ostream& os,
                    const std::string& title,
                    const std::string& plotName,
                    std::size_t pointCount,
                    bool includeTime,
                    const std::vector<ResolvedVariable>& variables){
    const std::size_t variableCount =
        variables.size() + (includeTime ? 1 : 0);

    os << "Title: " << title << '\n';
    os << "Date: " << currentSpiceDate() << '\n';
    os << "Plotname: " << plotName << '\n';
    os << "Flags: real\n";
    os << "No. Variables: " << variableCount << '\n';
    os << "No. Points: " << pointCount << '\n';
    os << "Variables:\n";

    std::size_t variableIndex = 0;
    if(includeTime){
        os << '\t' << variableIndex++ << "\ttime\ttime\n";
    }
    for(const auto& variable: variables){
        os << '\t' << variableIndex++ << '\t'
           << variable.rawLabel << '\t' << variable.rawType << '\n';
    }
    os << "Values:\n";
}

void writeRawPoint(std::ostream& os,
                   std::size_t pointIndex,
                   const double* time,
                   const Eigen::VectorXd& solution,
                   const std::vector<ResolvedVariable>& variables){
    os << ' ' << pointIndex;
    std::size_t firstVariable = 0;

    if(time){
        ensureFinite(*time);
        os << '\t' << *time << '\n';
    } else if(!variables.empty()){
        const double value = variableValue(variables.front(), solution);
        ensureFinite(value);
        os << '\t' << value << '\n';
        firstVariable = 1;
    } else {
        os << '\n';
    }

    for(std::size_t i = firstVariable; i < variables.size(); ++i){
        const double value = variableValue(variables[i], solution);
        ensureFinite(value);
        os << '\t' << value << '\n';
    }
    os << '\n';
}
}

void SpiceOutputWriter::writeOperatingPoint(std::ostream& os,
                                             const Circuit& circuit,
                                             const std::string& title,
                                             const AnalysisPlan& plan){
    os.imbue(std::locale::classic());
    const auto variables = resolveVariables(
        circuit,
        plan.operatingPointPrintRequested,
        plan.operatingPointPrints
    );
    writeTableHeader(
        os,
        title,
        "Operating Point",
        1,
        false,
        variables
    );
    writeTableRow(
        os,
        0,
        nullptr,
        SpiceOutputAccess::mna(circuit).solution(),
        variables
    );
}

void SpiceOutputWriter::writeTransient(std::ostream& os,
                                        const Circuit& circuit,
                                        const std::string& title,
                                        const AnalysisPlan& plan){
    os.imbue(std::locale::classic());
    const auto variables = resolveVariables(
        circuit,
        plan.transientPrintRequested,
        plan.transientPrints
    );
    writeTableHeader(
        os,
        title,
        "Transient Analysis",
        SpiceOutputAccess::transientSamples(circuit).size(),
        true,
        variables
    );

    const auto& samples = SpiceOutputAccess::transientSamples(circuit);
    for(std::size_t i = 0; i < samples.size(); ++i){
        const auto& sample = samples[i];
        writeTableRow(os, i, &sample.time, sample.solution, variables);
    }
}

void SpiceOutputWriter::writePtaDiagnostics(std::ostream& os,
                                             const Circuit& circuit){
    os.imbue(std::locale::classic());

    const PtaDiagnostics diagnostics = circuit.ptaDiagnostics();
    os << "PTA diagnostics:\n";
    if(!diagnostics.attempted){
        os << "  status: PTA was not invoked\n";
        return;
    }

    os << "  converged: " << (diagnostics.converged ? "true" : "false")
       << "\n"
       << "  iterations: " << diagnostics.iterations << "\n"
       << "  capacitance_growths: " << diagnostics.capacitanceGrowths << "\n"
       << "  capacitance_reductions: " << diagnostics.capacitanceReductions << "\n"
       << "  minimum_step_recoveries: "
       << diagnostics.minimumStepRecoveries << "\n";

    if(diagnostics.hasConvergenceMetrics){
        os << std::scientific << std::setprecision(10)
           << "  normalized_derivative: "
           << diagnostics.normalizedDerivative << "\n"
           << "  normalized_dc_residual: "
           << diagnostics.normalizedDcResidual << "\n";
    }
}

void SpiceRawWriter::writeOperatingPoint(std::ostream& os,
                                          const Circuit& circuit,
                                          const std::string& title){
    os.imbue(std::locale::classic());
    os << std::scientific << std::setprecision(15);

    const auto variables = defaultVariables(circuit);
    writeRawHeader(os, title, "Operating Point", 1, false, variables);
    writeRawPoint(
        os,
        0,
        nullptr,
        SpiceOutputAccess::mna(circuit).solution(),
        variables
    );
}

void SpiceRawWriter::writeTransient(std::ostream& os,
                                     const Circuit& circuit,
                                     const std::string& title){
    os.imbue(std::locale::classic());
    os << std::scientific << std::setprecision(15);

    const auto variables = defaultVariables(circuit);
    const auto& samples = SpiceOutputAccess::transientSamples(circuit);
    writeRawHeader(
        os,
        title,
        "Transient Analysis",
        samples.size(),
        true,
        variables
    );
    for(std::size_t i = 0; i < samples.size(); ++i){
        writeRawPoint(
            os,
            i,
            &samples[i].time,
            samples[i].solution,
            variables
        );
    }
}

namespace {

struct StagedOutput {
    std::filesystem::path destination;
    std::filesystem::path temporary;
    std::filesystem::path backup;
    const char* description = nullptr;
    bool committed = false;
};

std::filesystem::path normalizedPath(const std::string& path){
    std::error_code error;
    std::filesystem::path normalized =
        std::filesystem::weakly_canonical(path, error);
    if(!error){
        return normalized;
    }

    error.clear();
    normalized = std::filesystem::absolute(path, error);
    return error
        ? std::filesystem::path(path).lexically_normal()
        : normalized.lexically_normal();
}

bool pathsReferToSameFile(const std::string& left,
                          const std::string& right){
    if(normalizedPath(left) == normalizedPath(right)){
        return true;
    }

    std::error_code error;
    const bool equivalent = std::filesystem::equivalent(left, right, error);
    return !error && equivalent;
}

std::filesystem::path temporaryOutputPath(
    const std::filesystem::path& destination)
{
    std::filesystem::path directory = destination.parent_path();
    if(directory.empty()){
        directory = ".";
    }

    static unsigned long long counter = 0;
    for(int attempt = 0; attempt < 100; ++attempt){
        const auto ticks = std::chrono::steady_clock::now()
            .time_since_epoch().count();
        const auto candidate = directory /
            (".spice-tmp-" + std::to_string(ticks) +
             "-" + std::to_string(++counter));
        std::error_code error;
        if(!std::filesystem::exists(candidate, error) && !error){
            return candidate;
        }
    }
    throw std::runtime_error(
        "Cannot allocate a temporary output beside <" +
        destination.string() + ">"
    );
}

void removeTemporaryOutput(const std::filesystem::path& path){
    if(path.empty()){
        return;
    }
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void discardStagedOutputs(std::vector<StagedOutput>& outputs){
    for(auto& output: outputs){
        removeTemporaryOutput(output.temporary);
        removeTemporaryOutput(output.backup);
        output.temporary.clear();
        output.backup.clear();
    }
}

bool stageFile(const std::string& path,
               std::string_view content,
               const char* description,
               StagedOutput& staged,
               std::ostream& error){
    staged.destination = path;
    staged.description = description;

    std::error_code statusError;
    if(std::filesystem::exists(staged.destination, statusError) &&
       !std::filesystem::is_regular_file(staged.destination, statusError)){
        error << "Cannot replace non-regular " << description << " <"
              << path << ">\n";
        return false;
    }
    if(statusError){
        error << "Cannot inspect " << description << " <" << path
              << ">: " << statusError.message() << '\n';
        return false;
    }

    try {
        staged.temporary = temporaryOutputPath(staged.destination);
    } catch(const std::exception& exception){
        error << exception.what() << '\n';
        return false;
    }

    std::ofstream output(
        staged.temporary,
        std::ios::out | std::ios::trunc | std::ios::binary
    );
    if(!output){
        error << "Cannot open " << description << " <" << path << ">\n";
        removeTemporaryOutput(staged.temporary);
        staged.temporary.clear();
        return false;
    }

    output.write(content.data(), static_cast<std::streamsize>(content.size()));
    output.flush();
    output.close();
    if(!output){
        error << "Failed while writing " << description
              << " <" << path << ">\n";
        removeTemporaryOutput(staged.temporary);
        staged.temporary.clear();
        return false;
    }
    return true;
}

bool commitStagedOutputs(std::vector<StagedOutput>& outputs,
                         std::ostream& error){
    const auto rollback = [&outputs, &error](){
        bool restored = true;
        for(auto& output: outputs){
            if(output.committed){
                std::error_code removeError;
                std::filesystem::remove(output.destination, removeError);
                if(removeError){
                    error << "Cannot remove partially committed "
                          << output.description << " <"
                          << output.destination.string() << ">: "
                          << removeError.message() << '\n';
                    restored = false;
                }
                output.committed = false;
            }
        }
        for(auto& output: outputs){
            if(output.backup.empty()){
                continue;
            }
            std::error_code restoreError;
            std::filesystem::rename(
                output.backup,
                output.destination,
                restoreError
            );
            if(restoreError){
                error << "Cannot restore previous " << output.description
                      << " <" << output.destination.string() << ">: "
                      << restoreError.message() << '\n';
                restored = false;
            } else {
                output.backup.clear();
            }
        }
        for(auto& output: outputs){
            removeTemporaryOutput(output.temporary);
            output.temporary.clear();
        }
        return restored;
    };

    for(auto& output: outputs){
        std::error_code existsError;
        const bool exists = std::filesystem::exists(
            output.destination,
            existsError
        );
        if(existsError){
            error << "Cannot inspect " << output.description << " <"
                  << output.destination.string() << ">: "
                  << existsError.message() << '\n';
            rollback();
            return false;
        }
        if(!exists){
            continue;
        }

        std::error_code typeError;
        if(!std::filesystem::is_regular_file(output.destination, typeError) ||
           typeError){
            error << "Cannot replace non-regular " << output.description
                  << " <" << output.destination.string() << ">\n";
            rollback();
            return false;
        }

        try {
            output.backup = temporaryOutputPath(output.destination);
        } catch(const std::exception& exception){
            error << exception.what() << '\n';
            rollback();
            return false;
        }

        std::error_code backupError;
        std::filesystem::rename(
            output.destination,
            output.backup,
            backupError
        );
        if(backupError){
            error << "Cannot preserve previous " << output.description
                  << " <" << output.destination.string() << ">: "
                  << backupError.message() << '\n';
            output.backup.clear();
            rollback();
            return false;
        }
    }

    for(auto& output: outputs){
        std::error_code collisionError;
        if(std::filesystem::exists(output.destination, collisionError)){
            error << "Output destination appeared during commit <"
                  << output.destination.string()
                  << ">; possible path alias or concurrent writer\n";
            rollback();
            return false;
        }
        if(collisionError){
            error << "Cannot inspect output destination <"
                  << output.destination.string() << ">: "
                  << collisionError.message() << '\n';
            rollback();
            return false;
        }

        std::error_code commitError;
        std::filesystem::rename(
            output.temporary,
            output.destination,
            commitError
        );
        if(commitError){
            error << "Cannot replace " << output.description << " <"
                  << output.destination.string() << ">: "
                  << commitError.message() << '\n';
            rollback();
            return false;
        }
        output.temporary.clear();
        output.committed = true;
    }

    for(auto& output: outputs){
        removeTemporaryOutput(output.backup);
        output.backup.clear();
        output.committed = false;
    }
    return true;
}

}  // namespace

bool SpiceOutputFiles::validatePaths(
    const std::string& inputPath,
    const std::optional<std::string>& listingPath,
    const std::optional<std::string>& rawPath,
    std::ostream& error
){
    if(listingPath && pathsReferToSameFile(*listingPath, inputPath)){
        error << "Input netlist and listing output must be different files\n";
        return false;
    }
    if(rawPath && pathsReferToSameFile(*rawPath, inputPath)){
        error << "Input netlist and raw output must be different files\n";
        return false;
    }
    if(listingPath && rawPath &&
       pathsReferToSameFile(*listingPath, *rawPath)){
        error << "Listing output and raw output must be different files\n";
        return false;
    }
    return true;
}

bool SpiceOutputFiles::writeAtomically(
    const std::optional<std::string>& listingPath,
    std::string_view listing,
    const std::optional<std::string>& rawPath,
    std::string_view raw,
    std::ostream& error
){
    std::vector<StagedOutput> stagedOutputs;
    stagedOutputs.reserve(
        static_cast<std::size_t>(listingPath.has_value()) +
        static_cast<std::size_t>(rawPath.has_value())
    );

    if(listingPath){
        StagedOutput staged;
        if(!stageFile(
               *listingPath,
               listing,
               "listing output",
               staged,
               error)){
            discardStagedOutputs(stagedOutputs);
            return false;
        }
        stagedOutputs.push_back(std::move(staged));
    }
    if(rawPath){
        StagedOutput staged;
        if(!stageFile(
               *rawPath,
               raw,
               "raw output",
               staged,
               error)){
            discardStagedOutputs(stagedOutputs);
            return false;
        }
        stagedOutputs.push_back(std::move(staged));
    }
    return commitStagedOutputs(stagedOutputs, error);
}
