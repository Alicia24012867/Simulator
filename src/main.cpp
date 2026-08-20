#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "analysis/ptaAnalysis.h"
#include "analysis/solverOptions.h"
#include "config/applyOverrides.h"
#include "config/commandLineOverrides.h"
#include "config/config.h"
#include "config/overrides.h"
#include "circuit/circuit.h"
#include "io/spiceOutput.h"
#include "netlist/parser.h"

namespace {
struct CommandLineOptions {
    std::string inputPath;
    std::optional<std::string> listingPath;
    std::optional<std::string> rawPath;
    std::optional<std::filesystem::path> configPath;
    std::size_t configSearchDepth = 8;
    bool configSearchDepthSpecified = false;
    bool printConfigPath = false;
    PtaMode ptaMode = PtaMode::Disabled;
    bool ptaModeSpecified = false;
    bool ptaDiagnostics = false;
    std::set<std::string> operatingPointOptionKeys;
    std::vector<std::string> operatingPointOptionAssignments;
    std::set<std::string> ptaOptionKeys;
    std::vector<std::string> ptaOptionAssignments;
    std::set<std::string> transientOptionKeys;
    std::vector<std::string> transientOptionAssignments;
    bool parseOnly = false;
    bool helpRequested = false;
};

struct StagedOutput {
    std::filesystem::path destination;
    std::filesystem::path temporary;
    std::filesystem::path backup;
    const char* description = nullptr;
    bool committed = false;
};

void printUsage(std::ostream& os, const char* program){
    os << "Usage:\n"
       << "  " << program << " <input.cir> [output.out]\n"
       << "  " << program << " --parse-only <input.cir>\n"
       << "  " << program
       << " [-b] [--pta disabled|force|fallback]"
       << " [--pta-diagnostics]"
       << " [--op-option name=value]"
       << " [--pta-option name=value]"
       << " [--tran-option name=value]"
       << " [--config path] [--config-search-depth N]"
       << " [--print-config-path]"
       << " [-o output.out] [-r output.raw] <input.cir>\n";
    os << "\nConfiguration options:\n"
       << "  --config path              Read this configuration file only\n"
       << "  --config-search-depth N    Search at most N parent directories\n"
       << "  --print-config-path         Report the selected configuration file\n";
    os << "\nAnalysis overrides (repeat as needed; netlist controls win):\n"
       << "  --op-option name=value    OP fields: newton.* or source-stepping.*\n"
       << "  --tran-option name=value  TRAN fields: top-level or solver.*\n"
       << "  --pta-option name=value   PTA fields below; --pta selects its mode\n";
    os << "\nPTA option fields:\n"
       << "  initial-step, minimum-step, maximum-step, maximum-steps\n"
       << "  derivative-tolerance, derivative-relative-tolerance,\n"
       << "  derivative-voltage-absolute-tolerance,\n"
       << "  derivative-current-absolute-tolerance, dc-residual-tolerance\n"
       << "  dc-residual-relative-tolerance, dc-voltage-absolute-tolerance,\n"
       << "  dc-current-absolute-tolerance\n"
       << "  initial-node-capacitance, minimum-node-capacitance,\n"
       << "  maximum-node-capacitance, current-source-capacitance,\n"
       << "  voltage-source-inductance\n"
       << "  failed-step-scale, successful-step-scale,\n"
       << "  capacitance-grow-scale,\n"
       << "  small-oscillation-scale, medium-oscillation-scale,\n"
       << "  heavy-oscillation-scale, medium-oscillation-ratio,\n"
       << "  heavy-oscillation-ratio, compound-time-constant,\n"
       << "  compound-initial-resistance, compound-initial-conductance,\n"
       << "  source-ramp-time, initial-bjt-vbe (or null)\n"
       << "  include-mos-bulk, include-diodes (true or false)\n";
}

bool parseNonNegativeSize(const std::string& text, std::size_t& value){
    if(text.empty()){
        return false;
    }

    for(unsigned char character: text){
        if(!std::isdigit(character)){
            return false;
        }
    }

    try {
        std::size_t parsedLength = 0;
        const unsigned long long parsed =
            std::stoull(text, &parsedLength, 10);
        if(parsedLength != text.size() ||
           parsed > std::numeric_limits<std::size_t>::max()){
            return false;
        }
        value = static_cast<std::size_t>(parsed);
        return true;
    } catch(const std::exception&) {
        return false;
    }
}

bool parsePtaMode(const std::string& text, PtaMode& mode){
    if(text == "disabled"){
        mode = PtaMode::Disabled;
        return true;
    }
    if(text == "force"){
        mode = PtaMode::Force;
        return true;
    }
    if(text == "fallback"){
        mode = PtaMode::Fallback;
        return true;
    }
    return false;
}

simulator::config::NetlistAnalysisParameterLocks
netlistParameterLocks(const AnalysisPlan& plan){
    simulator::config::NetlistAnalysisParameterLocks locks;

    if(plan.transient){
        locks.transient.enabled = true;
        locks.transient.outputInterval = true;
        locks.transient.stopTime = true;

        if(plan.transientNetlistParameters){
            const auto& presence = *plan.transientNetlistParameters;
            locks.transient.outputStartTime = presence.outputStartTime;
            locks.transient.maximumStep = presence.maximumStep;
            locks.transient.useInitialConditions =
                presence.useInitialConditions;
        }
        if(plan.delmax){
            locks.transient.maximumStep = true;
        }
    }

    if(plan.pseudoTransient){
        const auto& pstran = *plan.pseudoTransient;
        locks.pta.mode = true;
        locks.pta.initialStep = pstran.initialStepSpecified;
        locks.pta.minimumStep = pstran.minimumStepSpecified;
        locks.pta.maximumStep = pstran.maximumStepSpecified ||
            plan.delmax.has_value();
        locks.pta.derivativeTolerance = pstran.convergenceValueSpecified;
        locks.pta.dcResidualTolerance = pstran.convergenceValueSpecified;
        locks.pta.compoundTimeConstant = pstran.tauSpecified;
        locks.pta.sourceRampTime = pstran.tauRampSpecified;
        locks.pta.initialBjtVbe = pstran.vbe0Specified;
    }

    return locks;
}

bool parseCommandLine(int argc,
                      char* argv[],
                      CommandLineOptions& options){
    std::vector<std::string> positional;

    for(int i = 1; i < argc; ++i){
        const std::string argument = argv[i];
        if(argument == "-h" || argument == "--help"){
            printUsage(std::cout, argv[0]);
            options.helpRequested = true;
            return false;
        }
        if(argument == "-b" || argument == "--batch"){
            continue;
        }
        if(argument == "--config"){
            if(++i >= argc || options.configPath ||
               std::string(argv[i]).empty() || argv[i][0] == '-'){
                std::cerr << "Invalid or repeated configuration file option\n";
                return false;
            }
            options.configPath = std::filesystem::path(argv[i]);
            continue;
        }
        if(argument == "--config-search-depth"){
            if(++i >= argc || options.configSearchDepthSpecified ||
               !parseNonNegativeSize(argv[i], options.configSearchDepth)){
                std::cerr
                    << "Invalid or repeated configuration search depth; "
                    << "expected a non-negative integer\n";
                return false;
            }
            options.configSearchDepthSpecified = true;
            continue;
        }
        if(argument == "--print-config-path"){
            if(options.printConfigPath){
                std::cerr << "Repeated print-config-path option\n";
                return false;
            }
            options.printConfigPath = true;
            continue;
        }
        if(argument == "--parse-only"){
            if(options.parseOnly){
                std::cerr << "Repeated parse-only option\n";
                return false;
            }
            options.parseOnly = true;
            continue;
        }
        if(argument == "--pta-diagnostics"){
            if(options.ptaDiagnostics){
                std::cerr << "Repeated PTA diagnostics option\n";
                return false;
            }
            options.ptaDiagnostics = true;
            continue;
        }
        if(argument == "--pta"){
            if(++i >= argc || options.ptaModeSpecified ||
               !parsePtaMode(argv[i], options.ptaMode)){
                std::cerr
                    << "Invalid or repeated PTA mode; expected "
                    << "disabled, force, or fallback\n";
                return false;
            }
            options.ptaModeSpecified = true;
            continue;
        }
        if(argument == "--op-option"){
            if(++i >= argc){
                std::cerr << "Missing OP option; expected name=value\n";
                return false;
            }

            OperatingPointSolverOptions validationOptions;
            std::string key;
            std::string error;
            if(!simulator::config::applyOperatingPointOption(
                   argv[i], validationOptions, key, error)){
                std::cerr << "Invalid OP option <" << argv[i]
                          << ">: " << error << '\n';
                return false;
            }
            if(!options.operatingPointOptionKeys.insert(key).second){
                std::cerr << "Repeated OP option: " << key << '\n';
                return false;
            }
            options.operatingPointOptionAssignments.emplace_back(argv[i]);
            continue;
        }
        if(argument == "--pta-option"){
            if(++i >= argc){
                std::cerr << "Missing PTA option; expected name=value\n";
                return false;
            }

            std::string key;
            std::string error;
            PtaAnalysisConfig validationOptions;
            if(!simulator::config::applyPtaOption(
                   argv[i], validationOptions, key, error)){
                std::cerr << "Invalid PTA option <" << argv[i]
                          << ">: " << error << '\n';
                return false;
            }
            if(!options.ptaOptionKeys.insert(key).second){
                std::cerr << "Repeated PTA option: " << key << '\n';
                return false;
            }
            options.ptaOptionAssignments.emplace_back(argv[i]);
            continue;
        }
        if(argument == "--tran-option"){
            if(++i >= argc){
                std::cerr << "Missing TRAN option; expected name=value\n";
                return false;
            }

            std::optional<TransientAnalysisConfig> validationOptions;
            std::string key;
            std::string error;
            if(!simulator::config::applyTransientOption(
                   argv[i], validationOptions, key, error)){
                std::cerr << "Invalid TRAN option <" << argv[i]
                          << ">: " << error << '\n';
                return false;
            }
            if(!options.transientOptionKeys.insert(key).second){
                std::cerr << "Repeated TRAN option: " << key << '\n';
                return false;
            }
            options.transientOptionAssignments.emplace_back(argv[i]);
            continue;
        }
        if(argument == "-o" || argument == "--output"){
            if(++i >= argc || options.listingPath ||
               std::string(argv[i]).empty() || argv[i][0] == '-'){
                std::cerr << "Invalid or repeated listing output option\n";
                return false;
            }
            options.listingPath = argv[i];
            continue;
        }
        if(argument == "-r" || argument == "--rawfile"){
            if(++i >= argc || options.rawPath ||
               std::string(argv[i]).empty() || argv[i][0] == '-'){
                std::cerr << "Invalid or repeated rawfile option\n";
                return false;
            }
            options.rawPath = argv[i];
            continue;
        }
        if(!argument.empty() && argument[0] == '-'){
            std::cerr << "Unknown option: " << argument << '\n';
            return false;
        }
        positional.push_back(argument);
    }

    if(positional.empty() || positional.size() > 2){
        std::cerr << "Exactly one input netlist is required\n";
        return false;
    }

    options.inputPath = positional[0];
    if(positional.size() == 2){
        if(options.listingPath){
            std::cerr << "Listing output was specified twice\n";
            return false;
        }
        options.listingPath = positional[1];
    }
    if(options.parseOnly &&
       (options.listingPath || options.rawPath || options.ptaModeSpecified ||
        options.ptaDiagnostics ||
        !options.operatingPointOptionKeys.empty() ||
        !options.ptaOptionKeys.empty() ||
        !options.transientOptionKeys.empty())){
        std::cerr
            << "--parse-only cannot be combined with output or analysis options\n";
        return false;
    }
    return true;
}

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

bool validateOutputPaths(const CommandLineOptions& options){
    if(options.listingPath &&
       pathsReferToSameFile(*options.listingPath, options.inputPath)){
        std::cerr << "Input netlist and listing output must be different files\n";
        return false;
    }
    if(options.rawPath &&
       pathsReferToSameFile(*options.rawPath, options.inputPath)){
        std::cerr << "Input netlist and raw output must be different files\n";
        return false;
    }
    if(options.listingPath && options.rawPath &&
       pathsReferToSameFile(*options.listingPath, *options.rawPath)){
        std::cerr << "Listing output and raw output must be different files\n";
        return false;
    }
    return true;
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
               const std::string& content,
               const char* description,
               StagedOutput& staged){
    staged.destination = path;
    staged.description = description;

    std::error_code statusError;
    if(std::filesystem::exists(staged.destination, statusError) &&
       !std::filesystem::is_regular_file(staged.destination, statusError)){
        std::cerr << "Cannot replace non-regular " << description << " <"
                  << path << ">\n";
        return false;
    }
    if(statusError){
        std::cerr << "Cannot inspect " << description << " <" << path
                  << ">: " << statusError.message() << '\n';
        return false;
    }

    try {
        staged.temporary = temporaryOutputPath(staged.destination);
    } catch(const std::exception& ex){
        std::cerr << ex.what() << '\n';
        return false;
    }

    std::ofstream output(
        staged.temporary,
        std::ios::out | std::ios::trunc | std::ios::binary
    );
    if(!output){
        std::cerr << "Cannot open " << description << " <" << path << ">\n";
        removeTemporaryOutput(staged.temporary);
        staged.temporary.clear();
        return false;
    }

    output.write(content.data(), static_cast<std::streamsize>(content.size()));
    output.flush();
    output.close();
    if(!output){
        std::cerr << "Failed while writing " << description
                  << " <" << path << ">\n";
        removeTemporaryOutput(staged.temporary);
        staged.temporary.clear();
        return false;
    }
    return true;
}

bool commitStagedOutputs(std::vector<StagedOutput>& outputs){
    auto rollback = [&outputs](){
        bool restored = true;
        for(auto& output: outputs){
            if(output.committed){
                std::error_code removeError;
                std::filesystem::remove(output.destination, removeError);
                if(removeError){
                    std::cerr << "Cannot remove partially committed "
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
                std::cerr << "Cannot restore previous " << output.description
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
            std::cerr << "Cannot inspect " << output.description << " <"
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
            std::cerr << "Cannot replace non-regular " << output.description
                      << " <" << output.destination.string() << ">\n";
            rollback();
            return false;
        }

        try {
            output.backup = temporaryOutputPath(output.destination);
        } catch(const std::exception& ex){
            std::cerr << ex.what() << '\n';
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
            std::cerr << "Cannot preserve previous " << output.description
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
            std::cerr << "Output destination appeared during commit <"
                      << output.destination.string()
                      << ">; possible path alias or concurrent writer\n";
            rollback();
            return false;
        }
        if(collisionError){
            std::cerr << "Cannot inspect output destination <"
                      << output.destination.string() << ">: "
                      << collisionError.message() << '\n';
            rollback();
            return false;
        }

        std::error_code error;
        std::filesystem::rename(
            output.temporary,
            output.destination,
            error
        );
        if(error){
            std::cerr << "Cannot replace " << output.description << " <"
                      << output.destination.string() << ">: "
                      << error.message() << '\n';
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
}

int main(int argc, char* argv[]){
    CommandLineOptions options;
    if(!parseCommandLine(argc, argv, options)){
        if(argc <= 1){
            printUsage(std::cerr, argv[0]);
        }
        return options.helpRequested ? 0 : 2;
    }
    if(!validateOutputPaths(options)){
        return 2;
    }

    simulator::config::ConfigSearchOptions configOptions;
    configOptions.explicitPath = options.configPath;
    configOptions.parentSearchLimit = options.configSearchDepth;

    simulator::config::LoadedConfig loadedConfig;
    simulator::config::ConfigOverrides configOverrides;

    try {
        loadedConfig = simulator::config::loadConfig(configOptions);
        configOverrides =
            simulator::config::parseConfigOverrides(loadedConfig);
    } catch(const std::runtime_error& error) {
        std::cerr << "Invalid configuration: " << error.what() << '\n';
        return 2;
    }

    if(options.printConfigPath){
        if(loadedConfig.found){
            std::cerr << "Configuration file: <"
                      << loadedConfig.path.string() << ">\n";
        } else {
            std::cerr
                << "Configuration file: none (using built-in defaults)\n";
        }
    }

    Parser parser(options.inputPath);
    Circuit circuit;

    if(!parser.parse(circuit)){
        return 1;
    }

    AnalysisPlan plan = parser.analysisPlan();
    const auto netlistLocks = netlistParameterLocks(plan);

    const std::optional<TransientAnalysisConfig> netlistTransient =
        plan.transient;
    const std::optional<double> netlistMaximumTransientStep =
        netlistTransient ? netlistTransient->maximumStep : std::nullopt;

    PtaAnalysisConfig ptaConfig = plan.pseudoTransient
        ? plan.pseudoTransient->makePtaConfig()
        : PtaAnalysisConfig{};
    OperatingPointSolverOptions operatingPointConfig;

    try {
        simulator::config::applyConfigOverrides(
            configOverrides,
            operatingPointConfig,
            ptaConfig,
            plan.transient,
            plan.pseudoTransient.has_value(),
            netlistLocks
        );

        for(const std::string& assignment:
            options.operatingPointOptionAssignments)
        {
            std::string key;
            std::string error;
            if(!simulator::config::applyOperatingPointOption(
                   assignment,
                   operatingPointConfig,
                   key,
                   error)){
                throw std::invalid_argument(
                    "invalid command-line OP option <" + assignment +
                    ">: " + error
                );
            }
        }

        if(options.ptaModeSpecified && !netlistLocks.pta.mode){
            ptaConfig.mode = options.ptaMode;
        }
        for(const std::string& assignment: options.ptaOptionAssignments){
            std::string key;
            std::string error;
            if(!simulator::config::applyPtaOption(
                   assignment,
                   ptaConfig,
                   key,
                   error,
                   netlistLocks.pta)){
                throw std::invalid_argument(
                    "invalid command-line PTA option <" + assignment +
                    ">: " + error
                );
            }
        }
        for(const std::string& assignment: options.transientOptionAssignments){
            std::string key;
            std::string error;
            if(!simulator::config::applyTransientOption(
                   assignment,
                   plan.transient,
                   key,
                   error,
                   netlistTransient,
                   netlistMaximumTransientStep,
                   netlistLocks.transient)){
                throw std::invalid_argument(
                    "invalid command-line TRAN option <" + assignment +
                    ">: " + error
                );
            }
        }

        if((!options.ptaOptionKeys.empty() || options.ptaDiagnostics) &&
           ptaConfig.mode == PtaMode::Disabled){
            throw std::invalid_argument(
                "PTA options and diagnostics require force or fallback mode"
            );
        }

        if(!operatingPointConfig.valid()){
            throw std::invalid_argument(
                "operating-point solver configuration is invalid"
            );
        }
        if(plan.transient && !plan.transient->valid()){
            throw std::invalid_argument(
                "transient solver configuration is invalid"
            );
        }
    } catch(const std::invalid_argument& error) {
        std::cerr << "Invalid analysis configuration: " << error.what() << '\n';
        return 2;
    }

    try {
        ptaConfig.validate();
    } catch(const std::invalid_argument& error) {
        std::cerr << "Invalid PTA configuration: " << error.what() << '\n';
        return 2;
    }

    if(options.parseOnly){
        return 0;
    }

    if(!circuit.build(ptaConfig)){
        std::cerr << "Failed to build circuit <" << options.inputPath << ">\n";
        return 1;
    }

    std::ostringstream listing;
    std::ostringstream raw;
    bool wroteAnalysis = false;
    bool wrotePtaDiagnostics = false;

    try {
        if(plan.operatingPointRequested || !plan.transient){
            bool solved = false;

            switch (ptaConfig.mode){
                case PtaMode::Disabled:
                    solved = circuit.solveOperatingPoint(operatingPointConfig);
                    break;
                case PtaMode::Force:
                    solved = circuit.solveAdaptivePta(ptaConfig);
                    break;
                case PtaMode::Fallback:
                    solved = circuit.solveOperatingPoint(operatingPointConfig);
                    if(!solved){
                        solved = circuit.solveAdaptivePta(ptaConfig);
                    }
                    break;
            }

            if(!solved){
                std::cerr << "Operating point analysis failed <"
                        << options.inputPath << ">\n";
                return 1;
            }

            if(options.ptaDiagnostics){
                SpiceOutputWriter::writePtaDiagnostics(std::cerr, circuit);
                wrotePtaDiagnostics = true;
            }

            SpiceOutputWriter::writeOperatingPoint(
                listing,
                circuit,
                parser.title(),
                plan
            );
            if(options.rawPath){
                SpiceRawWriter::writeOperatingPoint(
                    raw,
                    circuit,
                    parser.title()
                );
            }
            wroteAnalysis = true;
        }

        if(plan.transient){
            if(!circuit.solveTransient(*plan.transient, operatingPointConfig)){
                std::cerr << "Transient analysis failed <"
                          << options.inputPath << ">\n";
                return 1;
            }
            if(wroteAnalysis){
                listing << '\n';
                if(options.rawPath){
                    raw << '\n';
                }
            }
            SpiceOutputWriter::writeTransient(
                listing,
                circuit,
                parser.title(),
                plan
            );
            if(options.rawPath){
                SpiceRawWriter::writeTransient(
                    raw,
                    circuit,
                    parser.title()
                );
            }
        }

        if(options.ptaDiagnostics && !wrotePtaDiagnostics){
            SpiceOutputWriter::writePtaDiagnostics(std::cerr, circuit);
        }
    } catch(const std::exception& ex){
        std::cerr << "Output error: " << ex.what() << '\n';
        return 1;
    }

    std::vector<StagedOutput> stagedOutputs;
    if(options.listingPath){
        StagedOutput staged;
        if(!stageFile(
               *options.listingPath,
               listing.str(),
               "listing output",
               staged)){
            discardStagedOutputs(stagedOutputs);
            return 1;
        }
        stagedOutputs.push_back(std::move(staged));
    }
    if(options.rawPath){
        StagedOutput staged;
        if(!stageFile(
               *options.rawPath,
               raw.str(),
               "raw output",
               staged)){
            discardStagedOutputs(stagedOutputs);
            return 1;
        }
        stagedOutputs.push_back(std::move(staged));
    }
    if(!commitStagedOutputs(stagedOutputs)){
        return 1;
    }

    if(!options.listingPath){
        std::cout << listing.str();
        std::cout.flush();
        if(!std::cout){
            std::cerr << "Failed while writing listing output to stdout\n";
            return 1;
        }
    }

    return 0;
}
