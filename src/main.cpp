#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "app/command_line.h"
#include "config/config_loader.h"
#include "config/overrides.h"
#include "circuit/circuit.h"
#include "io/spice_output.h"
#include "netlist/parser.h"

int main(int argc, char* argv[]){
    simulator::app::CommandLineOptions options;
    if(!simulator::app::parseCommandLine(
           argc,
           argv,
           options,
           std::cout,
           std::cerr)){
        if(argc <= 1){
            simulator::app::printUsage(std::cerr, argv[0]);
        }
        return options.helpRequested ? 0 : 2;
    }
    if(!SpiceOutputFiles::validatePaths(
           options.inputPath,
           options.listingPath,
           options.rawPath,
           std::cerr)){
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
    const auto netlistLocks = simulator::config::parameterLocksFor(plan);

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

        if((!options.ptaOptionAssignments.empty() || options.ptaDiagnostics) &&
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
    } catch(const std::exception& exception){
        std::cerr << "Output error: " << exception.what() << '\n';
        return 1;
    }

    const std::string listingOutput = listing.str();
    const std::string rawOutput = raw.str();
    if(!SpiceOutputFiles::writeAtomically(
           options.listingPath,
           listingOutput,
           options.rawPath,
           rawOutput,
           std::cerr)){
        return 1;
    }

    if(!options.listingPath){
        std::cout << listingOutput;
        std::cout.flush();
        if(!std::cout){
            std::cerr << "Failed while writing listing output to stdout\n";
            return 1;
        }
    }

    return 0;
}
