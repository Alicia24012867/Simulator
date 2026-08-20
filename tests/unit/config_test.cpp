#include "config/applyOverrides.h"
#include "config/config.h"
#include "config/overrides.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int checkCount = 0;
int failureCount = 0;

void expect(bool condition, const std::string& description){
    ++checkCount;
    if(condition){
        return;
    }

    ++failureCount;
    std::cerr << "FAIL: " << description << '\n';
}

template<class Callback>
void expectRuntimeError(Callback&& callback, const std::string& description){
    bool threw = false;
    try {
        callback();
    } catch(const std::runtime_error&) {
        threw = true;
    } catch(...) {
    }
    expect(threw, description);
}

template<class Callback>
void expectInvalidArgument(Callback&& callback,
                           const std::string& description){
    bool threw = false;
    try {
        callback();
    } catch(const std::invalid_argument&) {
        threw = true;
    } catch(...) {
    }
    expect(threw, description);
}

class TemporaryDirectory {
public:
    TemporaryDirectory(){
        const std::filesystem::path base =
            std::filesystem::temp_directory_path();

        static unsigned long long counter = 0;
        for(int attempt = 0; attempt < 100; ++attempt){
            const auto ticks = std::chrono::steady_clock::now()
                .time_since_epoch().count();
            const std::filesystem::path candidate = base /
                ("simulator-config-test-" + std::to_string(ticks) +
                 "-" + std::to_string(++counter));
            std::error_code error;
            if(std::filesystem::create_directory(candidate, error)){
                path_ = candidate;
                return;
            }
            if(error){
                throw std::runtime_error(
                    "Cannot create temporary test directory: " +
                    error.message()
                );
            }
        }

        throw std::runtime_error("Cannot allocate temporary test directory");
    }

    ~TemporaryDirectory(){
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    const std::filesystem::path& path() const{
        return path_;
    }

private:
    std::filesystem::path path_;
};

void createDirectory(const std::filesystem::path& path){
    std::error_code error;
    if(!std::filesystem::create_directories(path, error) && error){
        throw std::runtime_error(
            "Cannot create test directory <" + path.string() + ">: " +
            error.message()
        );
    }
}

void writeFile(const std::filesystem::path& path, const std::string& contents){
    std::ofstream output(path);
    if(!output){
        throw std::runtime_error(
            "Cannot create test file <" + path.string() + ">"
        );
    }
    output << contents;
    if(!output){
        throw std::runtime_error(
            "Cannot write test file <" + path.string() + ">"
        );
    }
}

simulator::config::LoadedConfig loadedConfigFor(
    const nlohmann::json& document
){
    simulator::config::LoadedConfig loaded;
    loaded.found = true;
    loaded.path = "unit-test-config.json";
    loaded.document = document;
    return loaded;
}

void testAutomaticDiscovery(){
    TemporaryDirectory temporary;
    const std::filesystem::path project = temporary.path() / "project";
    const std::filesystem::path nested = project / "nested";
    createDirectory(nested);
    writeFile(
        project / simulator::config::kDefaultConfigFilename,
        R"({"scope":"project"})"
    );
    writeFile(
        temporary.path() / simulator::config::kDefaultConfigFilename,
        R"({"scope":"parent"})"
    );

    simulator::config::ConfigSearchOptions options;
    const simulator::config::LoadedConfig loaded =
        simulator::config::loadConfig(options, nested);

    expect(loaded.found, "automatic search finds a parent configuration");
    expect(
        loaded.path ==
            project / simulator::config::kDefaultConfigFilename,
        "nearest configuration file wins"
    );
    expect(
        loaded.document.at("scope") == "project",
        "loaded document preserves json values"
    );

    options.parentSearchLimit = 1;
    const auto candidates =
        simulator::config::configSearchCandidates(options, nested);
    expect(
        candidates.size() == 2,
        "search depth includes the working directory and one parent"
    );
    expect(
        candidates.front().parent_path() == nested,
        "first search candidate is in the working directory"
    );
}

void testSearchLimitAndDefaultFallback(){
    TemporaryDirectory temporary;
    const std::filesystem::path nested = temporary.path() / "nested";
    createDirectory(nested);
    writeFile(
        temporary.path() / simulator::config::kDefaultConfigFilename,
        R"({"scope":"parent"})"
    );

    simulator::config::ConfigSearchOptions options;
    options.parentSearchLimit = 0;
    const simulator::config::LoadedConfig loaded =
        simulator::config::loadConfig(options, nested);

    expect(!loaded.found, "zero parent searches does not inspect the parent");
    expect(loaded.path.empty(), "missing automatic configuration has no path");
    expect(
        loaded.document.is_object() && loaded.document.empty(),
        "missing automatic configuration returns an empty json object"
    );
}

void testExplicitConfiguration(){
    TemporaryDirectory temporary;
    const std::filesystem::path configPath = temporary.path() / "custom.json";
    writeFile(configPath, R"({"enabled":true})");

    simulator::config::ConfigSearchOptions options;
    options.explicitPath = configPath;
    const simulator::config::LoadedConfig loaded =
        simulator::config::loadConfig(options, temporary.path());

    expect(loaded.found, "explicit configuration is loaded");
    expect(
        loaded.path == configPath,
        "explicit configuration reports its absolute path"
    );
    expect(
        loaded.document.at("enabled") == true,
        "explicit configuration preserves json boolean values"
    );

    options.explicitPath = temporary.path() / "missing.json";
    expectRuntimeError(
        [&] {
            static_cast<void>(
                simulator::config::loadConfig(options, temporary.path())
            );
        },
        "missing explicit configuration is an error"
    );
}

void testInvalidConfigurationFiles(){
    TemporaryDirectory temporary;
    simulator::config::ConfigSearchOptions options;

    const std::filesystem::path invalidjson =
        temporary.path() / "invalid.json";
    writeFile(invalidjson, "{");
    options.explicitPath = invalidjson;
    expectRuntimeError(
        [&] {
            static_cast<void>(
                simulator::config::loadConfig(options, temporary.path())
            );
        },
        "invalid json is rejected"
    );

    const std::filesystem::path arrayRoot =
        temporary.path() / "array.json";
    writeFile(arrayRoot, "[]");
    options.explicitPath = arrayRoot;
    expectRuntimeError(
        [&] {
            static_cast<void>(
                simulator::config::loadConfig(options, temporary.path())
            );
        },
        "non-object json root is rejected"
    );

    const std::filesystem::path directoryPath =
        temporary.path() / "directory.json";
    createDirectory(directoryPath);
    options.explicitPath = directoryPath;
    expectRuntimeError(
        [&] {
            static_cast<void>(
                simulator::config::loadConfig(options, temporary.path())
            );
        },
        "configuration path must be a regular file"
    );
}

void testConfigOverrideParsing(){
    const simulator::config::ConfigOverrides missing =
        simulator::config::parseConfigOverrides({});
    expect(
        !missing.schemaVersion && !missing.pta && !missing.transient,
        "missing configuration produces no overrides"
    );

    const auto overrides = simulator::config::parseConfigOverrides(
        loadedConfigFor(nlohmann::json::parse(R"(
            {
                "schema_version": 1,
                "op": {
                    "newton": {
                        "maximum_iterations": 41,
                        "tolerance": "2n",
                        "maximum_solution_step": 0.25
                    },
                    "source_stepping": {
                        "enabled": false,
                        "initial_step": 0.05,
                        "maximum_step": 0.2,
                        "minimum_step": "10u",
                        "growth_factor": 1.25,
                        "failure_scale": 0.4
                    }
                },
                "pta": {
                    "mode": "FoRcE",
                    "newton": {"maximum_iterations": 53},
                    "initial_step": "2.5n",
                    "maximum_steps": 7,
                    "initial_bjt_vbe": 0.72,
                    "include_diodes": false
                },
                "tran": {
                    "enabled": true,
                    "output_interval": 2e-9,
                    "stop_time": "10n",
                    "use_initial_conditions": false,
                    "solver": {
                        "newton": {"tolerance": "3n"},
                        "relative_tolerance": 0.001,
                        "maximum_rejects": 0
                    }
                }
            }
        )"))
    );

    expect(
        overrides.schemaVersion && *overrides.schemaVersion == 1,
        "schema version is retained"
    );
    expect(
        overrides.operatingPoint && overrides.operatingPoint->newton &&
            overrides.operatingPoint->newton->maximumIterations &&
            *overrides.operatingPoint->newton->maximumIterations == 41,
        "OP Newton integer override is retained"
    );
    expect(
        overrides.operatingPoint &&
            overrides.operatingPoint->sourceStepping &&
            overrides.operatingPoint->sourceStepping->enabled &&
            !*overrides.operatingPoint->sourceStepping->enabled,
        "OP source-stepping boolean override is retained"
    );
    expect(
        overrides.pta && overrides.pta->mode &&
            *overrides.pta->mode == simulator::config::PtaModeOverride::Force,
        "PTA mode is parsed case-insensitively"
    );
    expect(
        overrides.pta && overrides.pta->initialStep &&
            *overrides.pta->initialStep == 2.5e-9,
        "PTA SPICE numeric string is converted"
    );
    expect(
        overrides.pta && overrides.pta->maximumSteps &&
            *overrides.pta->maximumSteps == 7,
        "PTA integer override is retained"
    );
    expect(
        overrides.pta && overrides.pta->newton &&
            overrides.pta->newton->maximumIterations &&
            *overrides.pta->newton->maximumIterations == 53,
        "PTA Newton override is retained"
    );
    expect(
        overrides.pta && overrides.pta->initialBjtVbe.specified &&
            overrides.pta->initialBjtVbe.value &&
            *overrides.pta->initialBjtVbe.value == 0.72,
        "PTA BJT voltage override is retained"
    );
    expect(
        overrides.pta && overrides.pta->includeDiodes &&
            !*overrides.pta->includeDiodes,
        "PTA boolean override is retained"
    );
    expect(
        overrides.transient && overrides.transient->enabled &&
            *overrides.transient->enabled &&
            overrides.transient->stopTime &&
            *overrides.transient->stopTime == 1e-8,
        "transient numeric and boolean overrides are retained"
    );
    expect(
        overrides.transient && overrides.transient->solver &&
            overrides.transient->solver->maximumRejects &&
            *overrides.transient->solver->maximumRejects == 0,
        "transient solver integer override is retained"
    );
    expect(
        overrides.transient && overrides.transient->solver &&
            overrides.transient->solver->newton &&
            overrides.transient->solver->newton->tolerance &&
            std::abs(
                *overrides.transient->solver->newton->tolerance - 3e-9
            ) < 1e-20,
        "transient Newton override is retained"
    );

    const auto nullBjtVbe = simulator::config::parseConfigOverrides(
        loadedConfigFor(nlohmann::json::parse(R"(
            {"schema_version": 1, "pta": {"initial_bjt_vbe": null}}
        )"))
    );
    expect(
        nullBjtVbe.pta && nullBjtVbe.pta->initialBjtVbe.specified &&
            !nullBjtVbe.pta->initialBjtVbe.value,
        "null BJT voltage explicitly clears the override"
    );
}

void testConfigOverrideApplication(){
    const auto overrides = simulator::config::parseConfigOverrides(
        loadedConfigFor(nlohmann::json::parse(R"(
            {
                "schema_version": 1,
                "op": {
                    "newton": {
                        "maximum_iterations": 31,
                        "tolerance": "4n",
                        "maximum_solution_step": 0.5
                    },
                    "source_stepping": {
                        "enabled": false,
                        "failure_scale": 0.4
                    }
                },
                "pta": {
                    "mode": "fallback",
                    "newton": {"maximum_iterations": 47},
                    "initial_bjt_vbe": null
                },
                "tran": {
                    "enabled": true,
                    "output_interval": "2n",
                    "stop_time": "20n",
                    "maximum_step": "5n",
                    "solver": {
                        "newton": {"maximum_solution_step": 0.2},
                        "maximum_rejects": 4
                    }
                }
            }
        )"))
    );

    OperatingPointSolverOptions operatingPoint;
    PtaAnalysisConfig pta;
    pta.initialBjtVbe = 0.65;
    std::optional<TransientAnalysisConfig> transient;
    transient.emplace();
    transient->outputInterval = 1e-9;
    transient->stopTime = 10e-9;
    transient->maximumStep = 1e-9;

    simulator::config::applyConfigOverrides(
        overrides,
        operatingPoint,
        pta,
        transient,
        false
    );

    expect(
        operatingPoint.newton.maximumIterations == 31 &&
            operatingPoint.newton.tolerance == 4e-9 &&
            operatingPoint.newton.maximumSolutionStep == 0.5,
        "OP Newton overrides are applied"
    );
    expect(
        !operatingPoint.sourceStepping.enabled &&
            operatingPoint.sourceStepping.failureScale == 0.4,
        "OP source-stepping overrides are applied"
    );
    expect(
        pta.mode == PtaMode::Fallback &&
            pta.newtonOptions.maximumIterations == 47 &&
            !pta.initialBjtVbe,
        "PTA overrides are applied"
    );
    expect(
        transient && transient->outputInterval == 2e-9 &&
            transient->stopTime == 20e-9 &&
            transient->maximumStep && *transient->maximumStep == 1e-9,
        "transient settings apply without relaxing netlist step caps"
    );
    expect(
        transient && transient->solverOptions.maximumRejects == 4 &&
            transient->solverOptions.newtonOptions.maximumSolutionStep == 0.2,
        "transient solver overrides are applied"
    );

    const auto invalidPstranMode = simulator::config::parseConfigOverrides(
        loadedConfigFor(nlohmann::json::parse(R"(
            {"schema_version": 1, "pta": {"mode": "disabled"}}
        )"))
    );
    expectInvalidArgument(
        [&] {
            simulator::config::applyConfigOverrides(
                invalidPstranMode,
                operatingPoint,
                pta,
                transient,
                true
            );
        },
        ".pstran mode cannot be disabled by configuration"
    );

    const auto invalidOperatingPoint = simulator::config::parseConfigOverrides(
        loadedConfigFor(nlohmann::json::parse(R"(
            {
                "schema_version": 1,
                "op": {"newton": {"tolerance": 0}}
            }
        )"))
    );
    OperatingPointSolverOptions invalidOptions;
    PtaAnalysisConfig defaultPta;
    std::optional<TransientAnalysisConfig> noTransient;
    simulator::config::applyConfigOverrides(
        invalidOperatingPoint,
        invalidOptions,
        defaultPta,
        noTransient,
        false
    );
    expect(
        !invalidOptions.valid(),
        "invalid OP numeric range is deferred to runtime validation"
    );
}

void testInvalidConfigOverrides(){
    const auto expectInvalid = [](
        const char* document,
        const std::string& description
    ){
        expectRuntimeError(
            [&] {
                static_cast<void>(simulator::config::parseConfigOverrides(
                    loadedConfigFor(nlohmann::json::parse(document))
                ));
            },
            description
        );
    };

    expectInvalid("{}", "configuration schema version is required");
    expectInvalid(
        R"({"schema_version": 2})",
        "unsupported configuration schema version is rejected"
    );
    expectInvalid(
        R"({"schema_version": 1, "unknown": true})",
        "unknown top-level configuration field is rejected"
    );
    expectInvalid(
        R"({"schema_version": 1, "pta": {"maximum_steps": 1.5}})",
        "fractional PTA integer field is rejected"
    );
    expectInvalid(
        R"({"schema_version": 1, "pta": {"include_diodes": "true"}})",
        "non-boolean PTA flag is rejected"
    );
    expectInvalid(
        R"({"schema_version": 1, "tran": {"solver": []}})",
        "non-object transient solver section is rejected"
    );
    expectInvalid(
        R"({"schema_version": 1, "tran": {"maximum_step": "abc"}})",
        "invalid SPICE numeric string is rejected"
    );
    expectInvalid(
        R"({"schema_version": 1, "op": {"newton": {"typo": 1}}})",
        "unknown OP Newton field is rejected"
    );
    expectInvalid(
        R"({"schema_version": 1, "op": {"source_stepping": true}})",
        "non-object OP source-stepping section is rejected"
    );
}

}  // namespace

int main(){
    try {
        testAutomaticDiscovery();
        testSearchLimitAndDefaultFallback();
        testExplicitConfiguration();
        testInvalidConfigurationFiles();
        testConfigOverrideParsing();
        testConfigOverrideApplication();
        testInvalidConfigOverrides();
    } catch(const std::exception& error) {
        std::cerr << "Unexpected test error: " << error.what() << '\n';
        return 1;
    }

    std::cout << "Configuration tests: "
              << (checkCount - failureCount) << '/' << checkCount
              << " checks passed\n";
    return failureCount == 0 ? 0 : 1;
}
