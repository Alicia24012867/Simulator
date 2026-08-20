#include "config/config.h"

#include <chrono>
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
        "loaded document preserves JSON values"
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
        "missing automatic configuration returns an empty JSON object"
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
        "explicit configuration preserves JSON boolean values"
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

    const std::filesystem::path invalidJson =
        temporary.path() / "invalid.json";
    writeFile(invalidJson, "{");
    options.explicitPath = invalidJson;
    expectRuntimeError(
        [&] {
            static_cast<void>(
                simulator::config::loadConfig(options, temporary.path())
            );
        },
        "invalid JSON is rejected"
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
        "non-object JSON root is rejected"
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

}  // namespace

int main(){
    try {
        testAutomaticDiscovery();
        testSearchLimitAndDefaultFallback();
        testExplicitConfiguration();
        testInvalidConfigurationFiles();
    } catch(const std::exception& error) {
        std::cerr << "Unexpected test error: " << error.what() << '\n';
        return 1;
    }

    std::cout << "Configuration tests: "
              << (checkCount - failureCount) << '/' << checkCount
              << " checks passed\n";
    return failureCount == 0 ? 0 : 1;
}
