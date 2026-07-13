#include "../include/core/nodeMap.h"
#include "../include/devices/device.hpp"

#include "../../utils/string_utils.hpp"

void Device::bindNodes(const NodeMap& nodemap){
    nodeIds.resize(nodes.size());

    for(std::size_t i = 0; i < nodes.size(); ++i){
        nodeIds[i] = nodemap.idxOf(nodes[i]);
    }
}

void NodeMap::build(const std::vector<std::unique_ptr<Device>>& devices){
    name_to_idx.clear();
    idx_to_name.clear();

    for(auto& device : devices){
        std::vector<std::string> nodes = device->getNodes();
        for(auto& node : nodes){
            node = to_lower_copy(node);
            if(node == "0" || node == "gnd"){
                continue;
            }
            auto it = name_to_idx.find(node);
            if(it != name_to_idx.end()){
                continue;
            }
            name_to_idx[node] = name_to_idx.size();
            idx_to_name.push_back(node);
        }
    }
}

int NodeMap::idxOf(std::string name) const {
    name = to_lower_copy(name);
    if(name == "0" || name == "gnd"){
        return -1;
    }
    
    auto it = name_to_idx.find(name);
    if(it == name_to_idx.end()){
        throw std::runtime_error("Unknown node: " + name);
    }

    return it->second;
}

const std::vector<std::string>& NodeMap::nodeNameByIdx() const {
    return idx_to_name;
}

int NodeMap::nodeCount() const{
    return name_to_idx.size();
}
