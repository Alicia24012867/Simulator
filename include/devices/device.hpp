#pragma once

#include <string>
#include <vector>

enum class DeviceType{
    Resistor,
    Inductor,
    Capacitor,
    VoltageSource,
    CurrentSource,
    Diode,
    BJT,
    MOSFET
};

inline const char* deviceTypeName(DeviceType type){
    switch (type)
    {
        case DeviceType::Resistor:{
            return "Resistor";
        }
        case DeviceType::Inductor:{
            return "Inductor";
        }
        case DeviceType::Capacitor:{
            return "Capacitor";
        }
        case DeviceType::VoltageSource:{
            return "Voltage Source";
        }
        case DeviceType::CurrentSource:{
            return "Current Source";
        }
        case DeviceType::Diode:{
            return "Diode";
        }
        case DeviceType::BJT:{
            return "BJT";
        }
        case DeviceType::MOSFET:{
            return "MOSFET";
        }
        default:{
            return "Unknown";
        }
    }
}


class Circuit;
class MNA;
class NodeMap;

class Device{
public:
    Device(std::string n, std::vector<std::string> ns, DeviceType t): name(n), nodes(ns), type(t){}
    virtual ~Device() = default;

    DeviceType getType() const{
        return type;
    }

    const std::vector<std::string>& getNodes() const {
        return nodes;
    }

    const std::vector<int>& getNodeIds() const {
        return nodeIds;
    }

    const std::string& getName() const {
        return name;
    }

    void bindNodes(const NodeMap& nodemap);

    virtual void allocateUnknown(Circuit&) {}

    virtual int branchUnknown() const {
        return -1;
    }

    virtual bool isNonlinear() const {
        return false;
    }

    virtual void pattern(MNA& mna) = 0;

    virtual void bindMatrix(MNA& mna) = 0;

    virtual void stamp() = 0;

    virtual void setSourceScale(double) {}

    virtual void saveState() {}

    virtual void restoreState() {}

protected:
    std::string name;
    std::vector<std::string> nodes;
    std::vector<int> nodeIds;
    DeviceType type;
};
