#pragma once

#include <cassert>

#include "device.hpp"

class Capacitor: public Device{
public:
    Capacitor(std::string name, std::vector<std::string> nodes, double C):
            Device(name, nodes, DeviceType::Capacitor){
        assert(C > 0.0);
    }

    void pattern(MNA&) override{}
    void bindMatrix(MNA&) override{}
    void stampOperatingPoint() override{}
};
