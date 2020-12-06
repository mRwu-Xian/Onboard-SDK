#include "test.hpp"
#include <dji_linux_helpers.hpp>
#include <dji_vehicle.hpp>
#include <iostream>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc , char** argv){
    int responseTimeout = 1;
    LinuxSetup linuxEnvironment(argc , argv);
    Vehicle* vehicle = linuxEnvironment.getVehicle();
    if(vehicle == NULL)
    {
        std::cout << "Vehicle not initialized, exiting!";
        return -1;
    }

    subscribeOne(vehicle , responseTimeout);
    telemetryOne (vehicle);
    vehicle->subscribe->removeAllExistingPackages();
    
}