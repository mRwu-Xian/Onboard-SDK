#include "dji_telemetry.hpp"
#include "dji_vehicle.hpp"
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


bool telemetryOne (DJI::OSDK::Vehicle* vehicle);

bool subscribeOne(DJI::OSDK::Vehicle* vehicle , int responseTimeout);
