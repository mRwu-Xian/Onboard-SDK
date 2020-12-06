#include "test.hpp"
#include <dji_telemetry.hpp>
#include  <signal.h>
#include  <stdlib.h>
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;



bool subscribeOne(Vehicle* vehicle , int responseTimeout)
{
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return false;
    }
    int pkgIndex = 0;
    int freq = 50;
    TopicName topicList50Hz[] = {
        TOPIC_VELOCITY,
        TOPIC_GPS_FUSED
    };
    int numTopic = sizeof(topicList50Hz)  /  sizeof(topicList50Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
        pkgIndex , numTopic , topicList50Hz , enableTimestamp , freq);
    if(!(pkgStatus)){
        return pkgStatus;
    }
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex , responseTimeout);
    if(ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getError(subscribeStatus);
        vehicle->subscribe->removePackage(pkgIndex,responseTimeout);
        return false;
    }
}

bool telemetryOne(Vehicle* vehicle)
{
    TypeMap<TOPIC_VELOCITY>::type       Velocity;
    TypeMap<TOPIC_GPS_FUSED>::type      GPSposition;

    Velocity = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
    GPSposition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();

    DSTATUS("we telemetry VelocityData x: %f", Velocity.data.x);
    DSTATUS("we telemetry VelocityData y: %f", Velocity.data.y);
    DSTATUS("we telemetry VelocityData z: %f", Velocity.data.z);
    DSTATUS("we telemetry GPSData LATIA: %f", GPSposition.latitude);
    DSTATUS("we telemetry GPSData lONG: %f", GPSposition.longitude);
    DSTATUS("we telemetry GPSData Altit: %f", GPSposition.altitude);

}