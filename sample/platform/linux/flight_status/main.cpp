#include "flight_flag.hpp"
#include <iostream>
#include "dji_vehicle.hpp"
#include "dji_linux_helpers.hpp"
#include "dji_telemetry.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool setUpSubscription(Vehicle* vehicle , int pkgIndex, int freq,
                                     TopicName topicList[], int topicSize,
                                     int timeout);
bool teardownSubscription(Vehicle* vehicle , const int pkgIndex, int timeout);

bool setUpSubscription(Vehicle* vehicle , int pkgIndex, int freq,
                                     TopicName topicList[] , int topicSize,
                                     int timeout) {
  if (vehicle) {
    /*! Telemetry: Verify the subscription*/
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      return false;
    }

    bool enableTimestamp = false;
    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
        pkgIndex, topicSize, topicList, enableTimestamp, freq);
    if (!(pkgStatus)) {
      return pkgStatus;
    }

    /*! Start listening to the telemetry data */
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      /*! Cleanup*/
      ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
      if (ACK::getError(ack)) {
        DERROR(
            "Error unsubscription; please restart the drone/FC to get "
            "back to a clean state");
      }
      return false;
    }
    return true;
  } else {
    DERROR("vehicle haven't been initialized", __func__);
    return false;
  }
}


bool teardownSubscription(Vehicle *vehicle , const int pkgIndex, int timeout) {
  ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
  if (ACK::getError(ack)) {
    DERROR(
        "Error unsubscription; please restart the drone/FC to get back "
        "to a clean state.");
    return false;
  }
  return true;
}

int main(int argc, char **argv){
    
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle* vehicle = linuxEnvironment.getVehicle();
    if(vehicle == NULL)
    {
        std::cout << "Vehicle not initialized , exiting.\n";
        return -1;
    }
    vehicle->control->obtainCtrlAuthority();
    int pkgIndex = 0;
    int freq = 5;
    TopicName topicList5Hz[] = {
        TOPIC_RTK_POSITION
    };
    int topicSize = sizeof(topicList5Hz) / sizeof(topicList5Hz[0]);
    int timeout = 1;
    setUpSubscription(vehicle , pkgIndex, freq, topicList5Hz , topicSize,
                                     timeout);

    while(1)
    {
        UVA_status *UVAone = new UVA_status;
        UVA_status::Point position;
        TypeMap<TOPIC_RTK_POSITION>::type  RTK_POSITION;
        RTK_POSITION = vehicle->subscribe->getValue<TOPIC_RTK_POSITION>();
        position.lat = RTK_POSITION.latitude;
        position.lon = RTK_POSITION.longitude;
        position.alt = RTK_POSITION.HFSL;
        UVAone->judge_Flightstatus(position); 
    };       
}






