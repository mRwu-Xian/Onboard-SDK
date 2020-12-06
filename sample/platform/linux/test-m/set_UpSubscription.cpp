#include "set_UpSubscription.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


bool Subscription::setUpSubscription(Vehicle* vehicle){

        int pkgIndex = 0;
        int freq = 50;
        TopicName topicList50Hz = {
                TOPIC_GPS_FUSED,
                TOPIC_STATUS_DISPLAYMODE,
                TOPIC_STATUS_FLIGHT,
                TOPIC_GIMBAL_ANGLES
        };
        int topicSize = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
        int timeout = 1;
        /*! Telemetry: Verify the subscription*/
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            return false;
        }

        bool enableTimestamp = false;
        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, topicSize, topicList50Hz, enableTimestamp, freq);
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

        pkgIndex = 1;
        freq = 200;
        TopicName topicList200Hz[] = {
            TOPIC_QUATERNION,
            TOPIC_VELOCITY
        };
        int topicSize = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
        /*! Telemetry: Verify the subscription*/
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            return false;
        }

        bool enableTimestamp = false;
        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, topicSize, topicList200Hz, enableTimestamp, freq);
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
        pkgIndex = 2;
        freq = 100;
        TopicName topicList100Hz[] = {
            TOPIC_HEIGHT_FUSION,
            TOPIC_HOME_POINT_INFO,
            TOPIC_HOME_POINT_SET_STATUS,
            TOPIC_AVOID_DATA
        };
        topicSize = sizeof(topicList100Hz) / sizeof(topicList100Hz[0])
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            return false;
        }

        bool enableTimestamp = false;
        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, topicSize, topicList100Hz, enableTimestamp, freq);
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
}




bool Subscription::teardownSubscription(Vehicle* vehicle){

    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->removeAllExistingPackages();
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
}