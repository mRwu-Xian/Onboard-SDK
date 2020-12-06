#include "collect_Waypoints.hpp"
#include "dji_ack.hpp"
#include <iostream>
using namespace DJI;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

#define GIMBA_SUB_PACKAGE_INDEX 0

//typedef FlightJoystick::ControlMode JoystickMode;

typedef FlightJoystick::ControlCommand JoystickCommand; 
typedef FlightJoystick::ControlMode JoystickMode;



static const double EarthCenter = 6378137.0;


bool MoveTakeoff(Vehicle* vehicle , int timeout ){
  

    vehicle->flightController->startTakeoffSync(timeout); 

    if(!motorStartedCheck(vehicle))
    {
        std::cout << "Takeoff failed. Motors are not spinning."<< std::endl;
        return false;
    }else
    {
        std::cout << "Motors spinning...\n" ;
    }
    if (!takeOffInAirCheck(vehicle)) {
    std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                 "motors are spinning."
              << std::endl;
    return false;
  } else {
    std::cout << "Ascending...\n";
  }

  //! Finished takeoff check
  if (takeoffFinishedCheck(vehicle)) {
    std::cout << "Successful takeoff!\n";
  } else {
    std::cout << "Takeoff finished, but the aircraft is in an unexpected mode. "
                 "Please connect DJI GO.\n";
    return false;
  }
  return true;
}

//----------------------------------------------




/*
bool Moveflight(Vehicle* vehicle , const Vector3f& offsetDesired,
                        float yawDesiredInDeg,
                        float posThresholdInM,
                        float yawThresholdInDeg
                         )
{
    char func[50];
    int responseTimeout = 1;
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    };

    int pkgIndex = 0;
    int freq = 50;
    TopicName topicList50Hz[] = { TOPIC_GPS_FUSED,TOPIC_QUATERNION  };
    int numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    int enableTimestamp = false;
    
    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    } 


    sleep(1);
   
    //----------------------------------------------------------------- 获取老位置,以及通过广播获取高度
    TypeMap<TOPIC_GPS_FUSED>::type originGPSPosition =
        vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    GlobalPosition currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    float32_t originHeightBaseHomepoint = currentBroadcastGP.height;
    //------------------------------------------------------------------ 操作模式设置
    FlightController::JoystickMode joystickMode = {
        
        FlightController::HorizontalLogic::HORIZONTAL_POSITION,
        FlightController::VerticalLogic::VERTICAL_POSITION,
        FlightController::YawLogic::YAW_ANGLE,
        FlightController::HorizontalCoordinate::HORIZONTAL_GROUND,
        FlightController::StableMode::STABLE_ENABLE, 
    };
    vehicle->flightController->setJoystickMode(joystickMode);
    //------------------------------------------------------------------ 计算走到的位置
    TypeMap<TOPIC_GPS_FUSED>::type currentGPSPosition =  //现在的gps位置
        vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
       
    TypeMap<TOPIC_QUATERNION>::type currentQuaternion =  //现在的飞行姿态
        vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    Vector3f localOffset =  localOffsetFromGpsOffset(currentGPSPosition, originGPSPosition);
    Vector3f offsetRemaining = vector3FSub(offsetDesired, localOffset);
    Vector3f positionCommand = offsetRemaining;
    
    JoystickCommand joystickcommand = {
        positionCommand.x ,
        positionCommand.y ,  
        offsetDesired.z + originHeightBaseHomepoint, yawDesiredInDeg
    };
    vehicle->flightController->setJoystickCommand(joystickcommand);

    vehicle->flightController->joystickAction();

     sleep(3);
     //-------------------------------------------------------拿到现在的位置,存入数组中
     WaypointV2 Waypts;
      TypeMap<TOPIC_GPS_FUSED>::type  GPSrecording =
         vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
         Waypts.latitude  = GPSrecording.latitude;
         Waypts.longitude = GPSrecording.longitude;
         Waypts.relativeHeight  = GPSrecording.altitude;
      generatedWaypints.push_back(Waypts);

     vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
}
*/
bool MoveFlight(Vehicle* vehicle , const Vector3f& offsetDesired,  //想要的偏移
                                        float yawDesiredInDeg,  //期望的偏航
                                        float posThresholdInM,  //位置的预值
                                        float yawThresholdInDeg) { //偏航预值

  int timeoutInMilSec = 20000;
  int controlFreqInHz = 50;
  int cycleTimeInMs = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
  int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs;  // 100 cycles
  int elapsedTimeInMs = 0;
  int withinBoundsCounter = 0;
  int outOfBounds = 0;
  int brakeCounter = 0;
  int speedFactor = 2;

  //! get origin position and relative height(from home point)of aircraft. 获取飞机的原点位置和相对高度（从原点开始）。
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originGPSPosition =
      vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
  /*! TODO: TOPIC_HEIGHT_FUSION is abnormal in real world but normal in simulator 待办事项：TOPIC_HEIGHT_FUSION在现实世界中是异常的，但在模拟器中是正常的*/
  Telemetry::GlobalPosition currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
  float32_t originHeightBaseHomepoint = currentBroadcastGP.height; //超声波高度
  FlightController::JoystickMode joystickMode = {   //操作杆的控制模式
    FlightController::HorizontalLogic::HORIZONTAL_POSITION,
    FlightController::VerticalLogic::VERTICAL_POSITION,
    FlightController::YawLogic::YAW_ANGLE,
    FlightController::HorizontalCoordinate::HORIZONTAL_GROUND,
    FlightController::StableMode::STABLE_ENABLE,
  };
  vehicle->flightController->setJoystickMode(joystickMode);

  while (elapsedTimeInMs < timeoutInMilSec) {
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentGPSPosition =
        vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    Telemetry::TypeMap<TOPIC_QUATERNION>::type currentQuaternion =
        vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    float yawInRad = quaternionToEulerAngle(currentQuaternion).z;
    //! get the vector between aircraft and origin point.获取飞机和原点之间的向量。
    Vector3f localOffset =
        localOffsetFromGpsOffset(currentGPSPosition, originGPSPosition);
    //! get the vector between aircraft and target point.获取飞机和目标点之间的向量。
    Vector3f offsetRemaining = vector3FSub(offsetDesired, localOffset);//计算相对距离就是我们需要走的

    Vector3f positionCommand = offsetRemaining;
    horizCommandLimit(speedFactor, positionCommand.x, positionCommand.y);  //？？？？？？

    FlightController::JoystickCommand joystickCommand = {                //操纵杆命令
        positionCommand.x, positionCommand.y,
        offsetDesired.z + originHeightBaseHomepoint, yawDesiredInDeg};

    vehicle->flightController->setJoystickCommand(joystickCommand); //设置操作杆命令


    vehicle->flightController->joystickAction();  //操作杆动作

    if (vectorNorm(offsetRemaining) < posThresholdInM &&
        std::fabs(yawInRad / DEG2RAD - yawDesiredInDeg) < yawThresholdInDeg) {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    } else {
      if (withinBoundsCounter != 0) {
        
        //! 2. Start incrementing an out-of-bounds counter
        outOfBounds += cycleTimeInMs;
      }
    }
    //! 3. Reset withinBoundsCounter if necessary
    if (outOfBounds > outOfControlBoundsTimeLimit) {
      withinBoundsCounter = 0;
      outOfBounds = 0;
    }
    //! 4. If within bounds, set flag and break
    if (withinBoundsCounter >= withinControlBoundsTimeReqmt) {
      break;
    }
    usleep(cycleTimeInMs * 1000);
    elapsedTimeInMs += cycleTimeInMs;
    DERROR("ARE YOU OK elspsed :%d?",elapsedTimeInMs);
  }
  DERROR("ARE YOU OK1?");
  while (brakeCounter < withinControlBoundsTimeReqmt) {
    //! TODO: remove emergencyBrake 待办事项：卸下紧急制动器
    vehicle->control->emergencyBrake();
    usleep(cycleTimeInMs * 1000);
    brakeCounter += cycleTimeInMs;
    DERROR("ARE YOU OK2?");
  }
  DERROR("ARE YOU OK3?");
  WaypointV2 Waypts;
      TypeMap<TOPIC_GPS_FUSED>::type  GPSrecording =
         vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
         Waypts.latitude  = GPSrecording.latitude;
         Waypts.longitude = GPSrecording.longitude;
         Waypts.relativeHeight  = GPSrecording.altitude;
      generatedWaypints.push_back(Waypts);

  if (elapsedTimeInMs >= timeoutInMilSec) {
    std::cout << "Task timeout!\n";
    return false;
  }
  return true;
}


Vector3f vector3FSub(const Vector3f& vectorA,
                                   const Vector3f& vectorB) {
  Vector3f result;
  result.x = vectorA.x - vectorB.x;
  result.y = vectorA.y - vectorB.y;
  result.z = vectorA.z - vectorB.z;
  return result;
}

bool MoveGimbal(Vehicle* vehicle , int timeout , float pitch )
{

  ErrorCode::ErrorCodeType ret;
  /*! main gimbal init */
  ret = vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_0,
                                                 "Sample_main_gimbal");
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Init Camera module Sample_main_gimbal failed.");
    ErrorCode::printErrorCodeMsg(ret);
  }
  GimbalManager *p = vehicle->gimbalManager;


  TypeMap<TOPIC_GIMBAL_ANGLES>::type Gimbal;   
  Gimbal  =   vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>();
  DERROR("gimbal Old Yaw : %d",Gimbal.y);
  DERROR("gimbal Old Pitch : %d",Gimbal.x);
  DERROR("gimbal Old Roll : %d",Gimbal.z);

  GimbalModule::Rotation rotation;
  rotation.roll = 0.0f;
  rotation.pitch = pitch;
  rotation.yaw = 0.0f;
  rotation.rotationMode = 0;
  rotation.time = 0.5;
  vehicle->gimbalManager->rotateSync(PAYLOAD_INDEX_0 , rotation , timeout );
  Gimbal = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>();
  DERROR("gimbal Old Yaw : %d",Gimbal.z);
  DERROR("gimbal Old Pitch : %d",Gimbal.x);
  DERROR("gimbal Old Roll : %d",Gimbal.y);
  sleep(5);
}





//---------------------------------------------------------------------
float32_t vectorNorm(Vector3f v) {
  return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

void horizCommandLimit(float speedFactor, float& commandX,
                                     float& commandY) {
  if (fabs(commandX) > speedFactor)
    commandX = signOfData<float>(commandX) * speedFactor;
  if (fabs(commandY) > speedFactor)
    commandY = signOfData<float>(commandY) * speedFactor;
}

template <typename Type>
int signOfData(Type type) {
  return type < 0 ? -1 : 1;
}

Vector3f localOffsetFromGpsOffset(
    const Telemetry::GPSFused& target, const Telemetry::GPSFused& origin) {
  Telemetry::Vector3f deltaNed;
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;
  deltaNed.x = deltaLat * EarthCenter;
  deltaNed.y = deltaLon * EarthCenter * cos(target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
  return deltaNed;
}



bool teardownSubscription(DJI::OSDK::Vehicle* vehicle ,const int pkgIndex, int timeout) {
  ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
  if (ACK::getError(ack)) {
    DERROR(
        "Error unsubscription; please restart the drone/FC to get back "
        "to a clean state.");
    return false;
  }
  return true;
}


void asyncSampleCallBack(ErrorCode::ErrorCodeType retCode, UserData SampleLog) {
  DSTATUS("retCode : 0x%lX", retCode);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Pass : %s.", SampleLog);
  } else {
    DERROR("Error : %s. Error code : %d", SampleLog, retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }
}


Vector3f quaternionToEulerAngle(
    const Telemetry::Quaternion& quat) {
  Telemetry::Vector3f eulerAngle;
  double q2sqr = quat.q2 * quat.q2;
  double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
  double t1 = 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
  double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
  double t3 = 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
  double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;
  eulerAngle.x = asin(t2);
  eulerAngle.y = atan2(t3, t4);
  eulerAngle.z = atan2(t1, t0);
  return eulerAngle;
}
        
//---------------------------------------------------------------------------

bool motorStartedCheck(Vehicle* vehicle) {
    int motorsNotStarted = 0;
    int timeoutCycles = 20;
    while ( vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
        VehicleStatus::FlightStatus::ON_GROUND &&
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
        VehicleStatus::DisplayMode::MODE_ENGINE_START &&
         motorsNotStarted < timeoutCycles) {
        motorsNotStarted++;
        usleep(100000);
    }
        return motorsNotStarted != timeoutCycles ? true : false;
}
    
bool takeOffInAirCheck(Vehicle* vehicle){                 //空中check
        int stillOnGround = 0;
        int timeoutCycles = 110;
        while ( vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
            VehicleStatus::FlightStatus::IN_AIR &&
                (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
                vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
            stillOnGround < timeoutCycles) {
        stillOnGround++;
        usleep(100000);
    }
        return stillOnGround != timeoutCycles ? true : false;
}

bool takeoffFinishedCheck(Vehicle* vehicle) {                        //起飞完成检查 
        while ( vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
            VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
                vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
            VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) {
        sleep(1);
        }
        return ((vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
        VehicleStatus::DisplayMode::MODE_P_GPS) ||
            (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
        VehicleStatus::DisplayMode::MODE_ATTITUDE))
            ? true
            : false;
}


bool setUpSubscription(Vehicle* vehicle , int pkgIndex, int freq,
                                     TopicName topicList[], uint8_t topicSize,
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






bool getHomeLocation(DJI::OSDK::Vehicle* vehicle , HomeLocationSetStatus& homeLocationSetStatus,HomeLocationData& homeLocationInfo,int responseTimeout)
{

  /*! Wait for the data to start coming in.*/
  Platform::instance().taskSleepMs(2000);
  homeLocationSetStatus =
      vehicle->subscribe->getValue<TOPIC_HOME_POINT_SET_STATUS>();
  homeLocationInfo = vehicle->subscribe->getValue<TOPIC_HOME_POINT_INFO>();
  DERROR("POINT %d",homeLocationInfo);
  return true;
}



bool goHomeAndConfirmLanding(DJI::OSDK::Vehicle* vehicle , int timeout){
 
  ErrorCode::ErrorCodeType goHomeAck =
      vehicle->flightController->startGoHomeSync(timeout);
  if (goHomeAck != ErrorCode::SysCommonErr::Success) {
    DERROR("Fail to execute go home action!  Error code: %llx\n", goHomeAck);
    return false;
  }
  DSTATUS("Finished go home action");
}


ErrorCode::ErrorCodeType setNewHomeLocation(DJI::OSDK::Vehicle* vehicle , int timeout){
  HomeLocationSetStatus homeLocationSetStatus;
  HomeLocationData originHomeLocation;
  ErrorCode::ErrorCodeType ret =
      ErrorCode::FlightControllerErr::SetHomeLocationErr::Fail;
  bool retCode = getHomeLocation(vehicle , homeLocationSetStatus , originHomeLocation, 1);
  if(retCode && homeLocationSetStatus.status == 1){
    ret = vehicle->flightController->setHomeLocationUsingCurrentAircraftLocationSync(timeout);
  if(ret != ErrorCode::SysCommonErr::Success){
    DSTATUS("Set new home location failed, ErrorCode is:%8x", ret);
    } else {
      DSTATUS("Set new home location successfully");
    } 
  return ret;
  }
}

ErrorCode::ErrorCodeType setGoHomeAltitude(DJI::OSDK::Vehicle* vehicle , 
    FlightController::GoHomeHeight altitude, int timeout) {
  ErrorCode::ErrorCodeType ret =
      vehicle->flightController->setGoHomeAltitudeSync(altitude, timeout);
  if (ret != ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set go home altitude failed, ErrorCode is:%8x", ret);
  } else {
    DSTATUS("Set go home altitude successfully,altitude is: %d", altitude);
  }
  return ret;
}

bool getHomePOINT(DJI::OSDK::Vehicle* vehicle , int responseTimeout)
{
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }
  int pkgIndex = 0;
  int freq = 1;
  TopicName topicList[] = {TOPIC_HOME_POINT_SET_STATUS, TOPIC_HOME_POINT_INFO};

  int numTopic = sizeof(topicList) / sizeof(topicList[0]);
  bool enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList, enableTimestamp, freq);

  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    /*! Cleanup before return */
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }
  TypeMap<TOPIC_HOME_POINT_INFO>::type     homepoint;
  homepoint  =  vehicle->subscribe->getValue<TOPIC_HOME_POINT_INFO>();
  // STD("HOME POINT %d",homepoint.latitude);
  DERROR("POINT %d",homepoint);
  
  vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
}