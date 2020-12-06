
#include "wayPoint_Mission.hpp"
#include <iostream>
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;




ErrorCode::ErrorCodeType runWaypointV2Mission(Vehicle* vehicle  )
{
    int timeout = 1;
    ErrorCode::ErrorCodeType ret;
   /*初始化任务*/
    ret = initMissionSetting(vehicle , timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
        return ret;
    sleep(timeout);

    /*! upload mission 上传任务*/
    uploadWaypointMission(vehicle , timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
      return ret;
    sleep(timeout);
    /*下载任务*/
    std::vector<WaypointV2> mission;
    downloadWaypointMission(vehicle , mission, timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
        return ret;
    sleep(timeout);
    /*! upload  actions 上传动作*/
    uploadWapointActions(vehicle , timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
        return ret;
    sleep(timeout);
    /*! start mission 开始任务*/
    startWaypointMission(vehicle , timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
        return ret;
    sleep(20);
    /*! set global cruise speed 设定全球巡航速度*/
    setGlobalCruiseSpeed(vehicle , 1.5, timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
        return ret;
    sleep(timeout);

    // /*! get global cruise speed 获得全球巡航速度*/
    // getGlobalCruiseSpeed(vehicle , timeout);
    // if(ret != ErrorCode::SysCommonErr::Success)
    //     return ret;
    // sleep(timeout);

    // /*! pause the mission 暂停任务*/
    // pauseWaypointMission(vehicle , timeout);
    // if(ret != ErrorCode::SysCommonErr::Success)
    //     return ret;
    // sleep(5);

    // /*! resume the mission 恢复任务*/
    // resumeWaypointMission(vehicle , timeout);
    // if(ret != ErrorCode::SysCommonErr::Success)
    //     return ret;
    /*! Set up telemetry subscription 设置遥测订阅*/
    sleep(20);
    return ErrorCode::SysCommonErr::Success;
}
    
    
    ErrorCode::ErrorCodeType initMissionSetting(Vehicle* vehicle ,int timeout){
        
        uint16_t polygonNum = 3;
        uint16_t actionNum = 5;
        float32_t radius = 6;
        actions = generateWaypointActions(actionNum);
        WayPointV2InitSettings missionInitSettings;
        missionInitSettings.repeatTimes                =  1;
        missionInitSettings.maxFlightSpeed             = 10;
        missionInitSettings.autoFlightSpeed            =  2;
        missionInitSettings.exitMissionOnRCSignalLost  =  1;
        missionInitSettings.missionID              = rand();
        missionInitSettings.finishedAction         = DJIWaypointV2MissionFinishedGoToFirstWaypoint;
        missionInitSettings.gotoFirstWaypointMode  = DJIWaypointV2MissionGotoFirstWaypointModeSafely;
        missionInitSettings.mission =  generatePolygonWaypoints(vehicle);
        missionInitSettings.missTotalLen = missionInitSettings.mission.size();

        ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->init(&missionInitSettings,timeout);
        if(ret != ErrorCode::SysCommonErr::Success)
        {
            DERROR("Init mission setting ErrorCode:0x%lX", ret);
            ErrorCode::printErrorCodeMsg(ret);
            return ret;
        }
        else
        {
            DSTATUS("Init mission setting successfully!");
        }
        return ret;
    }

    std::vector<DJIWaypointV2Action> generateWaypointActions(uint16_t actioNum){
        std::vector<DJIWaypointV2Action> actionVector;

        for(uint16_t i = 0 ; i < actioNum ; i++){
            DJIWaypointV2SampleReachPointTriggerParam sampleReachPointTriggerParam;
            sampleReachPointTriggerParam.waypointIndex = i;
            sampleReachPointTriggerParam.terminateNum  = 0;

            auto *trigger = new DJIWaypointV2Trigger(DJIWaypointV2ActionTriggerTypeSampleReachPoint,&sampleReachPointTriggerParam);
            auto *cameraActuatorParam = new DJIWaypointV2CameraActuatorParam(DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto, nullptr);
            auto *actuator = new DJIWaypointV2Actuator(DJIWaypointV2ActionActuatorTypeCamera/*确定有谁去执行动作*/, 0, cameraActuatorParam/*这里下面一层会进行相对应的判断*/);
            auto *action = new DJIWaypointV2Action(i, *trigger,*actuator);
            actionVector.push_back(*action); //包含了 DJIWaypointV2Action中的三个信息 放入数组中
        }
        return actionVector;
    }
    
    std::vector<WaypointV2>        generatePolygonWaypoints(Vehicle* vehicle){
        std::vector<WaypointV2> waypointList;

        Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        float32_t TakeoffHight = subscribeGPosition.altitude;
        for(vector<WaypointV2>::iterator it = generatedWaypints.begin() ; it != generatedWaypints.end() ; ++it){

            setWaypointV2Defaults(*it);
            (*it).relativeHeight = (*it).relativeHeight - TakeoffHight;
            
            waypointList.push_back(*it);
            
        }
        return waypointList;
    }

    void setWaypointV2Defaults(WaypointV2& waypointV2) {

        waypointV2.waypointType = DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
        waypointV2.headingMode = DJIWaypointV2HeadingModeAuto;
        waypointV2.config.useLocalCruiseVel = 0;
        waypointV2.config.useLocalMaxVel = 0;

        waypointV2.dampingDistance = 40;
        waypointV2.heading = 0;
        waypointV2.turnMode = DJIWaypointV2TurnModeClockwise;

        waypointV2.pointOfInterest.positionX = 0;
        waypointV2.pointOfInterest.positionY = 0;
        waypointV2.pointOfInterest.positionZ = 0;
        waypointV2.maxFlightSpeed= 9;
        waypointV2.autoFlightSpeed = 2;
    }       





    ErrorCode::ErrorCodeType uploadWaypointMission(Vehicle* vehicle , int timeout){

        ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->uploadMission(timeout);
        if(ret != ErrorCode::SysCommonErr::Success)
        {
            DERROR("Upload waypoint v2 mission ErrorCode:0x%lX", ret);
            ErrorCode::printErrorCodeMsg(ret);
            return ret;
        }
        else{
            DSTATUS("Upload waypoint v2 mission successfully");
        }
        return ret;
    }

    ErrorCode::ErrorCodeType startWaypointMission(Vehicle* vehicle , int timeout) {

        ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->start(timeout);
        if(ret != ErrorCode::SysCommonErr::Success)
        {
            DERROR("Start waypoint v2 mission ErrorCode:0x%lX", ret);
            ErrorCode::printErrorCodeMsg(ret);
            return ret;
        }
        else
        {
        DSTATUS("Start waypoint v2 mission successfully!");
        }
        return ret;
    }

    ErrorCode::ErrorCodeType downloadWaypointMission(Vehicle* vehicle , std::vector<WaypointV2> &mission,int timeout)
    {
        ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->downloadMission(mission, timeout);
        if(ret != ErrorCode::SysCommonErr::Success)
        {
            DERROR("Download waypoint v2 mission ErrorCode:0x%lX", ret);
            ErrorCode::printErrorCodeMsg(ret);
            return ret;
        }
        else
        {
            DSTATUS("Download waypoint v2 mission successfully!");
        }
        return ret;
    }

    ErrorCode::ErrorCodeType uploadWapointActions(Vehicle* vehicle , int timeout)
    {
        ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->uploadAction(actions,timeout);
        if(ret != ErrorCode::SysCommonErr::Success)
        {
            DERROR("Upload waypoint v2 actions ErrorCode:0x%lX", ret);
            ErrorCode::printErrorCodeMsg(ret);
            return ret;
        }
        else
        {
            DSTATUS("Upload waypoint v2 actions successfully!");
        }
        return ret;
    }

    ErrorCode::ErrorCodeType WaypointV2MissionSample(Vehicle* vehicle , int timeout) {
        ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->start(timeout);
        if(ret != ErrorCode::SysCommonErr::Success)
        {
            DERROR("Start waypoint v2 mission ErrorCode:0x%lX", ret);
            ErrorCode::printErrorCodeMsg(ret);
            return ret;
        }
        else
        {
            DSTATUS("Start waypoint v2 mission successfully!");
        }
        return ret;
    }


    void setGlobalCruiseSpeed(Vehicle* vehicle , const GlobalCruiseSpeed &cruiseSpeed, int timeout)
    {
        ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->setGlobalCruiseSpeed(cruiseSpeed, timeout);
        if(ret !=  ErrorCode::SysCommonErr::Success)
        {
            DERROR("Set glogal cruise speed %f m/s failed ErrorCode:0x%lX", cruiseSpeed, ret);
            ErrorCode::printErrorCodeMsg(ret);
        return;
        }
        DSTATUS("Current cruise speed is: %f m/s", cruiseSpeed);
    }          

    void  getGlobalCruiseSpeed(Vehicle* vehicle , int timeout)
    {
        GlobalCruiseSpeed cruiseSpeed = 0;
        ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->getGlobalCruiseSpeed(cruiseSpeed, timeout);
        if(ret !=  ErrorCode::SysCommonErr::Success)
        {
            DERROR("Get glogal cruise speed failed ErrorCode:0x%lX", ret);
            ErrorCode::printErrorCodeMsg(ret);
            return;
        }
        DSTATUS("Current cruise speed is: %f m/s",cruiseSpeed);
    }

    ErrorCode::ErrorCodeType pauseWaypointMission(Vehicle* vehicle , int timeout) {
        ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->pause(timeout);
        if(ret != ErrorCode::SysCommonErr::Success)
        {
            DERROR("Pause waypoint v2 mission ErrorCode:0x%lX", ret);
            ErrorCode::printErrorCodeMsg(ret);
            return ret;
        }
        else
        {
            DSTATUS("Pause waypoint v2 mission successfully!");
        }
        sleep(5);
        return ret;
    }

    ErrorCode::ErrorCodeType resumeWaypointMission(Vehicle* vehicle , int timeout) {
        ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->resume(timeout);
        if(ret != ErrorCode::SysCommonErr::Success)
        {
        DERROR("Resume Waypoint v2 mission ErrorCode:0x%lX", ret);
        ErrorCode::printErrorCodeMsg(ret);
        return ret;
        }
        else
        {
            DSTATUS("Resume Waypoint v2 mission successfully!");
        }
        return ret;
    }   

bool teardownSubscription1(DJI::OSDK::Vehicle* vehicle ,const int pkgIndex, int timeout) {
  ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
  if (ACK::getError(ack)) {
    DERROR(
        "Error unsubscription; please restart the drone/FC to get back "
        "to a clean state.");
    return false;
  }
  return true;
}



E_OsdkStat updateMissionState(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                              const uint8_t *cmdData, void *userData) {

  if (cmdInfo) {
    if (userData) {
      auto *wp2Ptr = (WaypointV2MissionOperator *)userData;
      auto *missionStatePushAck =
        (DJI::OSDK::MissionStatePushAck *)cmdData;

      wp2Ptr->setCurrentState(wp2Ptr->getCurrentState());
      wp2Ptr->setCurrentState((DJI::OSDK::DJIWaypointV2MissionState)missionStatePushAck->data.state);
      static uint32_t curMs = 0;
      static uint32_t preMs = 0;
      OsdkOsal_GetTimeMs(&curMs);
      if (curMs - preMs >= 1000)
      {
        preMs = curMs;
        DSTATUS("missionStatePushAck->commonDataVersion:%d",missionStatePushAck->commonDataVersion);
        DSTATUS("missionStatePushAck->commonDataLen:%d",missionStatePushAck->commonDataLen);
        DSTATUS("missionStatePushAck->data.state:0x%x",missionStatePushAck->data.state);
        DSTATUS("missionStatePushAck->data.curWaypointIndex:%d",missionStatePushAck->data.curWaypointIndex);
        DSTATUS("missionStatePushAck->data.velocity:%d",missionStatePushAck->data.velocity);
      }
    } else {
      DERROR("cmdInfo is a null value");
    }
    return OSDK_STAT_OK;
  }
  return OSDK_STAT_ERR_ALLOC;
}