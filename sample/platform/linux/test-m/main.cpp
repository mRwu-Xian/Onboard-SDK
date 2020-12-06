#include <iostream>
#include "flight_Status.hpp"
#include "collect_Waypoints.hpp"
#include "wayPoint_Mission.hpp"
#include "dji_vehicle.hpp"
#include "set_UpSubscription.hpp"
#include <dji_linux_helpers.hpp>
using namespace DJI;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int writeStreamData(const char *fileName, const uint8_t *data, uint32_t len) {
  FILE *fp = NULL;
  size_t size = 0;  

  fp = fopen(fileName, "a+");
  if(fp == NULL) {
    printf("fopen failed!\n");
    return -1;
  }
  size = fwrite(data, 1, len, fp);
  if(size != len) {
    return -1;
  }

  fflush(fp);
  if(fp) {
    fclose(fp);
  }
  return 0;
}

void liveViewSampleCb(uint8_t* buf, int bufLen, void* userData) {
  if (userData) {
    const char *filename = (const char *) userData;
    writeStreamData(filename, buf, bufLen);
  } else {
  DERROR("userData is a null value (should be a file name to log h264 stream).");
  }
}



const int DEFAULT_PACKAGE_INDEX = 0;
std::vector<WaypointV2> generatedWaypints;
std::vector<DJIWaypointV2Action> actions;


int main( int argc , char** argv){

    char h264FileName[50] = {0};
    struct tm tm;
    bool enableAdvancedSensing = true;
    LinuxSetup linuxEnvironment(argc, argv,enableAdvancedSensing);
    Vehicle* vehicle = linuxEnvironment.getVehicle();
    std::thread *Monitorflightstatusthread;
    if(vehicle == NULL)
    {
        std::cout << "Vehicle not initialized , exiting.\n";
        return -1;
    }

    Subscription *sub = new Subscription;
    //--------------------------------------------订阅
    sub->setUpSubscription(vehicle);
    //--------------------------------------------
    vehicle->control->obtainCtrlAuthority();
    //---------------------------------------------- 起飞以及home点的设置
    MoveTakeoff(vehicle);

    new std::thread([&]() {
        UVA_status *UVAone = new UVA_status;
        UVA_status::Point position;
        TypeMap<TOPIC_RTK_POSITION>::type  RTK_POSITION;
        RTK_POSITION = vehicle->subscribe->getValue<TOPIC_RTK_POSITION>();
        position.lat = RTK_POSITION.latitude;
        position.lon = RTK_POSITION.longitude;
        position.alt = RTK_POSITION.HFSL;
        UVAone->judge_Flightstatus(position); 
      });
      
    sleep(5);

    setNewHomeLocation(vehicle , 1); 

    setGoHomeAltitude(vehicle , 30 , 1);
    //------------------------------------------------------------
    //---------------------------------------------- 飞行（采集经纬坐标）并且移动云台
    MoveFlight(vehicle ,  (Vector3f){ 0 , 0 , 20} , 0 , 0.8 , 1.0 ); 
    sprintf(h264FileName, "FPV_%d-%d-%d_%d-%d-%d.h264", tm.tm_year + 1900,
                tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    vehicle->advancedSensing->startH264Stream(LiveView::OSDK_CAMERA_POSITION_FPV,
                                           liveViewSampleCb,
                                           (void *) h264FileName);
  
    MoveFlight(vehicle ,  (Vector3f){20 , 0 , 20} , 0 , 0.8 , 1.0 );

    MoveGimbal(vehicle , 1 , 30.0f );

    sprintf(h264FileName, "FPV_%d-%d-%d_%d-%d-%d.h264", tm.tm_year + 1900,
                tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

    vehicle->advancedSensing->startH264Stream(LiveView::OSDK_CAMERA_POSITION_FPV,
                                           liveViewSampleCb,
                                           (void *) h264FileName);

    // sprintf(h264FileName, "FPV_%d-%d-%d_%d-%d-%d.h264", tm.tm_year + 1900,
    //     tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    // vehicle->advancedSensing->startH264Stream(LiveView::OSDK_CAMERA_POSITION_FPV,
    //         liveViewSampleCb,
    //         (void *) h264FileName);


    sleep(4);

    MoveFlight(vehicle ,  (Vector3f){ 0 , 20 , 20}, 0 , 0.8 , 1.0 );

    MoveGimbal(vehicle , 1 , -30.0f );
    sprintf(h264FileName, "FPV_%d-%d-%d_%d-%d-%d.h264", tm.tm_year + 1900,
                tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    vehicle->advancedSensing->startH264Stream(LiveView::OSDK_CAMERA_POSITION_FPV,
                                           liveViewSampleCb,
                                           (void *) h264FileName);
    //----------------------------------------------- 回到home点
    goHomeAndConfirmLanding(vehicle , 1);

    sleep(80);
    //------------------------------------------------路点任务
    runWaypointV2Mission(vehicle);
    //-------------------------------------------------------
    //------------------------------------------------取消订阅
    sub->teardownSubscription(vehicle);
    //-------------------------------------------------------

}




