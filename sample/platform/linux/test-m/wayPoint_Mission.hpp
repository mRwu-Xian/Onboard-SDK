#include "dji_waypoint_v2.hpp"
#include "dji_waypoint_v2_action.hpp"
#include <dji_linux_helpers.hpp>



extern std::vector<WaypointV2> generatedWaypints;
extern std::vector<DJIWaypointV2Action> actions;
extern const int DEFAULT_PACKAGE_INDEX;


ErrorCode::ErrorCodeType runWaypointV2Mission(DJI::OSDK::Vehicle* vehicle );


std::vector<WaypointV2> generatePolygonWaypoints(DJI::OSDK::Vehicle* vehicle);
void setWaypointV2Defaults(WaypointV2& waypointV2);
std::vector<DJIWaypointV2Action> generateWaypointActions(uint16_t actionNum);

ErrorCode::ErrorCodeType WaypointV2MissionSample(DJI::OSDK::Vehicle* vehicle , int timeout);
ErrorCode::ErrorCodeType initMissionSetting(DJI::OSDK::Vehicle* vehicle ,int timeout);
ErrorCode::ErrorCodeType uploadWapointActions(DJI::OSDK::Vehicle* vehicle , int timeout);
ErrorCode::ErrorCodeType uploadWaypointMission(DJI::OSDK::Vehicle* vehicle , int timeout);
ErrorCode::ErrorCodeType startWaypointMission(DJI::OSDK::Vehicle* vehicle , int timeout);
ErrorCode::ErrorCodeType downloadWaypointMission(DJI::OSDK::Vehicle* vehicle , std::vector<WaypointV2> &mission,int timeout);

void setGlobalCruiseSpeed(DJI::OSDK::Vehicle* vehicle , const GlobalCruiseSpeed &cruiseSpeed, int timeout);
void getGlobalCruiseSpeed(DJI::OSDK::Vehicle* vehicle , int timeout);
bool setUpSubscription(int timeout);
ErrorCode::ErrorCodeType pauseWaypointMission(DJI::OSDK::Vehicle* vehicle , int timeout);
ErrorCode::ErrorCodeType resumeWaypointMission(DJI::OSDK::Vehicle* vehicle , int timeout);


bool teardownSubscription1( DJI::OSDK::Vehicle* vehicle, int pkgIndex,
                            int timeout);
