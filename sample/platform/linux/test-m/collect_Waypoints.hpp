#include "dji_flight_controller.hpp"
#include "dji_vehicle.hpp"
#include "dji_status.hpp"
#include "dji_telemetry.hpp"
#include <dji_subscription.hpp>
#include "dji_flight_actions_module.hpp"
#include "dji_flight_assistant_module.hpp"
#include "dji_flight_joystick_module.hpp"
#include <cmath>
#include "dji_status.hpp"
#include <dji_linux_helpers.hpp>
#include "dji_gimbal_manager.hpp"
#include <vector>
#include "dji_waypoint_v2.hpp"
#include "dji_waypoint_v2_action.hpp"
#include <dji_linux_helpers.hpp>
extern const int DEFAULT_PACKAGE_INDEX;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252
typedef struct Telemetry::Vector3f Vector3f;
extern vector<WaypointV2> generatedWaypints;

typedef struct Telemetry::Vector3f Vector3f;

bool MoveTakeoff( DJI::OSDK::Vehicle* vehicle , 
     int timeout = 1 );

bool MoveFlight(DJI::OSDK::Vehicle* vehicle , const Vector3f &offsetDesired , float yawDesiredInDeg,
                            float posThresholdInM = 0.8,
                            float yawThresholdInDeg = 1.0  );

bool MoveGimbal(DJI::OSDK::Vehicle* vehicle , int timeout = 1 , float pitch = 0);


static void  horizCommandLimit(float speedFactor, float& commandX, float& commandY);
                           

static Vector3f localOffsetFromGpsOffset(const Telemetry::GPSFused &target,
     const Telemetry::GPSFused &origin);

static Vector3f vector3FSub(const Vector3f &vectorA, const Vector3f &vectorB);

template <typename Type>

static int signOfData(Type type);

static float32_t vectorNorm(Vector3f v);



bool teardownSubscription( DJI::OSDK::Vehicle* vehicle, int pkgIndex,
                            int timeout);

static Vector3f quaternionToEulerAngle(const Telemetry::Quaternion &quat);

bool motorStartedCheck(DJI::OSDK::Vehicle* vehicle);   
bool takeOffInAirCheck(DJI::OSDK::Vehicle* vehicle);
bool takeoffFinishedCheck(DJI::OSDK::Vehicle* vehicle);


bool setUpSubscription(DJI::OSDK::Vehicle* vehicle ,  int pkgIndex, int freq,
                         Telemetry::TopicName topicList[], uint8_t topicSize,
                         int timeout = 1);

ErrorCode::ErrorCodeType setNewHomeLocation(DJI::OSDK::Vehicle* vehicle , int timeout);
bool getHomeLocation(Vehicle* vehicle , Telemetry::HomeLocationSetStatus &homeLocationSetStatus,
                       Telemetry::HomeLocationData &homeLocationInfo,
                       int responseTimeout);
          
//------------------------------------------------------
bool getHomePOINT(DJI::OSDK::Vehicle* vehicle,int responseTimeout);

bool goHomeAndConfirmLanding(DJI::OSDK::Vehicle* vehicle , int timeout = 1);


ErrorCode::ErrorCodeType setGoHomeAltitude(DJI::OSDK::Vehicle* vehicle , 
      FlightController::GoHomeHeight altitude, int timeout = 1);

