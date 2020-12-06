#include "flight_Status.hpp"

static inline float rad(float degree)
{
    return PI * degree / 180.0;
}
// bool UVA_status::judge_Circlingstatus(DY::CloudDroneStatus droneStatus){
//     MonitorException::Point point;
//     point.lat = droneStatus.status.physicDroneStatus.dji_RTK.lat;
//     point.lon = droneStatus.status.physicDroneStatus.dji_RTK.lon;
//     point.alt = droneStatus.status.physicDroneStatus.dji_RTK.alt;
//     if (judge_Positionstatus(point))
//     {
//         if (!CirclingFlag.load())
//         {
//             lastHoverTime = std::chrono::system_clock::now();
//             CirclingFlag.store(true);
//         }
//         std::chrono::system_clock::duration focusDuration = std::chrono::system_clock::now() - lastHoverTime;
//         if (std::chrono::duration_cast<std::chrono::seconds>(focusDuration).count() > 120)
//         {
//             LOG(INFO) << "Aircraft has been hovered for 120s";
//             return true;
//         }
//     }
//     else
//     {
//         CirclingFlag.store(false);
//         lastHoverTime = std::chrono::system_clock::now();
//     }
// }

bool UVA_status::judge_Positionstatus(UVA_status::Point position){
    std::deque<UVA_status::Point> positionQueue;
    if (positionQueue.size() >= queueLimit)
    {
        positionQueue.pop_front();
    }
    positionQueue.push_back(position);

    //判断当前位置与之前位置相差的方差
    for (UVA_status::Point each : positionQueue)
    {
        float distance = get_Distance(each, positionQueue.back());   //计算出一个结果貌似是一个距离
        range.push_back(distance);
    }

    if (1 > get_Variance(range))   //小于1 我认为 应该是没有移动
    {
        return true;              
    }
    return false;

}

float UVA_status::get_Variance(vector<float> range)
{
    float sum = std::accumulate(std::begin(range), std::end(range), 0.0f);
    float mean = sum / range.size(); //均值

    float accum = 0.0f;
    std::for_each(std::begin(range), std::end(range), [&](const double d) {
        accum += (d - mean) * (d - mean);
    });

    float stdev = sqrt(accum / range.size()); //方差
    return stdev;
}

float UVA_status::get_Distance(UVA_status::Point one, UVA_status::Point two)
{
    float radLat1 = rad(one.lat);
    float radLat2 = rad(two.lat);
    float a = radLat1 - radLat2;
    float b = rad(one.lon) - rad(two.lon);
    float distance = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2)));
    distance = distance * EARTH_RADIUS;

    LOG(INFO) << "Waypoint mission distance :" << distance;
    return distance;
}



//--------------------------------------------------------------------



bool UVA_status::judge_Flightstatus(UVA_status::Point position){
    ActionStatus status;
    ActionStatusInfo statusInfo;
    if(judge_Positionstatus(position)){
        status = ActionStatus::Static;
        statusInfo.curStatus = ActionStatus::Static;
    }
    else
    {
        status = ActionStatus::Move;
        statusInfo.curStatus = ActionStatus::Move;
    }
    if (statusInfo.preStatus != statusInfo.curStatus)
    {
        if (statusInfo.curStatus == ActionStatus::Static)
        {
            // 从运动到静止
            FlightFlag.store(false);
            statusInfo.preStatus = statusInfo.curStatus;
            LOG(INFO) << "Flight Status from Move to static"
            return true;
        }
        else
        {   
            // 从静止到运动
            FlightFlag.store(true);
            statusInfo.preStatus = statusInfo.curStatus;
            LOG(INFO) << "Flight Status from static to Move"
            return true;
        }       
    }
    else
    {
        if(status == ActionStatus::Static)
        {
            LOG(INFO) << "The current state is" << status;
        }
        else
        {
            LOG(INFO) << "The current state is" << status;
        }
        
    }
    return false;
}

// bool UVA_status::status_sample(Vehicle* vehicle){
//     UVA_status::FlightTrendJudgmentThread = new std::thread([&]() {
//         UVA_status::Point position;
//         TypeMap<TOPIC_RTK_POSITION>::type  RTK_POSITION;
//         RTK_POSITION = vehicle->subscribe->getValue<TOPIC_RTK_POSITION>();
//         position.lat = RTK_POSITION.longitude;
//         position.lon = RTK_POSITION.latitude;
//         position.alt = RTK_POSITION.HFSL;
//         UVA_status::judge_Flightstatus(position); 
//     });  
