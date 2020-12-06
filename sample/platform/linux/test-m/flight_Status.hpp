#include <iostream>
#include <chrono>
#include <atomic>
#include "dji_vehicle.hpp"
#include "dji_telemetry.hpp"
#include <vector>
#include <deque>
#include <thread>
#include <numeric>
#include <algorithm>
#include <math.h>
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
#define PI 3.141592657    

static inline float rad(float degree);

class UVA_status{

    public:

        UVA_status();
        ~UVA_status();
        struct Point
        {
            float64_t lat;
            float64_t lon;
            float64_t alt;
        };

        std::thread *FlightTrendJudgmentThread;
        //----------------------判断飞机静止或移动状态--------------------------
        //***判断飞机的飞行趋势
            bool judge_Flightstatus(Point Position);
        //---------------------------------------------------------------------

            // bool status_sample();
    private:


        //该标志位true代表在飞行过程中false代表在悬停状态下
        std::atomic<bool> CirclingFlag {false};
        //该标志位true代表从静止状态到移动状态，反之反之
        std::atomic<bool> FlightFlag   {false};
        int queueLimit = 5;
        //记录最后的悬停系统时间
        float32_t lastHoverTime = 0;


        std::vector<float> range;

        enum class ActionStatus{
            Static,
            Move,
        };
        struct ActionStatusInfo
        {
            ActionStatus preStatus = ActionStatus::Static;
            ActionStatus curStatus = ActionStatus::Static;
            int recordTimes = 0;
        };


    //--------------------------计算工具函数-------------------------------
    //获取基准点到各点的距离
        float get_Distance(Point one, Point two);
    //获取方差(描述在悬停状态下细微移动各点之间的波动大小)
        float get_Variance(vector<float> range);    
    //判断方差是否在可容忍范围之内
        bool judge_Positionstatus(Point point);
    //--------------------------------------------------------------------

};



