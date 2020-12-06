#include <iostream>
#include "dji_vehicle.hpp"



class Subscription{

    public:
        Subscription();
        ~Subscription();
        
        bool setUpSubscription(Vehicle* vehicle);
        bool teardownSubscription(Vehicle* vehicle);
};