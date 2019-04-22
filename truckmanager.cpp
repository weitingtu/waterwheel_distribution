#include "truckmanager.h"

TruckManager::TruckManager(): _trucks()
{
    _trucks.push_back(Truck(4, 5));
    _trucks.push_back(Truck(13, 3.5));
    _trucks.push_back(Truck(26, 2.5));
}
