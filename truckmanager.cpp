#include "truckmanager.h"

TruckManager::TruckManager(): _trucks()
{
    _trucks.push_back(Truck(4.0, 5));
    _trucks.push_back(Truck(13.0, 3.5));
    _trucks.push_back(Truck(26.0, 2.5));
}
