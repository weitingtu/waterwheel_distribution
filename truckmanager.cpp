#include "truckmanager.h"

TruckManager::TruckManager(): _trucks()
{
    _trucks.push_back(Truck(13.0, 3.2, 170));
    _trucks.push_back(Truck(19.0, 2.9, 200));
    _trucks.push_back(Truck(32.0, 2,   220));
}
