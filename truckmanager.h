#ifndef TRUCKMANAGER_H
#define TRUCKMANAGER_H

#include <vector>
#include <crtdefs.h>

struct Truck
{
    Truck(double l, double k) : load(l), km_per_liter(k) {}

    double load;
    double km_per_liter;
};

class TruckManager
{
public:
    static TruckManager& get_inst()
    {
        static TruckManager inst;
        return inst;
    }
public:
    TruckManager();

    size_t get_truck_size() const { return _trucks.size(); }
    const Truck& get_truck(size_t i) const { return _trucks.at(i); }
private:
    std::vector<Truck> _trucks;
};

#endif // TRUCKMANAGER_H
