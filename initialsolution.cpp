#include "initialsolution.h"
#include "waterstationmanager.h"
#include "truckmanager.h"

static const double g_diesel_price = 26.3;
static const double g_water_price  = 17;

InitialSolution::InitialSolution(const TruckManager &t, const WaterStationManager &m) :
    _t(t),
    _m(m),
    _cost()
{
}

void InitialSolution::_compute_cost(std::vector<std::vector<double> >& cost, double km_per_liter)
{
    size_t size = _m.get_station_size();

    cost.clear();
    cost.resize(size, std::vector<double>(size, 0.0));

    for(size_t i = 0; i < size; ++i)
    {
        const WaterStation& source = _m.get_station(i);
        (void) source;
        for(size_t j = 0; j < size; ++j)
        {
            const WaterStation& target = _m.get_station(j);
            cost[i][j] = target.supply * g_water_price + ((double)_m.get_distance(i, j)) / 1000 / km_per_liter * g_diesel_price;
        }
    }
}

void InitialSolution::_compute_cost()
{
    size_t truck_size = _t.get_truck_size();
    _cost.clear();
    _cost.resize(truck_size);

    for(size_t i = 0; i < truck_size; ++i)
    {
        const Truck& truck = _t.get_truck(i);
        _compute_cost(_cost.at(i), truck.km_per_liter);
    }
}

void InitialSolution::_generate(const std::vector<size_t>& stations)
{
    (void) stations;
}

void InitialSolution::generate()
{
    _compute_cost();
}
