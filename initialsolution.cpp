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

bool InitialSolution::_get_max_distance(size_t source_idx, const std::vector<bool>& visited, size_t& max_idx) const
{
    max_idx = std::numeric_limits<size_t>::max();
    const std::vector<WaterStationDistance>& max_distances = _m.get_max_distances(source_idx);

    for(size_t i = 0; i < max_distances.size(); ++i)
    {
        const WaterStationDistance& d = max_distances.at(i);
        if(d.idx == source_idx)
        {
            continue;
        }
        if(visited.at(d.idx))
        {
            continue;
        }
        max_idx = d.idx;
        break;
    }

    return max_idx != std::numeric_limits<size_t>::max();
}

bool InitialSolution::_get_min_3_distance_with_min_supply(size_t source_idx, const std::vector<bool>& visited,
                                                          size_t& min_idx) const
{
    // find min supply station within 3 min nearby stations
    min_idx = std::numeric_limits<size_t>::max();
    double min_supply   = std::numeric_limits<int>::max();
    int min_distance = std::numeric_limits<int>::max();
    size_t count = 0;
    const std::vector<WaterStationDistance>& min_distances = _m.get_min_distances(source_idx);

    for(size_t i = 0; i < min_distances.size(); ++i)
    {
        const WaterStationDistance& d = min_distances.at(i);
        if(d.idx == source_idx)
        {
            continue;
        }
        if(visited.at(d.idx))
        {
            continue;
        }
        ++count;
        const WaterStation& water_station = _m.get_station(d.idx);

        if(min_supply > water_station.supply)
        {
            min_idx      = d.idx;
            min_supply   = water_station.supply;
            min_distance = d.distance;
        }
        else if(min_supply == water_station.supply && min_distance > d.distance)
        {
            min_idx      = d.idx;
            min_supply   = water_station.supply;
            min_distance = d.distance;
        }
        if(count == 3)
        {
            break;
        }
    }
    return min_idx != std::numeric_limits<size_t>::max();
}

double InitialSolution::_generate(const std::vector<size_t>& stations,
                                  const std::vector<size_t>& trucks,
                                  std::vector<std::vector<size_t> >& truck_stations)
{
    truck_stations.clear();
    truck_stations.resize(trucks.size());
    std::vector<bool> visited(_m.get_station_size(), true);
    for(size_t i = 0; i < stations.size(); ++i)
    {
        visited.at(stations.at(i)) = false;
    }
    visited.at(0) = true;

    double cost = 0.0;
    for(size_t truck_seq_idx = 0; truck_seq_idx < trucks.size(); ++truck_seq_idx)
    {
        size_t truck_idx = trucks.at(truck_seq_idx);
        const Truck& truck = _t.get_truck(truck_idx);
        printf("Run %zu th truck %zu (%f ton)\n", truck_seq_idx, truck_idx, truck.load);
        size_t source_idx = 0;
        size_t target_idx = std::numeric_limits<size_t>::max();
        if(!_get_max_distance(source_idx, visited, target_idx))
        {
            printf("unable to get max distance station, stop\n");
            break;
        }
        printf("max distance station %zu\n", target_idx);
        double load = truck.load;
        while(load > 0)
        {
            const WaterStation& water_station = _m.get_station(target_idx);
            if(load < water_station.supply)
            {
                printf("run out of load %f < %f, next truck\n", load, water_station.supply);
                break;
            }
            printf("visit station from %zu to %zu, load %f = %f - %f\n", source_idx, target_idx, load - water_station.supply,
                   load, water_station.supply);
            load -= water_station.supply;
            visited.at(target_idx) = true;
            cost += _cost.at(truck_idx).at(source_idx).at(target_idx);
            truck_stations.at(truck_idx).push_back(target_idx);

            source_idx = target_idx;
            if(!_get_min_3_distance_with_min_supply(source_idx, visited, target_idx ))
            {
                printf("unable to get min distance station, next truck\n");
                break;
            }
            printf("min distance station %zu\n", target_idx);
        }
    }

    return cost;
}

void InitialSolution::_generate(const std::vector<size_t>& stations)
{
    //std::vector<size_t> trucks = {0, 1, 2};
    std::vector<size_t> trucks = {1, 2, 0};
    std::vector<std::vector<size_t> > min_truck_stations(trucks.size(), std::vector<size_t>());
    double min_cost = std::numeric_limits<double>::max();
    do
    {
        std::vector<std::vector<size_t> > truck_stations(trucks.size(), std::vector<size_t>());
        double cost = _generate( stations, trucks, truck_stations);
        if(min_cost > cost)
        {
            min_cost = cost;
            min_truck_stations = truck_stations;
        }
        break;

    } while(std::next_permutation(trucks.begin(), trucks.end()));
}

void InitialSolution::generate()
{
    _compute_cost();
    for(size_t i = 0; i < _m.get_schedule_size(); ++i)
    {
        _generate(_m.get_schedule(i));
        break;
    }
}
