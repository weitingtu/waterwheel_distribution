#include "initialsolution.h"
#include "waterstationmanager.h"
#include "truckmanager.h"
#include <tgmath.h>

static const size_t g_max_group_iteration = 20;
static const double g_diesel_price = 26.3;
static const double g_water_price  = 17;
static const double g_M            = 1000;

static const int minutes_per_station = 5;
static const int minutes_per_ton     = 3;

static const int totoal_ant_num = 100;
static const int alpha = 1;
static const int beta  = 2;
static const int Q     = 100;
static const double rho = 0.6;
static const double rho_prime = 0.7;
static const double q0 = 0.7;

InitialSolution::InitialSolution(const TruckManager &t, const WaterStationManager &m) :
    _t(t),
    _m(m),
    _distance_cost_matrix(),
    _wage_cost_matrix()
{
}

void InitialSolution::_compute_distance_cost_matrix(std::vector<std::vector<double> >& cost, double km_per_liter) const
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

void InitialSolution::_compute_wage_cost_matrix(std::vector<std::vector<double> >& cost, int wage) const
{
    size_t size = _m.get_station_size();

    cost.clear();
    cost.resize(size, std::vector<double>(size, 0.0));

    for(size_t i = 0; i < size; ++i)
    {
        for(size_t j = 0; j < size; ++j)
        {
            const WaterStation& target = _m.get_station(j);
            double minutes = ((double) _m.get_duration(i, j) / 60)
                             + minutes_per_station
                             + minutes_per_ton * target.supply;
            cost[i][j] = minutes * wage;
        }
    }
}

void InitialSolution::_compute_distance_cost_matrix()
{
    size_t truck_size = _t.get_truck_size();
    _distance_cost_matrix.clear();
    _distance_cost_matrix.resize(truck_size);
    _wage_cost_matrix.clear();
    _wage_cost_matrix.resize(truck_size);

    for(size_t i = 0; i < truck_size; ++i)
    {
        const Truck& truck = _t.get_truck(i);
        _compute_distance_cost_matrix(_distance_cost_matrix.at(i), truck.km_per_liter);
        _compute_wage_cost_matrix(_wage_cost_matrix.at(i), truck.wage);
    }
}

double InitialSolution::_compute_solution_cost(size_t truck_idx, const std::vector<size_t>& stations) const
{
    double load = 0.0;
    for(size_t i = 0; i < stations.size(); ++i)
    {
        const WaterStation& water_station = _m.get_station(stations[i]);
        load += water_station.supply;
    }

    double penalty = 0.0;
    const Truck& truck = _t.get_truck(truck_idx);
    if(load > truck.load)
    {
        penalty = (load - truck.load) * g_M;
    }

    double all_cost = 0.0;
    size_t source_idx = 0;
    for(size_t i = 0; i < stations.size(); ++i)
    {
        size_t target_idx = stations[i];
        double cost = _distance_cost_matrix.at(truck_idx).at(source_idx).at(target_idx);
        cost += _wage_cost_matrix.at(truck_idx).at(source_idx).at(target_idx);
//        printf("%zu truck %zu -> %zu cost %f\n", truck_idx, source_idx, target_idx, cost );
        all_cost += cost;
        source_idx = target_idx;
    }

//    printf("cost = %f = %f + penalty %f\n", all_cost + penalty, all_cost, penalty);
    return all_cost + penalty;
}

double InitialSolution::_compute_all_solution_cost(const std::vector<std::vector<size_t> >& truck_stations) const
{
    double all_cost = 0.0;
    for(size_t i = 0; i < truck_stations.size(); ++i)
    {
        double cost = _compute_solution_cost(i, truck_stations[i]);
//        printf("truck %zu cost %f\n", i, cost);
        all_cost += cost;
    }

    return all_cost;
}

size_t InitialSolution::_get_min_distance(size_t source_idx, const std::set<size_t>& targets) const
{
    int min_distance = std::numeric_limits<int>::max();
    size_t min_idx = std::numeric_limits<size_t>::max();

    for(auto ite = targets.begin(); ite != targets.end(); ++ite)
    {
        size_t target_idx = *ite;
        int distance = _m.get_distance(source_idx, target_idx);
        if(distance < min_distance)
        {
            min_distance = distance;
            min_idx = target_idx;
        }
    }

    return min_idx;
}

std::vector<size_t> InitialSolution::_get_nearby_solution(const std::vector<size_t>& truck_stations) const
{
    std::vector<size_t> solution;
    std::set<size_t> targets(truck_stations.begin(), truck_stations.end());

    size_t source_idx = 0;
    while(!targets.empty())
    {
        size_t target_idx = _get_min_distance(source_idx, targets);
        solution.push_back(target_idx);
        targets.erase(target_idx);
        source_idx = target_idx;
    }

    return solution;
}

bool InitialSolution::_is_all_stations_visited(const std::vector<size_t>& stations,
                                      const std::vector<std::vector<size_t> >& truck_stations ) const
{
    size_t s = 0;
    for(const std::vector<size_t>&v : truck_stations)
    {
        s += v.size();
    }
    if(s == stations.size())
    {
//        printf("finished\n");
        return true;
    }
    if(s < stations.size())
    {
//        printf("not finished\n");
        return false;
    }
    printf("check solution error\n");
    return true;
}

void InitialSolution::_add_missing_stations( const std::vector<bool>& visited,
                                             std::vector<std::vector<size_t> >& truck_stations ) const
{
    std::vector<double> loads(3, 0.0);
    for(size_t i = 0; i < truck_stations.size(); ++i)
    {
        const std::vector<size_t>& t_stations = truck_stations[i];
        for(size_t j = 0; j < t_stations.size(); ++j)
        {
            const WaterStation& water_station = _m.get_station(t_stations[j]);
            loads[i] += water_station.supply;
        }
    }

    for(size_t i = 0; i < visited.size(); ++i)
    {
        if(visited[i])
        {
            continue;
        }
        double min_cost = std::numeric_limits<double>::max();
        size_t min_idx = std::numeric_limits<size_t>::max();
        for(size_t j = 0; j < loads.size(); ++j)
        {
            std::vector<size_t> solution = truck_stations[j];
            solution.push_back(i);
            solution = _get_nearby_solution(solution);

            double cost = _compute_solution_cost(j, solution);
            if(cost < min_cost)
            {
                min_cost = cost;
                min_idx = j;
            }
        }
        truck_stations.at(min_idx).push_back(i);
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

bool InitialSolution::_get_min_distance_within_load(size_t source_idx, double load, const std::vector<bool>& visited,
                                                          size_t& min_idx) const
{
    min_idx = std::numeric_limits<size_t>::max();
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
        const WaterStation& water_station = _m.get_station(d.idx);
        if(water_station.supply > load)
        {
            continue;
        }
        min_idx = d.idx;
        break;
    }
    return min_idx != std::numeric_limits<size_t>::max();
}

double InitialSolution::_group_station(const std::vector<size_t>& stations,
                                  const std::vector<size_t>& trucks,
                                  std::vector<std::vector<size_t> >& truck_stations) const
{
    truck_stations.clear();
    truck_stations.resize(trucks.size());
    std::vector<bool> visited(_m.get_station_size(), true);
    for(size_t i = 0; i < stations.size(); ++i)
    {
        visited.at(stations.at(i)) = false;
    }
    visited.at(0) = true;

    for(size_t truck_seq_idx = 0; truck_seq_idx < trucks.size(); ++truck_seq_idx)
    {
        size_t truck_idx = trucks.at(truck_seq_idx);
        const Truck& truck = _t.get_truck(truck_idx);
//        printf("Run %zu th truck %zu (%f ton)\n", truck_seq_idx, truck_idx, truck.load);
        size_t source_idx = 0;
        size_t target_idx = std::numeric_limits<size_t>::max();
        if(!_get_max_distance(source_idx, visited, target_idx))
        {
//            printf("unable to get max distance station, stop\n");
            break;
        }
//        printf("max distance station %zu\n", target_idx);
        double load = truck.load;
        while(load > 0)
        {
            const WaterStation& water_station = _m.get_station(target_idx);
            if(load < water_station.supply)
            {
//                printf("run out of load %f < %f, next truck\n", load, water_station.supply);
                break;
            }
//            printf("visit station from %zu to %zu, load %f = %f - %f\n", source_idx, target_idx, load - water_station.supply,
//                   load, water_station.supply);
            load -= water_station.supply;
            visited.at(target_idx) = true;
            truck_stations.at(truck_idx).push_back(target_idx);

            source_idx = target_idx;
            if(!_get_min_distance_within_load(source_idx, load, visited, target_idx ))
            {
//                printf("unable to get min distance station, next truck\n");
                break;
            }
//            printf("min distance station %zu\n", target_idx);
        }
    }

    if(!_is_all_stations_visited( stations, truck_stations ))
    {
        _add_missing_stations( visited, truck_stations );
    }

    for(size_t i = 0; i < truck_stations.size(); ++i)
    {
        truck_stations[i] = _get_nearby_solution(truck_stations[i]);
    }

    double cost = _compute_all_solution_cost(truck_stations);

    return cost;
}

std::vector<std::vector<size_t> > InitialSolution::_group_station(const std::vector<size_t>& stations,
                                                                  double& min_cost) const
{
    std::vector<size_t> trucks = {0, 1, 2};
    std::vector<std::vector<size_t> > min_truck_stations(trucks.size());
    min_cost = std::numeric_limits<double>::max();
    do
    {
        std::vector<std::vector<size_t> > truck_stations(trucks.size());
        double cost = _group_station( stations, trucks, truck_stations);
        if(min_cost > cost)
        {
            min_cost = cost;
            min_truck_stations = truck_stations;
        }
    } while(std::next_permutation(trucks.begin(), trucks.end()));

    return min_truck_stations;
}

void InitialSolution::_check_solution(const std::set<size_t>& ignored_stations, size_t truck_idx, const std::vector<size_t>& stations, std::vector<size_t>& removed_stations) const
{
    const Truck& truck = _t.get_truck(truck_idx);
    double load = 0.0;
    for(size_t i = 0; i < stations.size(); ++i)
    {
        if(ignored_stations.find(stations[i]) != ignored_stations.end())
        {
            continue;
        }
        const WaterStation& water_station = _m.get_station(stations[i]);
        load += water_station.supply;
    }

    std::set<size_t> removed_set;
    while(truck.load < load)
    {
        double r01 = (double)(rand()) / (RAND_MAX + 1);
        double rnd = r01 * (stations.size());
        size_t selected_idx = (size_t) rnd;
        size_t station_idx = stations.at(selected_idx);
        if(removed_set.find(station_idx) != removed_set.end())
        {
            continue;
        }
//        printf("truck load %f < %f, remove station %zu\n", truck.load, load, station_idx);

        const WaterStation& water_station = _m.get_station(station_idx);
        load -= water_station.supply;

        removed_stations.push_back(station_idx);
        removed_set.insert(station_idx);
    }
}

void InitialSolution::_check_solution(const std::set<size_t>& ignored_stations, const std::vector<std::vector<size_t> >& stations, std::vector<size_t>& removed_stations) const
{
    removed_stations.clear();
    for(size_t i = 0; i < stations.size(); ++i)
    {
        _check_solution(ignored_stations, i, stations[i], removed_stations);
    }
}

bool InitialSolution::_check_solution(const std::vector<std::vector<std::vector<size_t> > >& stations,
                                      std::set<size_t>& ignored_stations ) const
{
    ignored_stations.clear();
    for(size_t i = 0; i < stations.size(); ++i)
    {
        std::vector<size_t> removed_stations;
        _check_solution(ignored_stations, stations[i], removed_stations);

        for(size_t j = 0; j < removed_stations.size(); ++j)
        {
            ignored_stations.insert(removed_stations[j]);
        }
    }

    if(ignored_stations.empty())
    {
        return true;
    }

    return false;
}

bool InitialSolution::_change_start(const std::set<std::vector<int> >& tabu,
                                    const std::set<size_t>& ignored_stations,
                                    const std::vector<int>& station_start,
                                    std::vector<int>& new_station_start) const
{
    const size_t max_count = 20;
    size_t count = 0;
    while (count < max_count) {
        printf("Run change station iteration %zu\n", count);
        count++;
        new_station_start = station_start;
        _change_start(ignored_stations, station_start, new_station_start);
        if(tabu.find(new_station_start) == tabu.end())
        {
            break;
        }
    }

    if(count == max_count)
    {
        return false;
    }

    return true;
}

void InitialSolution::_change_start(size_t idx,
                                    const std::vector<int>& station_start,
                                    std::vector<int>& new_station_start
                                    ) const
{
    const WaterStation& water_station = _m.get_station(idx);
    int start = station_start.at(idx);

    while(true)
    {
        double r01 = (double)(rand()) / (RAND_MAX + 1);
        double rnd = r01 * water_station.cycle;
        if(start != (int) rnd)
        {
//            printf("change station %zu start from %d to %d\n", idx, start, (int) rnd);
            start = (int) rnd;
            break;
        }
    }
    new_station_start.at(idx) = start;
}

void InitialSolution::_change_start(const std::set<size_t>& ignored_stations,
                                    const std::vector<int>& station_start,
                                    std::vector<int>& new_station_start
                                    ) const
{
    for(size_t idx : ignored_stations)
    {
        _change_start(idx, station_start, new_station_start);
    }
}

double InitialSolution::_get_schedule_solutions(
        const std::vector<std::vector<size_t> >& schedule,
        std::vector<std::vector<std::vector<size_t> > >& schedule_solutions) const
{
    schedule_solutions.clear();
    double cost = 0.0;
    for(size_t i = 0; i < _m.get_schedule_size(); ++i)
    {
        double solution_cost = std::numeric_limits<double>::max();
        schedule_solutions.push_back( _group_station(schedule.at(i), solution_cost) );
        cost += solution_cost;
    }
    return cost;
}

void InitialSolution::init()
{
    srand(0);
    _compute_distance_cost_matrix();
}

std::vector<std::vector<std::vector<size_t> > > InitialSolution::tabu()
{
    double min_cost = std::numeric_limits<double>::max();
    std::vector<int> min_station_start;

    std::set<std::vector<int>> tabu;

    std::vector<int> station_start = _m.get_station_start();
    size_t count = 0;
    while(count < g_max_group_iteration )
    {
        printf("Run iteration %zu ", count);
        ++count;

        tabu.insert(station_start);

        std::vector<std::vector<size_t> > schedule = _m.get_schedule(station_start);

        std::vector<std::vector<std::vector<size_t> > > schedule_solutions;
        double cost = _get_schedule_solutions( schedule, schedule_solutions);

        printf("cost = %f\n", cost);
        if(cost < min_cost)
        {
            printf("Update min cost from %f to %f\n", min_cost, cost);
            min_cost = cost;
            min_station_start = station_start;
        }

        std::set<size_t> ignored_stations;
        if(_check_solution(schedule_solutions, ignored_stations))
        {
            printf("All stations are satisfied. Finished\n");
            break;
        }
        std::vector<int> updated_station_start;
        if(!_change_start( tabu, ignored_stations, station_start, updated_station_start))
        {
            printf("Unable to find new station start. Exit\n");
            break;
        }
        station_start = updated_station_start;
    }
    if(g_max_group_iteration == count)
    {
        printf("Finish iteration due to reach max count %zu\n", g_max_group_iteration);
    }

    std::vector<std::vector<size_t> > schedule = _m.get_schedule(station_start);
    std::vector<std::vector<std::vector<size_t> > > schedule_solutions;
    _get_schedule_solutions( schedule, schedule_solutions);

    return schedule_solutions;
}

std::vector<std::vector<double>> InitialSolution::_create_value_matrix(
        const std::vector<std::vector<double>>& pheromone_matrix ) const
{
    std::vector<std::vector<double>> value_matrix;
    size_t size = _m.get_station_size();

    value_matrix.resize(size);
    for(size_t i = 0; i < size; ++i)
    {
        value_matrix[i].resize(size);
        for(size_t j = 0; j < size; ++j)
        {
            if(_m.get_expected_value(i, j) > 0)
            {
                value_matrix[i][j] = std::pow(pheromone_matrix[i][j], alpha)
                        * std::pow(_m.get_expected_value(i, j), beta);
            }
            else
            {
                value_matrix[i][j] = 0.0;
            }
        }
    }

    return value_matrix;
}

size_t InitialSolution::_develop( const std::vector<std::vector<double>>& value_matrix,
                                  const std::vector<bool>& visited,
                                  size_t source_idx,
                                  const std::vector<size_t>& stations) const
{
    if(stations.empty())
    {
        printf("error, empty stations\n");
        return std::numeric_limits<size_t>::max();
    }

    size_t max_idx = std::numeric_limits<size_t>::max();
    double max_value = std::numeric_limits<double>::lowest();

    for(size_t i = 0; i <stations.size(); ++i)
    {
        size_t target_idx = stations[i];
        if(visited[target_idx])
        {
            continue;
        }
        if(source_idx == target_idx)
        {
            continue;
        }
        if(max_value < value_matrix[source_idx][target_idx])
        {
            max_idx = target_idx;
            max_value = value_matrix[source_idx][target_idx];
        }
    }

    return max_idx;
}

size_t InitialSolution::_explore( const std::vector<std::vector<double>>& value_matrix,
                                  const std::vector<bool>& visited,
                                  size_t source_idx,
                                  const std::vector<size_t>& stations) const
{
    if(stations.empty())
    {
        printf("error, empty stations\n");
        return std::numeric_limits<size_t>::max();
    }

    double sum = 0.0;
    for(size_t i = 0; i < stations.size(); ++i)
    {
        size_t target_idx = stations[i];
        if(visited[target_idx])
        {
            continue;
        }
        sum += value_matrix[source_idx][target_idx];
    }

    std::vector<double> prob(stations.size(), 0.0);
    for(size_t i = 0; i < stations.size(); ++i)
    {
        size_t target_idx = stations[i];
        if(visited[target_idx])
        {
            continue;
        }
        prob[i] = value_matrix[source_idx][target_idx] / sum;
    }

    std::vector<double> cp(stations.size(), 0.0);
    cp[0] = prob[0];
    for(size_t i = 1; i <prob.size(); ++i)
    {
        cp[i] = cp[i-1] + prob[i];
    }

    size_t idx = std::numeric_limits<size_t>::max();
    double rf = rand() / (RAND_MAX + 1.0);
    for(size_t i = 0; i < cp.size(); ++i)
    {
        if(rf < cp[i])
        {
            idx = stations[i];
            break;
        }
    }

    return idx;
}

bool InitialSolution::_is_develop() const
{
    double rf = rand() / (RAND_MAX + 1.0);

//    if(rf < q0)
//    {
//        printf("rf %f < q0 %f, select develop\n", rf, q0);
//    }
//    else
//    {
//        printf("rf %f >= q0 %f, select explore\n", rf, q0);
//    }
    return rf < q0;
}

double InitialSolution::_get_L(const std::vector<size_t>& stations) const
{
    double L = 0.0;
    size_t source_idx = 0;
    for(size_t i = 0; i < stations.size(); ++i)
    {
        size_t target_idx = stations[i];
        L += _m.get_distance(source_idx, target_idx);
        source_idx = target_idx;
    }
    L += _m.get_distance(source_idx, 0);

    return L / 1000;
}

void InitialSolution::_local_update_pheromone( const std::vector<size_t>& stations,
                                         double L,
                                         size_t source_idx, size_t target_idx,
                                         std::vector<double>& pheromone) const
{

    for(size_t i = 0; i <stations.size(); ++i)
    {
        size_t idx = stations[i];
        if(source_idx == idx)
        {
            continue;
        }
        if(target_idx == idx)
        {
            pheromone[idx] = pheromone[idx] * (1 - rho) + rho * (1 / L);
        }
        else
        {
            pheromone[idx] = pheromone[idx] * (1 - rho);

        }
    }
}

size_t InitialSolution::_get_max_pheromone_idx(const std::vector<std::vector<double>>& pheromone_matrix,
                                            const std::vector<size_t>& stations,
                                            const std::vector<bool>& visited,
                                            size_t source_idx) const
{
    double max = std::numeric_limits<double>::lowest();
    size_t max_idx = std::numeric_limits<size_t>::max();

    for(size_t i = 0; i < stations.size(); ++i)
    {
        size_t target_idx = stations[i];
        if(target_idx == source_idx)
        {
            continue;
        }
        if(visited.at(target_idx))
        {
            continue;
        }
        if(pheromone_matrix.at(source_idx).at(target_idx) > max)
        {
            max = pheromone_matrix.at(source_idx).at(target_idx);
            max_idx = target_idx;
        }
    }

    return max_idx;
}

std::vector<size_t> InitialSolution::_max_pheromone(const std::vector<std::vector<double>>& pheromone_matrix,
                                                    const std::vector<size_t>& stations) const
{
    std::vector<size_t> path;

    std::vector<bool> visited(_m.get_station_size(), true);
    for(size_t i = 0; i < stations.size(); ++i)
    {
        visited[stations[i]] = false;
    }
    visited[0] = true;

    size_t source_idx = 0;
    while(path.size() < stations.size())
    {
        size_t target_idx = _get_max_pheromone_idx(pheromone_matrix, stations, visited, source_idx);
        visited.at(target_idx) = true;
        path.push_back(target_idx);
        source_idx = target_idx;
    }

    return path;
}

std::vector<size_t> InitialSolution::_aco( const std::vector<size_t>& stations) const
{
    if(stations.empty())
    {
        printf("error, empty stations\n");
        return std::vector<size_t>();
    }
    double L = _get_L(stations);
    size_t N = stations.size() + 1;
    double tau0 = 1 / (L * N);

    std::vector<std::vector<double>> pheromone_matrix = std::vector<std::vector<double>>(_m.get_station_size(), std::vector<double>(_m.get_station_size(), tau0));
    size_t ant_count = 0;
    while(ant_count < totoal_ant_num)
    {
        ++ant_count;
        std::vector<size_t> path;
        std::vector<std::vector<double>> value_matrix = _create_value_matrix(pheromone_matrix);
        std::vector<bool> visited(_m.get_station_size(), true);
        for(size_t i = 0; i < stations.size(); ++i)
        {
            visited[stations[i]] = false;
        }
        visited[0] = true;

        size_t source_idx = 0;
        while(path.size() < stations.size())
        {
            size_t target_idx = _is_develop() ? _develop(value_matrix, visited, source_idx, stations)
                                            : _explore(value_matrix, visited, source_idx, stations);
            visited[target_idx] = true;
            path.push_back(target_idx);
            _local_update_pheromone(stations, L, source_idx, target_idx, pheromone_matrix[source_idx]);
            source_idx = target_idx;
        }
    }

    std::vector<size_t> path = _max_pheromone( pheromone_matrix, stations);

    return path;
}

void InitialSolution::aco( const std::vector<std::vector<std::vector<size_t> > >& schedule_solutions)
{
    std::vector<std::vector<std::vector<size_t> > > schedule_pathes;
    schedule_pathes.resize(schedule_solutions.size());

    for(size_t i = 0; i < schedule_solutions.size(); ++i)
    {
        for(size_t j = 0; j < schedule_solutions[i].size(); ++j)
        {
            if(schedule_solutions[i][j].empty())
            {
                continue;
            }
            std::vector<size_t> path = _aco( schedule_solutions[i][j]);
            schedule_pathes[i].push_back(path);
        }
    }
}
