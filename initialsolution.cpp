#include "initialsolution.h"
#include "waterstationmanager.h"
#include "truckmanager.h"
#include <tgmath.h>
#include <time.h>
#include <chrono>

static const size_t g_max_group_iteration = 20;
static const double g_diesel_price = 26.3;
static const double g_water_price  = 17;
static const double g_M            = 1000;

static const int minutes_per_station = 5;
static const int minutes_per_ton     = 3;

static const int totoal_ant_iteration = 5;
static const int totoal_ant_num = 5;
static const int alpha = 1;
static const int beta  = 2;
static const int Q     = 100;
static const double rho = 0.6;
static const double rho_prime = 0.7;
static const double q0 = 0.7;
static const unsigned int tabu_random_seed = 0;
static const unsigned int aco_random_seed = 0;
static const unsigned int random_start_random_seed = 0;
static const unsigned int change_start_random_seed = 0;
static const int QQ = 1;

static const bool dump_to_file = false;

static FILE* _fp = nullptr;

void _open_log_file()
{
    if(!dump_to_file)
    {
        return;
    }
    char filename[40];
    struct tm *timenow;

    time_t now = time(NULL);
    timenow = localtime(&now);

    strftime(filename, sizeof(filename), "log_%Y_%m_%d_%H_%M_%S.txt", timenow);

    _fp = fopen(filename, "w");

    if(_fp == nullptr)
    {
        printf("failed to open file\n");
    }
}

void _close_log_file()
{
    if(nullptr == _fp)
    {
        return;
    }
    fclose(_fp);
}

void _log(const char* format, ...)
{
    va_list argptr;
    va_start(argptr, format);
    vfprintf(stdout, format, argptr);
    if(_fp)
    {
        vfprintf(_fp, format, argptr);
    }
    va_end(argptr);
}

InitialSolution::InitialSolution(const TruckManager &t, const WaterStationManager &m) :
    _t(t),
    _m(m),
    _distance_cost_matrix(),
    _wage_cost_matrix(),
    _tabu_gen(std::default_random_engine(tabu_random_seed)),
    _aco_gen(std::default_random_engine(aco_random_seed)),
    _random_start_gen(std::default_random_engine(random_start_random_seed)),
    _change_start_gen(std::default_random_engine(change_start_random_seed)),
    _dis(0, 1.0),
    _seconds(0)
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
//        _log("%zu truck %zu -> %zu cost %f\n", truck_idx, source_idx, target_idx, cost );
        all_cost += cost;
        source_idx = target_idx;
    }

//    _log("cost = %f = %f + penalty %f\n", all_cost + penalty, all_cost, penalty);
    return all_cost + penalty;
}

double InitialSolution::_compute_all_solution_cost(const std::vector<std::vector<size_t> >& truck_stations) const
{
    double all_cost = 0.0;
    for(size_t i = 0; i < truck_stations.size(); ++i)
    {
        double cost = _compute_solution_cost(i, truck_stations[i]);
//        _log("truck %zu cost %f\n", i, cost);
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
//        _log("finished\n");
        return true;
    }
    if(s < stations.size())
    {
//        _log("not finished\n");
        return false;
    }
    _log("check solution error\n");
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
//        _log("Run %zu th truck %zu (%f ton)\n", truck_seq_idx, truck_idx, truck.load);
        size_t source_idx = 0;
        size_t target_idx = std::numeric_limits<size_t>::max();
        if(!_get_max_distance(source_idx, visited, target_idx))
        {
//            _log("unable to get max distance station, stop\n");
            break;
        }
//        _log("max distance station %zu\n", target_idx);
        double load = truck.load;
        while(load > 0)
        {
            const WaterStation& water_station = _m.get_station(target_idx);
            if(load < water_station.supply)
            {
//                _log("run out of load %f < %f, next truck\n", load, water_station.supply);
                break;
            }
//            _log("visit station from %zu to %zu, load %f = %f - %f\n", source_idx, target_idx, load - water_station.supply,
//                   load, water_station.supply);
            load -= water_station.supply;
            visited.at(target_idx) = true;
            truck_stations.at(truck_idx).push_back(target_idx);

            source_idx = target_idx;
            if(!_get_min_distance_within_load(source_idx, load, visited, target_idx ))
            {
//                _log("unable to get min distance station, next truck\n");
                break;
            }
//            _log("min distance station %zu\n", target_idx);
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

void InitialSolution::_check_solution(const std::set<size_t>& ignored_stations,
                                      size_t truck_idx, const
                                      std::vector<size_t>& stations,
                                      std::vector<size_t>& removed_stations)
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
//        double r01 = (double)(rand()) / (RAND_MAX + 1);
        double r01 = _dis(_tabu_gen);
        double rnd = r01 * (stations.size());
        size_t selected_idx = (size_t) rnd;
        size_t station_idx = stations.at(selected_idx);
        if(removed_set.find(station_idx) != removed_set.end())
        {
            continue;
        }
//        _log("truck load %f < %f, remove station %zu\n", truck.load, load, station_idx);

        const WaterStation& water_station = _m.get_station(station_idx);
        load -= water_station.supply;

        removed_stations.push_back(station_idx);
        removed_set.insert(station_idx);
    }
}

void InitialSolution::_check_solution(const std::set<size_t>& ignored_stations,
                                      const std::vector<std::vector<size_t> >& stations,
                                      std::vector<size_t>& removed_stations)
{
    removed_stations.clear();
    for(size_t i = 0; i < stations.size(); ++i)
    {
        _check_solution(ignored_stations, i, stations[i], removed_stations);
    }
}

bool InitialSolution::_check_solution(const std::vector<std::vector<std::vector<size_t> > >& stations,
                                      std::set<size_t>& ignored_stations )
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
                                    std::vector<int>& new_station_start)
{
//    double r01 = (double)(rand()) / (RAND_MAX + 1);
    double r01 = _dis(_change_start_gen);
    if(r01 > 0.5)
    {
        _log("Probility %f > 0.5, don't change start\n", r01);
        new_station_start = station_start;
        return true;
    }

    const size_t max_count = 20;
    size_t count = 0;
    while (count < max_count) {
        _log("Run change station iteration %zu\n", count);
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
                                    )
{
    const WaterStation& water_station = _m.get_station(idx);
    int start = station_start.at(idx);

    while(true)
    {
//        double r01 = (double)(rand()) / (RAND_MAX + 1);
        double r01 = _dis(_change_start_gen);
        double rnd = r01 * water_station.cycle;
        if(start != (int) rnd)
        {
//            _log("change station %zu start from %d to %d\n", idx, start, (int) rnd);
            start = (int) rnd;
            break;
        }
    }
    new_station_start.at(idx) = start;
}

void InitialSolution::_change_start(const std::set<size_t>& ignored_stations,
                                    const std::vector<int>& station_start,
                                    std::vector<int>& new_station_start
                                    )
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
    _open_log_file();
    _compute_distance_cost_matrix();
}

void InitialSolution::end()
{
    _close_log_file();
}

std::vector<int> InitialSolution::_get_random_station_start()
{
//    srand(random_start_random_seed);
    std::vector<int> station_start;
    station_start.reserve(_m.get_station_size());

    for(size_t i = 0; i < _m.get_station_size(); ++i)
    {
//        double r01 = (double)(rand()) / (RAND_MAX + 1);
        double r01 = _dis(_random_start_gen);
        double rnd = r01 * (_m.get_schedule_size());
        station_start.push_back((size_t) rnd);
    }

    return station_start;
}

std::vector<std::vector<std::vector<size_t> > > InitialSolution::tabu()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    double min_cost = std::numeric_limits<double>::max();
    std::vector<int> min_station_start;

    std::set<std::vector<int>> tabu;

//    std::vector<int> station_start = _get_random_station_start();
    std::vector<int> station_start = _m.get_station_start();

//    srand(tabu_random_seed);
    size_t count = 0;
    while(count < g_max_group_iteration )
    {
        _log("Run iteration %zu ", count);
        ++count;

        tabu.insert(station_start);

        std::vector<std::vector<size_t> > schedule = _m.get_schedule(station_start);

        std::vector<std::vector<std::vector<size_t> > > schedule_solutions;
        double cost = _get_schedule_solutions( schedule, schedule_solutions);

        _log("cost = %f\n", cost);
        if(cost < min_cost)
        {
            _log("Update min cost from %f to %f\n", min_cost, cost);
            min_cost = cost;
            min_station_start = station_start;
        }
        else
        {
            station_start = min_station_start;
        }

        std::set<size_t> ignored_stations;
        if(_check_solution(schedule_solutions, ignored_stations))
        {
            _log("All stations are satisfied. Finished\n");
            break;
        }
        std::vector<int> updated_station_start;
        if(!_change_start( tabu, ignored_stations, station_start, updated_station_start))
        {
            _log("Unable to find new station start. Exit\n");
            break;
        }
        station_start = updated_station_start;
    }
    if(g_max_group_iteration == count)
    {
        _log("Finish iteration due to reach max count %zu\n", g_max_group_iteration);
    }

    std::vector<std::vector<size_t> > schedule = _m.get_schedule(min_station_start);
    std::vector<std::vector<std::vector<size_t> > > schedule_solutions;
    _get_schedule_solutions( schedule, schedule_solutions);

    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();

    _seconds += std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();

    return schedule_solutions;
}

std::vector<std::vector<double>> InitialSolution::_create_pheromone_matrix() const
{
    std::vector<std::vector<double>> pheromone_matrix;
    size_t size = _m.get_station_size();

    pheromone_matrix.resize(size);
    for(size_t i = 0; i < size; ++i)
    {
        pheromone_matrix[i].resize(size);
        for(size_t j = 0; j < size; ++j)
        {
            pheromone_matrix[i][j] = std::max(0.0, _m.get_expected_value(i, j));
        }
    }

    return pheromone_matrix;
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
        _log("error, empty stations\n");
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
                                  const std::vector<size_t>& stations)
{
    if(stations.empty())
    {
        _log("error, empty stations\n");
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
//    double rf = rand() / (RAND_MAX + 1.0);
    double rf = _dis(_aco_gen);
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

bool InitialSolution::_is_develop()
{
//    double rf = rand() / (RAND_MAX + 1.0);
    double rf = _dis(_aco_gen);

//    if(rf < q0)
//    {
//        _log("rf %f < q0 %f, select develop\n", rf, q0);
//    }
//    else
//    {
//        _log("rf %f >= q0 %f, select explore\n", rf, q0);
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
                                               int q,
                                               double Lgb,
                                               double Pbest,
                                               size_t N,
                                               size_t source_idx, size_t target_idx,
                                               std::vector<double>& pheromone) const
{
    double tau_max = 1 / (rho * Lgb);
    double p = std::pow(Pbest, 1 / (double)N);
    double avg = (double)N / 2;
    if(N <= 2)
    {
        avg = 2;
    }
    double tau_min = tau_max * p / ((avg - 1) * p);

    for(size_t i = 0; i <stations.size(); ++i)
    {
        size_t idx = stations[i];
        if(source_idx == idx)
        {
            continue;
        }
        if(target_idx == idx)
        {
            pheromone[idx] = pheromone[idx] * (1 - rho) + rho * (q / Lgb);
        }
        else
        {
            pheromone[idx] = pheromone[idx] * (1 - rho);
        }

        pheromone[idx] = std::min(tau_max, pheromone[idx]);
        pheromone[idx] = std::max(tau_min, pheromone[idx]);
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

void InitialSolution::_disturb_pheromone(const std::vector<size_t>& stations,
                                         std::vector<std::vector<double>>& pheromone_matrix)
{
    for(size_t i = 0; i <stations.size(); ++i)
    {
        size_t source_idx = stations[i];
        for(size_t j = 0; j <stations.size(); ++j)
        {
            size_t target_idx = stations[j];
            if(source_idx == target_idx)
            {
                continue;
            }
//            double r01 = (double)(rand()) / (RAND_MAX + 1);
            double r01 = _dis(_aco_gen);
            if(r01 > 0.5)
            {
                continue;
            }
//            double r02 = (double)(rand()) / (RAND_MAX + 1);
            double r02 = _dis(_aco_gen);
            pheromone_matrix[source_idx][target_idx] *= r02;
        }
    }
}

std::vector<size_t> InitialSolution::_aco( size_t truck_idx, const std::vector<size_t>& stations)
{
    if(stations.empty())
    {
        _log("error, empty stations\n");
        return std::vector<size_t>();
    }
//    double L = _get_L(stations);
    size_t N = stations.size() + 1;
    (void) N;
//    double tau0 = 1 / (L * N);

//    std::vector<std::vector<double>> pheromone_matrix = std::vector<std::vector<double>>(_m.get_station_size(), std::vector<double>(_m.get_station_size(), tau0));
    std::vector<std::vector<double>> pheromone_matrix = _create_pheromone_matrix();
    size_t ant_iteration = 0;
    size_t non_improve_iteration = 0;
    double iteration_min_cost = std::numeric_limits<double>::max();
    std::vector<size_t> iteration_min_path;
    while(ant_iteration < totoal_ant_iteration)
    {
        ++ant_iteration;
        size_t ant_count = 0;
        double count_min_cost = std::numeric_limits<double>::max();
        std::vector<size_t> count_min_path;
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
                source_idx = target_idx;
            }

           double cost = _compute_solution_cost(truck_idx, path);
           if(cost < count_min_cost)
           {
               count_min_cost = cost;
               count_min_path = path;
           }
        }

        bool improved = false;
        if(count_min_cost < iteration_min_cost)
        {
            iteration_min_cost = count_min_cost;
            iteration_min_path = count_min_path;
            improved = true;
        }

        improved |= _local_search( truck_idx, iteration_min_path);

        if(!improved)
        {
            ++non_improve_iteration;
        }

        if(non_improve_iteration > 0.15 * totoal_ant_iteration)
        {
            non_improve_iteration = 0;
            _disturb_pheromone( stations, pheromone_matrix);
        }
        else
        {
            double Lgb   = _get_L(iteration_min_path);
            double Pbest = _get_L(count_min_path);
            size_t source_idx = 0;
            for(size_t i = 0; i < iteration_min_path.size(); ++i)
            {
                size_t target_idx = iteration_min_path[i];
                _local_update_pheromone(stations, QQ,
                                        Lgb,
                                        Pbest,
                                        N,
                                        source_idx, target_idx, pheromone_matrix[source_idx]);
                source_idx = target_idx;
            }
        }
    }

    std::vector<size_t> path = _max_pheromone( pheromone_matrix, stations);

    return path;
}

bool InitialSolution::_local_search(size_t truck_idx, std::vector<size_t>& stations) const
{
    if(stations.size() < 2)
    {
        return false;
    }

    double cost = _compute_solution_cost(truck_idx, stations);

    std::vector<size_t> shuffle(stations.size());
    for(size_t i = 0; i < shuffle.size(); ++i)
    {
        shuffle[i] = 0;
    }

    std::random_shuffle(shuffle.begin(), shuffle.end());

    std::vector<size_t> tmp_stations = stations;
    std::swap(tmp_stations.at(shuffle.at(0)), tmp_stations.at(shuffle.at(1)));

    double tmp_cost = _compute_solution_cost(truck_idx, stations);

    if(tmp_cost < cost)
    {
        stations = tmp_stations;
        _log("update local search truck %zu cost %f -> %f\n", truck_idx, cost, tmp_cost);
        return true;
    }
    return false;
}

void InitialSolution::aco( const std::vector<std::vector<std::vector<size_t> > >& schedule_solutions)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

//    srand(aco_random_seed);

    std::vector<std::vector<std::vector<size_t> > > schedule_pathes;
    schedule_pathes.resize(schedule_solutions.size());

    _log("start aco\n");
    for(size_t i = 0; i < schedule_solutions.size(); ++i)
    {
        for(size_t j = 0; j < schedule_solutions[i].size(); ++j)
        {
            if(schedule_solutions[i][j].empty())
            {
                continue;
            }
            std::vector<size_t> path = _aco( j, schedule_solutions[i][j]);
            schedule_pathes[i].push_back(path);
        }
    }

    double total_cost = 0.0;
    for(size_t i = 0; i < schedule_solutions.size(); ++i)
    {
        for(size_t j = 0; j < schedule_solutions[i].size(); ++j)
        {
            double cost = _compute_solution_cost(j, schedule_solutions[i][j]);
            _log("day %zu truck %zu cost %f\n", i, j, cost);
            total_cost += cost;
        }
    }
    _log("total cost = %f\n", total_cost);

    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();

    _seconds += std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();
    printf( "Tabu + ACO time difference = %lld seconds\n", _seconds );
}

std::vector<std::vector<std::vector<size_t> > > InitialSolution::_create_real_schedule() const
{
    std::vector<std::vector<std::vector<size_t> > > schedule_solutions;
    schedule_solutions.resize(60);
    schedule_solutions[0].push_back(std::vector<size_t>());
    schedule_solutions[0].push_back({{ 34, 35, 37, 38, 2, 3, 1 }});
    schedule_solutions[0].push_back({{ 85, 92 }});

    schedule_solutions[1].push_back(std::vector<size_t>());
    schedule_solutions[1].push_back({{ 11, 10, 79, 26 }});
    schedule_solutions[1].push_back({{ 88, 105 }});

    schedule_solutions[2].push_back(std::vector<size_t>());
    schedule_solutions[2].push_back({{  }});
    schedule_solutions[2].push_back({{ 89, 93, 91 }});

    schedule_solutions[3].push_back(std::vector<size_t>());
    schedule_solutions[3].push_back({{ 80, 22, 23, 20 }});
    schedule_solutions[3].push_back({{ 94, 95, 96, 97 }});

    schedule_solutions[4].push_back(std::vector<size_t>());
    schedule_solutions[4].push_back({{ 49, 47, 49, 13, 64 }});
    schedule_solutions[4].push_back({{ 99, 101, 82, 83, 84 }});

    schedule_solutions[5].push_back(std::vector<size_t>());
    schedule_solutions[5].push_back({{ 72, 15 }});
    schedule_solutions[5].push_back({{ 110 }});

    schedule_solutions[6].push_back(std::vector<size_t>());
    schedule_solutions[6].push_back({{ 80, 71, 4, 5, 7, 8 }});
    schedule_solutions[6].push_back({{  }});

    schedule_solutions[7].push_back(std::vector<size_t>());
    schedule_solutions[7].push_back({{ 11, 2, 3, 1, 21, 67 }});
    schedule_solutions[7].push_back({{ 85, 86, 87, 82, 114 }});

    schedule_solutions[8].push_back(std::vector<size_t>());
    schedule_solutions[8].push_back({{ 66, 22, 12, 44 }});
    schedule_solutions[8].push_back({{ 106, 103, 104, 100, 98 }});

    schedule_solutions[9].push_back(std::vector<size_t>());
    schedule_solutions[9].push_back({{  }});
    schedule_solutions[9].push_back({{ 88, 105, 92, 82, 83 }});

    schedule_solutions[10].push_back(std::vector<size_t>());
    schedule_solutions[10].push_back({{ 53, 32, 52 }});
    schedule_solutions[10].push_back({{ 107, 108, 109 }});

    schedule_solutions[11].push_back(std::vector<size_t>());
    schedule_solutions[11].push_back({{ 80, 30, 29, 6, 19 }});
    schedule_solutions[11].push_back({{ 94, 95 }});

    schedule_solutions[12].push_back(std::vector<size_t>());
    schedule_solutions[12].push_back({{ 39, 23, 42, 24 }});
    schedule_solutions[12].push_back({{ 99, 101 }});

    schedule_solutions[13].push_back(std::vector<size_t>());
    schedule_solutions[13].push_back({{ 4, 5 }});
    schedule_solutions[13].push_back({{  }});

    schedule_solutions[14].push_back(std::vector<size_t>());
    schedule_solutions[14].push_back({{ 11, 10, 18, 33, 36, 21, 22 }});
    schedule_solutions[14].push_back({{ 89, 85, 82, 114 }});

    schedule_solutions[15].push_back(std::vector<size_t>());
    schedule_solutions[15].push_back({{  }});
    schedule_solutions[15].push_back({{  }});

    schedule_solutions[16].push_back(std::vector<size_t>());
    schedule_solutions[16].push_back({{  }});
    schedule_solutions[16].push_back({{ 88, 105 }});

    schedule_solutions[17].push_back(std::vector<size_t>());
    schedule_solutions[17].push_back({{ 80, 20, 2, 3, 1 }});
    schedule_solutions[17].push_back({{ 108, 91, 92, 93 }});

    schedule_solutions[18].push_back(std::vector<size_t>());
    schedule_solutions[18].push_back({{ 43, 65, 34, 35, 37, 38 }});
    schedule_solutions[18].push_back({{ 82, 83, 110 }});

    schedule_solutions[19].push_back(std::vector<size_t>());
    schedule_solutions[19].push_back({{ 55, 56, 57, 50, 27, 15, 16, 13 }});
    schedule_solutions[19].push_back({{ 99, 100, 101 }});

    schedule_solutions[20].push_back(std::vector<size_t>());
    schedule_solutions[20].push_back({{ 80, 22, 23, 4, 5 }});
    schedule_solutions[20].push_back({{  }});

    schedule_solutions[21].push_back(std::vector<size_t>());
    schedule_solutions[21].push_back({{ 54, 45, 46, 7, 8, 12, 81 }});
    schedule_solutions[21].push_back({{ 115, 114 }});

    schedule_solutions[22].push_back(std::vector<size_t>());
    schedule_solutions[22].push_back({{ 11, 26 }});
    schedule_solutions[22].push_back({{ 113, 105, 85, 86, 82 }});

    schedule_solutions[23].push_back(std::vector<size_t>());
    schedule_solutions[23].push_back({{  }});
    schedule_solutions[23].push_back({{ 110, 112, 90, 92 }});

    schedule_solutions[24].push_back(std::vector<size_t>());
    schedule_solutions[24].push_back({{ 2, 3, 1, 21 }});
    schedule_solutions[24].push_back({{  88, 83, 94, 95, 96}});

    schedule_solutions[25].push_back(std::vector<size_t>());
    schedule_solutions[25].push_back({{ 30, 29, 6, 25, 28, 75, 77, 73, 74, 76 }});
    schedule_solutions[25].push_back({{ 89, 82, 83, 102 }});

    schedule_solutions[26].push_back(std::vector<size_t>());
    schedule_solutions[26].push_back({{ 80, 31, 58, 62, 69 }});
    schedule_solutions[26].push_back({{ 106, 103, 98 }});

    schedule_solutions[27].push_back(std::vector<size_t>());
    schedule_solutions[27].push_back({{ 78, 17, 22, 23, 4, 5 }});
    schedule_solutions[27].push_back({{  }});

    schedule_solutions[28].push_back(std::vector<size_t>());
    schedule_solutions[28].push_back({{ 72, 24, 11, 10, 79 }});
    schedule_solutions[28].push_back({{ 99, 100, 101, 114 }});

    schedule_solutions[29].push_back(std::vector<size_t>());
    schedule_solutions[29].push_back({{  }});
    schedule_solutions[29].push_back({{ 107, 111, 110 }});

    schedule_solutions[30].push_back(std::vector<size_t>());
    schedule_solutions[30].push_back({{  }});
    schedule_solutions[30].push_back({{ 85, 82, 105 }});

    schedule_solutions[31].push_back(std::vector<size_t>());
    schedule_solutions[31].push_back({{ 80, 7, 8, 59, 61, 9, 19 }});
    schedule_solutions[31].push_back({{ 83, 84, 87 }});

    schedule_solutions[32].push_back(std::vector<size_t>());
    schedule_solutions[32].push_back({{ 20, 2, 3, 1 }});
    schedule_solutions[32].push_back({{ 94, 95, 92, 115 }});

    schedule_solutions[33].push_back(std::vector<size_t>());
    schedule_solutions[33].push_back({{ 39, 40, 60, 18, 63, 21 }});
    schedule_solutions[33].push_back({{ 88, 82, 83 }});

    schedule_solutions[34].push_back(std::vector<size_t>());
    schedule_solutions[34].push_back({{ 80, 68, 70, 4, 5 }});
    schedule_solutions[34].push_back({{  }});

    schedule_solutions[35].push_back(std::vector<size_t>());
    schedule_solutions[35].push_back({{ 22, 23, 12, 44, 43, 42, 64 }});
    schedule_solutions[35].push_back({{ 91, 85, 114 }});

    schedule_solutions[36].push_back(std::vector<size_t>());
    schedule_solutions[36].push_back({{ 11, 49, 47, 48, 13 }});
    schedule_solutions[36].push_back({{ 99, 100, 101, 103, 89 }});

    schedule_solutions[37].push_back(std::vector<size_t>());
    schedule_solutions[37].push_back({{  }});
    schedule_solutions[37].push_back({{ 86, 105 }});

    schedule_solutions[38].push_back(std::vector<size_t>());
    schedule_solutions[38].push_back({{ 34, 35, 37, 38, 15, 14 }});
    schedule_solutions[38].push_back({{ 82, 83 }});

    schedule_solutions[39].push_back(std::vector<size_t>());
    schedule_solutions[39].push_back({{ 80, 30, 29, 6, 2, 3, 1 }});
    schedule_solutions[39].push_back({{ 106, 104, 98 }});

    schedule_solutions[40].push_back(std::vector<size_t>());
    schedule_solutions[40].push_back({{ 53, 32, 52, 7, 8, 17 }});
    schedule_solutions[40].push_back({{ 92, 94, 95 }});

    schedule_solutions[41].push_back(std::vector<size_t>());
    schedule_solutions[41].push_back({{ 4, 5, 36, 33, 21 }});
    schedule_solutions[41].push_back({{  }});

    schedule_solutions[42].push_back(std::vector<size_t>());
    schedule_solutions[42].push_back({{ 26, 11, 10, 22, 23, 24 }});
    schedule_solutions[42].push_back({{ 85, 88, 114 }});

    schedule_solutions[43].push_back(std::vector<size_t>());
    schedule_solutions[43].push_back({{ 56, 55, 50, 51, 57 }});
    schedule_solutions[43].push_back({{ 110, 82, 83, 105 }});

    schedule_solutions[44].push_back(std::vector<size_t>());
    schedule_solutions[44].push_back({{  }});
    schedule_solutions[44].push_back({{ 99, 100, 101 }});

    schedule_solutions[45].push_back(std::vector<size_t>());
    schedule_solutions[45].push_back({{ 80, 54, 45, 46, 21 }});
    schedule_solutions[45].push_back({{ 107, 108 }});

    schedule_solutions[46].push_back(std::vector<size_t>());
    schedule_solutions[46].push_back({{ 65, 2, 3, 1, 43, 20 }});
    schedule_solutions[46].push_back({{ 115, 113, 92, 93 }});

    schedule_solutions[47].push_back(std::vector<size_t>());
    schedule_solutions[47].push_back({{ 31, 58, 62, 18, 63 }});
    schedule_solutions[47].push_back({{ 89, 93, 91 }});

    schedule_solutions[48].push_back(std::vector<size_t>());
    schedule_solutions[48].push_back({{ 4, 5, 27, 12, 81 }});
    schedule_solutions[48].push_back({{  }});

    schedule_solutions[49].push_back(std::vector<size_t>());
    schedule_solutions[49].push_back({{ 7, 8, 17, 15, 16, 22, 23, 11 }});
    schedule_solutions[49].push_back({{ 88, 82, 83, 94, 95, 97 }});

    schedule_solutions[50].push_back(std::vector<size_t>());
    schedule_solutions[50].push_back({{ 72 }});
    schedule_solutions[50].push_back({{ 85, 86 }});

    schedule_solutions[51].push_back(std::vector<size_t>());
    schedule_solutions[51].push_back({{  }});
    schedule_solutions[51].push_back({{ 105, 83 }});

    schedule_solutions[52].push_back(std::vector<size_t>());
    schedule_solutions[52].push_back({{ 39, 13 }});
    schedule_solutions[52].push_back({{ 99, 100, 101 }});

    schedule_solutions[53].push_back(std::vector<size_t>());
    schedule_solutions[53].push_back({{ 80, 2, 3, 1, 41, 30, 29, 6, 25, 28 }});
    schedule_solutions[53].push_back({{ 112, 111, 110 }});

    schedule_solutions[54].push_back(std::vector<size_t>());
    schedule_solutions[54].push_back({{ 71, 4, 5, 21 }});
    schedule_solutions[54].push_back({{ 106, 103, 104, 98 }});

    schedule_solutions[55].push_back(std::vector<size_t>());
    schedule_solutions[55].push_back({{ 22, 23 }});
    schedule_solutions[55].push_back({{  }});

    schedule_solutions[56].push_back(std::vector<size_t>());
    schedule_solutions[56].push_back({{ 80 }});
    schedule_solutions[56].push_back({{ 114, 92, 89, 82 }});

    schedule_solutions[57].push_back(std::vector<size_t>());
    schedule_solutions[57].push_back({{  }});
    schedule_solutions[57].push_back({{ 88, 85, 115 }});

    schedule_solutions[58].push_back(std::vector<size_t>());
    schedule_solutions[58].push_back({{  }});
    schedule_solutions[58].push_back({{ 105, 82, 83 }});

    schedule_solutions[59].push_back(std::vector<size_t>());
    schedule_solutions[59].push_back({{ 34, 35, 37, 38, 7, 8 }});
    schedule_solutions[59].push_back({{ 94, 95, 96 }});

    return schedule_solutions;
}

void InitialSolution::compute_real_cost() const
{
    std::vector<std::vector<std::vector<size_t> > > schedule_solutions = _create_real_schedule();

    double cost = 0.0;
    for(size_t i = 0; i < schedule_solutions.size(); ++i)
    {
        cost += _compute_all_solution_cost(schedule_solutions[i]);
    }

    _log("real schedule cost is %f\n", cost);
}

std::vector<size_t> InitialSolution::_tsp(size_t truck_idx, const std::vector<size_t>& stations,
                                                                  double& min_cost) const
{
    std::vector<size_t> schedule = stations;
    std::vector<size_t> min_schedule;
    min_cost = std::numeric_limits<double>::max();
    do
    {
        double cost = _compute_solution_cost( truck_idx, schedule);
        if(min_cost > cost)
        {
            min_cost = cost;
            min_schedule = schedule;
        }
    } while(std::next_permutation(schedule.begin(), schedule.end()));

    return min_schedule;
}

void InitialSolution::tsp( const std::vector<std::vector<std::vector<size_t> > >& schedule_solutions) const
{
    std::vector<std::vector<std::vector<size_t> > > schedule_pathes;
    schedule_pathes.resize(schedule_solutions.size());

    _log("start tsp\n");
    double all_cost = 0.0;
    for(size_t i = 0; i < schedule_solutions.size(); ++i)
    {
        for(size_t j = 0; j < schedule_solutions[i].size(); ++j)
        {
            if(schedule_solutions[i][j].empty())
            {
                continue;
            }
            double cost = 0.0;
            std::vector<size_t> path = _tsp( j, schedule_solutions[i][j], cost);
            schedule_pathes[i].push_back(path);
            all_cost += cost;
        }
    }

    _log("tsp cost = %f\n", all_cost);
}

void InitialSolution::near_by( const std::vector<std::vector<std::vector<size_t> > >& schedule_solutions) const
{
    std::vector<std::vector<std::vector<size_t> > > schedule_pathes;
    schedule_pathes.resize(schedule_solutions.size());

    _log("start near by\n");
    double all_cost = 0.0;
    for(size_t i = 0; i < schedule_solutions.size(); ++i)
    {
        for(size_t j = 0; j < schedule_solutions[i].size(); ++j)
        {
            if(schedule_solutions[i][j].empty())
            {
                continue;
            }
            std::vector<size_t> path = _get_nearby_solution(schedule_solutions[i][j]);
            schedule_pathes[i].push_back(path);
            double cost = _compute_solution_cost(j, path);
            all_cost += cost;
        }
    }

    _log("near by cost = %f\n", all_cost);
}

