#ifndef INITIALSOLUTION_H
#define INITIALSOLUTION_H

#include <vector>
#include <set>
#include <crtdefs.h>
#include <random>

class WaterStationManager;
class TruckManager;

class InitialSolution
{
public:
    InitialSolution(const TruckManager& t, const WaterStationManager& m);
    void init();
    void end();
    std::vector<std::vector<std::vector<size_t> > > tabu();
    void aco( const std::vector<std::vector<std::vector<size_t> > >& schedule_solutions);
    void tsp( const std::vector<std::vector<std::vector<size_t> > >& schedule_solutions) const;
    void near_by( const std::vector<std::vector<std::vector<size_t> > >& schedule_solutions) const;
    void compute_real_cost() const;

private:
    void _dump_sequence( const std::vector<std::vector<std::vector<size_t> > >& schedule_pathes ) const;
    double _compute_solution_cost(size_t truck_idx, const std::vector<size_t>& stations) const;
    double _compute_all_solution_cost(const std::vector<std::vector<size_t> >& truck_stations) const;
    size_t _get_min_distance(size_t source_idx, const std::set<size_t>& targets) const;
    std::vector<size_t> _get_nearby_solution(const std::vector<size_t>& truck_stations) const;
    void _add_missing_stations(const std::vector<bool>& visited,
                         std::vector<std::vector<size_t> >& truck_stations ) const;
    bool _get_max_distance(size_t source_idx, const std::vector<bool>& visited, size_t &max_idx) const;
    bool _get_min_distance_within_load(size_t source_idx, double load, const std::vector<bool>& visited, size_t& min_idx) const;
    bool _is_all_stations_visited(const std::vector<size_t>& stations, const std::vector<std::vector<size_t> >& truck_stations ) const;
    double _group_station(const std::vector<size_t>& stations, const std::vector<size_t>& trucks,
                     std::vector<std::vector<size_t> > &truck_stations) const;
    std::vector<std::vector<size_t> > _group_station(const std::vector<size_t>& stations, double &min_cost) const;
    void _compute_distance_cost_matrix(std::vector<std::vector<double> > &cost, double km_per_liter) const;
    void _compute_wage_cost_matrix(std::vector<std::vector<double> >& cost, int wage) const;
    void _compute_distance_cost_matrix();

    void _check_solution(const std::set<size_t>& ignored_stations, size_t truck_idx, const std::vector<size_t> &stations, std::vector<size_t>& removed_stations);
    void _check_solution(const std::set<size_t>& ignored_stations, const std::vector<std::vector<size_t> > &stations, std::vector<size_t>& removed_stations);
    bool _check_solution(const std::vector<std::vector<std::vector<size_t> > >& stations, std::set<size_t>& ignored_stations );

    void _change_start(size_t idx, const std::vector<int> &station_start, std::vector<int> &new_station_start);
    void _change_start(const std::set<size_t>& ignored_stations, const std::vector<int> &station_start, std::vector<int> &new_station_start);
    bool _change_start(const std::set<std::vector<int> >& tabu,
                       const std::set<size_t>& ignored_stations,
                       const std::vector<int>& station_start,
                       std::vector<int>& new_station_start);

    double _get_schedule_solutions( const std::vector<std::vector<size_t> >& schedule,
                                    std::vector<std::vector<std::vector<size_t> > >& schedule_solutions) const;

    std::vector<int> _get_random_station_start();

    std::vector<std::vector<double>> _create_pheromone_matrix() const;
    std::vector<std::vector<double>> _create_value_matrix(const std::vector<std::vector<double> > &pheromone_matrix) const;
    size_t _develop(const std::vector<std::vector<double>>& value_matrix, const std::vector<bool> &visited,
                     size_t source_idx,
                     const std::vector<size_t>& stations) const;
    size_t _explore(const std::vector<std::vector<double>>& value_matrix, const std::vector<bool> &visited,
                     size_t source_idx,
                     const std::vector<size_t>& stations);
    bool _is_develop();
    void _local_update_pheromone(const std::vector<size_t> &stations, int q, double Lgb, double Pbest, size_t N, size_t source_idx, size_t target_idx, std::vector<double>& pheromone) const;
    double _get_L(const std::vector<size_t>& stations) const;
    std::vector<size_t> _aco(size_t truck_idx, const std::vector<size_t>& stations);

    size_t _get_max_pheromone_idx(const std::vector<std::vector<double>>& pheromone_matrix,
                                            const std::vector<size_t>& stations,
                                            const std::vector<bool>& visited,
                                            size_t source_idx) const;
    std::vector<size_t> _max_pheromone(const std::vector<std::vector<double>>& pheromone_matrix,
                                                    const std::vector<size_t>& stations) const;
    void _disturb_pheromone(const std::vector<size_t>& stations,
                                         std::vector<std::vector<double>>& pheromone_matrix);
    bool _local_search(size_t truck_idx, std::vector<size_t>& stations) const;

    std::vector<std::vector<std::vector<size_t> > > _create_real_schedule() const;

    std::vector<size_t> _tsp(size_t truck_idx, const std::vector<size_t>& stations,
                                                                  double& min_cost) const;

    const TruckManager& _t;
    const WaterStationManager& _m;
    std::vector<std::vector<std::vector<double> > > _distance_cost_matrix;
    std::vector<std::vector<std::vector<double> > > _wage_cost_matrix;
    std::default_random_engine _tabu_gen;
    std::default_random_engine _aco_gen;
    std::default_random_engine _random_start_gen;
    std::default_random_engine _change_start_gen;
    std::uniform_real_distribution<double> _dis;
    long long int _seconds;
};

#endif // INITIALSOLUTION_H
