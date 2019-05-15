#ifndef INITIALSOLUTION_H
#define INITIALSOLUTION_H

#include <vector>
#include <set>
#include <crtdefs.h>

class WaterStationManager;
class TruckManager;

class InitialSolution
{
public:
    InitialSolution(const TruckManager& t, const WaterStationManager& m);
    void init();
    void generate();
    void aco();

private:
    double _compute_solution_cost(size_t truck_idx, const std::vector<size_t>& stations) const;
    double _compute_all_solution_cost(const std::vector<std::vector<size_t> >& truck_stations) const;
    size_t _get_min_distance(size_t source_idx, const std::set<size_t>& targets) const;
    std::vector<size_t> _get_nearby_solution(const std::vector<size_t>& truck_stations) const;
    void _add_missing_stations(const std::vector<bool>& visited,
                         std::vector<std::vector<size_t> >& truck_stations ) const;
    bool _get_max_distance(size_t source_idx, const std::vector<bool>& visited, size_t &max_idx) const;
    bool _get_min_distance_within_load(size_t source_idx, double load, const std::vector<bool>& visited, size_t& min_idx) const;
    bool _get_min_3_distance_with_min_supply(size_t source_idx, const std::vector<bool>& visited , size_t &min_idx) const;
    bool _is_all_solution_visited(const std::vector<size_t>& stations, const std::vector<std::vector<size_t> >& truck_stations ) const;
    double _group_station(const std::vector<size_t>& stations, const std::vector<size_t>& trucks,
                     std::vector<std::vector<size_t> > &truck_stations) const;
    std::vector<std::vector<size_t> > _group_station(const std::vector<size_t>& stations, double &min_cost) const;
    void _compute_distance_cost_matrix(std::vector<std::vector<double> > &cost, double km_per_liter) const;
    void _compute_wage_cost_matrix(std::vector<std::vector<double> >& cost, int wage) const;
    void _compute_distance_cost_matrix();

    void _check_solution(const std::set<size_t>& ignored_stations, size_t truck_idx, const std::vector<size_t> &stations, std::vector<size_t>& removed_stations) const;
    void _check_solution(const std::set<size_t>& ignored_stations, const std::vector<std::vector<size_t> > &stations, std::vector<size_t>& removed_stations) const;
    bool _check_solution(const std::vector<std::vector<std::vector<size_t> > >& stations, std::set<size_t>& ignored_stations ) const;

    void _change_start(size_t idx, const std::vector<int> &station_start, std::vector<int> &new_station_start) const;
    void _change_start(const std::set<size_t>& ignored_stations, const std::vector<int> &station_start, std::vector<int> &new_station_start) const;
    bool _change_start(const std::set<std::vector<int> >& tabu,
                       const std::set<size_t>& ignored_stations,
                       const std::vector<int>& station_start,
                       std::vector<int>& new_station_start) const;

    std::vector<std::vector<double>> _create_value_matrix(const std::vector<std::vector<double> > &pheromone_matrix) const;
    size_t _develop(const std::vector<std::vector<double>>& value_matrix, const std::vector<size_t> &visited,
                     size_t source_idx,
                     const std::vector<size_t>& stations) const;
    size_t _explore(const std::vector<std::vector<double>>& value_matrix, const std::vector<size_t> &visited,
                     size_t source_idx,
                     const std::vector<size_t>& stations) const;
    bool _is_develop() const;
    void _local_update_pheromone(const std::vector<size_t> &stations, double L, size_t source_idx, size_t target_idx, std::vector<double>& pheromone) const;
    double _get_L(const std::vector<size_t>& stations) const;
    std::vector<size_t> _aco( const std::vector<size_t>& stations) const;

    const TruckManager& _t;
    const WaterStationManager& _m;
    std::vector<std::vector<std::vector<double> > > _distance_cost_matrix;
    std::vector<std::vector<std::vector<double> > > _wage_cost_matrix;
};

#endif // INITIALSOLUTION_H
