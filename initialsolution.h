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
    void generate();
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
    std::vector<std::vector<size_t> > _group_station(const std::vector<size_t>& stations) const;
    void _compute_cost_matrix(std::vector<std::vector<double> > &cost, double km_per_liter) const;
    void _compute_wage_cost_matrix(std::vector<std::vector<double> >& cost, int wage) const;
    void _compute_cost_matrix();

    void _check_solution(const std::set<size_t>& ignored_stations, size_t truck_idx, std::vector<size_t>& stations, std::vector<size_t>& removed_stations) const;
    void _check_solution(const std::set<size_t>& ignored_stations, std::vector<std::vector<size_t> >& stations, std::vector<size_t>& removed_stations) const;
    void _check_solution(std::vector<std::vector<std::vector<size_t> > >& stations) const;

    void _change_start(size_t idx) const;
    void _change_start(const std::set<size_t>& ignored_stations) const;

    const TruckManager& _t;
    const WaterStationManager& _m;
    std::vector<std::vector<std::vector<double> > > _cost_matrix;
    std::vector<std::vector<std::vector<double> > > _wage_cost_matrix;
};

#endif // INITIALSOLUTION_H
