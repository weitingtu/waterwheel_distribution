#ifndef INITIALSOLUTION_H
#define INITIALSOLUTION_H

#include <vector>
#include <crtdefs.h>

class WaterStationManager;
class TruckManager;

class InitialSolution
{
public:
    InitialSolution(const TruckManager& t, const WaterStationManager& m);
    void generate();
private:
    bool _get_max_distance(size_t source_idx, const std::vector<bool>& visited, size_t &max_idx) const;
    bool _get_min_3_distance_with_min_supply(size_t source_idx, const std::vector<bool>& visited , size_t &min_idx) const;
    double _generate(const std::vector<size_t>& stations, const std::vector<size_t>& trucks,
                     std::vector<std::vector<size_t> > &truck_stations);
    void _generate(const std::vector<size_t>& stations);
    void _compute_cost(std::vector<std::vector<double> > &cost, double km_per_liter);
    void _compute_cost();

    const TruckManager& _t;
    const WaterStationManager& _m;
    std::vector<std::vector<std::vector<double> > > _cost;
};

#endif // INITIALSOLUTION_H
