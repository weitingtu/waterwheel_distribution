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
    void _generate(const std::vector<size_t>& stations);
    void _compute_cost(std::vector<std::vector<double> > &cost, double km_per_liter);
    void _compute_cost();

    const TruckManager& _t;
    const WaterStationManager& _m;
    std::vector<std::vector<std::vector<double> > > _cost;
};

#endif // INITIALSOLUTION_H
