#ifndef WATERSTATIONMANAGER_H
#define WATERSTATIONMANAGER_H

#include <QString>
#include <vector>

class WaterStation
{
public:
    WaterStation(int c, int s, int st):cycle(c), supply(s), start(st) {}

    int cycle;
    int supply;
    int start;
};

class WaterStationManager
{
public:
    static WaterStationManager& get_inst()
    {
        static WaterStationManager inst;
        return inst;
    }
public:
    WaterStationManager();

    bool parse(const QString& file_name);
    void clear();

private:
    std::vector<WaterStation> _stations;
    std::vector<std::vector<int>> _duration_matrix;
    std::vector<std::vector<int>> _distance_matrix;
};

#endif // WATERSTATIONMANAGER_H
