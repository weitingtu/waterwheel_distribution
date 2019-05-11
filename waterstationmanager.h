#ifndef WATERSTATIONMANAGER_H
#define WATERSTATIONMANAGER_H

#include <QString>
#include <vector>
#include <tuple>

class WaterStation
{
public:
    WaterStation(int c, int s, int st):cycle(c), supply(s), start(st) {}

    int cycle;
    double supply;
    int start;
};

struct WaterStationDistance
{
    WaterStationDistance(int d, size_t i) : distance(d), idx(i) {}
    int distance;
    size_t idx;

    bool operator <(const WaterStationDistance& rhs) const
    {
        return std::tie(distance, idx) < std::tie(rhs.distance, rhs.idx);
    }
    bool operator >(const WaterStationDistance& rhs) const
    {
        return std::tie(distance, idx) > std::tie(rhs.distance, rhs.idx);
    }
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
    void schedule();

    size_t get_station_size() const { return _stations.size(); }
    const WaterStation& get_station(size_t i) const { return _stations.at(i); }
    int get_duration(size_t i, size_t j) const { return _duration_matrix.at(i).at(j); }
    int get_distance(size_t i, size_t j) const { return _distance_matrix.at(i).at(j); }
    const std::vector<WaterStationDistance>& get_min_distances(size_t i) const { return _min_distance_matrix.at(i); }
    const std::vector<WaterStationDistance>& get_max_distances(size_t i) const { return _max_distance_matrix.at(i); }
    size_t get_schedule_size() const { return _schedules.size(); }
    const std::vector<size_t> get_schedule(size_t i) const { return _schedules.at(i); }

    std::vector<int> get_station_start() const;
    std::vector<std::vector<size_t> > get_schedule(const std::vector<int>& station_start) const;

private:
    void _save_distance_cache();

    std::vector<WaterStation> _stations;
    std::vector<std::vector<int>> _duration_matrix;
    std::vector<std::vector<int>> _distance_matrix;
    std::vector<std::vector<WaterStationDistance>> _min_distance_matrix;
    std::vector<std::vector<WaterStationDistance>> _max_distance_matrix;
    std::vector<std::vector<size_t> > _schedules;
};

#endif // WATERSTATIONMANAGER_H
