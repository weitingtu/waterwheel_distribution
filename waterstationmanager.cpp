#include "waterstationmanager.h"
#include <QFile>
#include <QByteArray>
#include <QVector>

WaterStationManager::WaterStationManager():
    _stations(),
    _duration_matrix(116, std::vector<int>(116, 0)),
    _distance_matrix(116, std::vector<int>(116, 0)),
    _min_distance_matrix(116, std::vector<WaterStationDistance>()),
    _max_distance_matrix(116, std::vector<WaterStationDistance>()),
    _schedules(60, std::vector<size_t>())
{
}

void WaterStationManager::clear()
{
    _stations.clear();
    _duration_matrix.clear();
    _distance_matrix.clear();
    _duration_matrix = std::vector<std::vector<int>>(116, std::vector<int>(116, 0));
    _distance_matrix = std::vector<std::vector<int>>(116, std::vector<int>(116, 0));
    _min_distance_matrix = std::vector<std::vector<WaterStationDistance>>(116, std::vector<WaterStationDistance>());
    _max_distance_matrix = std::vector<std::vector<WaterStationDistance>>(116, std::vector<WaterStationDistance>());
    _schedules.clear();
}

bool WaterStationManager::parse(const QString &file_name)
{
    QFile file(file_name);
    if (!file.open(QIODevice::ReadOnly))
    {
        printf("%s\n", file.errorString().toLocal8Bit().data());
        return false;
    }

    clear();

    int count = 0;
    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        ++count;
        if(count <= 2)
        {
            continue;
        }
        QList<QByteArray>   word_list = line.split(',');
        QVector<QByteArray> word_vector = word_list.toVector();
        // idx, id, name, cycle, supply, address, start, duration value, duration text, distance value, distance text, ...
        int station_idx = count - 3;
        int cycle  = 0;
        double supply = 0;
        int start  = 0;
        if(station_idx > 0)
        {
            cycle  = QString(word_vector.at(3)).toInt();
            supply = QString(word_vector.at(4)).toDouble();
            start  = QString(word_vector.at(6)).toInt() - 1; // start from day 0
        }

        WaterStation water_station(cycle, supply, start);
        _stations.push_back(water_station);

        if(static_cast<int>(_duration_matrix.size()) <= station_idx)
        {
            _duration_matrix.resize(station_idx + 1);
            _distance_matrix.resize(station_idx + 1);
        }

        std::vector<int>& duration_matrix = _duration_matrix[station_idx];
        std::vector<int>& distance_matrix = _distance_matrix[station_idx];

        int idx = 0;
        while(idx * 4 + 7 + 4 <= word_vector.size())
        {
            if(static_cast<int>(duration_matrix.size()) <= idx)
            {
                duration_matrix.resize(idx + 1);
                distance_matrix.resize(idx + 1);
            }
            int duration = QString(word_vector.at(idx * 4 + 7 )).toInt();
            int distance = QString(word_vector.at(idx * 4 + 7 + 2)).toInt();

            duration_matrix.at(idx) = duration;
            distance_matrix.at(idx) = distance;

            ++idx;
        }
    }

    _save_distance_cache();

    return true;
}

void WaterStationManager::schedule()
{
    _schedules.clear();
    _schedules.resize(60);
    // ignore 0th station
    for(size_t i = 1; i < _stations.size(); ++i)
    {
        const WaterStation& water_station = _stations[i];
        int start = water_station.start;
        while(start < 60)
        {
            _schedules[start].push_back(i);
            start += water_station.cycle;
        }
    }
}

std::vector<int> WaterStationManager::get_station_start() const
{
    std::vector<int> station_start;
    station_start.reserve(_stations.size());

    for(size_t i = 0; i < _stations.size(); ++i)
    {
        station_start.push_back(_stations.at(i).start);
    }

    return station_start;
}

std::vector<std::vector<size_t> > WaterStationManager::get_schedule(const std::vector<int>& station_start) const
{
    std::vector<std::vector<size_t> > schedules;
    schedules.resize(60);
    // ignore 0th station
    for(size_t i = 1; i < station_start.size(); ++i)
    {
        const WaterStation& water_station = _stations[i];
        int start = station_start[i];
        while(start < 60)
        {
            schedules[start].push_back(i);
            start += water_station.cycle;
        }
    }

    return schedules;
}

void WaterStationManager::_save_distance_cache()
{
    for(size_t i = 0; i < _stations.size(); ++i)
    {
        std::vector<WaterStationDistance>& min_dis = _min_distance_matrix.at(i);
        std::vector<WaterStationDistance>& max_dis = _max_distance_matrix.at(i);
        min_dis.reserve(_stations.size());
        max_dis.reserve(_stations.size());
        for(size_t j = 0; j < _stations.size(); ++j)
        {
            min_dis.push_back(WaterStationDistance(get_distance(i, j), j));
            max_dis.push_back(WaterStationDistance(get_distance(i, j), j));
        }
        std::sort(min_dis.begin(), min_dis.end());
        std::sort(max_dis.begin(), max_dis.end(), std::greater<WaterStationDistance>());
    }
}
