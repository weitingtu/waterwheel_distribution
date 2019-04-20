#include "waterstationmanager.h"
#include <QFile>
#include <QByteArray>
#include <QVector>

WaterStationManager::WaterStationManager():
    _stations(),
    _duration_matrix(116, std::vector<int>(116, 0)),
    _distance_matrix(116, std::vector<int>(116, 0))
{
}

void WaterStationManager::clear()
{
    _stations.clear();
    _duration_matrix.clear();
    _distance_matrix.clear();
    _duration_matrix = std::vector<std::vector<int>>(116, std::vector<int>(116, 0));
    _distance_matrix = std::vector<std::vector<int>>(116, std::vector<int>(116, 0));
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
        int supply = 0;
        int start  = 0;
        if(station_idx > 0)
        {
            cycle  = QString(word_vector.at(3)).toInt();
            supply = QString(word_vector.at(4)).toInt();
            start  = QString(word_vector.at(6)).toInt();
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

    return true;
}
