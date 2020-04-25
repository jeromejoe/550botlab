#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>

SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
    double scanScore = 0.0;
    double rayScore = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);

    for (const auto& ray : movingScan)
    {
        rayScore = scoreRay(ray, map);
        scanScore += rayScore;
    }

    return scanScore;
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
    Point<int> rayEnd;
    rayEnd.x = static_cast<int>(
        ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
    rayEnd.y = static_cast<int>(
        ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

    CellOdds tempOdds;
    tempOdds = map.logOdds(rayEnd.x, rayEnd.y);
    double Odds;
    Odds = static_cast<double>(tempOdds);
    // std::cout << int(tempOdds) << std::endl;
    
    // apply simplified likelihood field model
    if (Odds >= 100)
    {
        return 4*Odds;
    }
    else if (Odds >= 50)
    {
        return 2*Odds;
    }

    else if (Odds < 50)
    {
        //take one step closer to the origin
        Point<int> rayEnd1;
        rayEnd1.x = static_cast<int>(
            (ray.range - 1.0*map.metersPerCell()) * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayEnd1.y = static_cast<int>(
            (ray.range - 1.0*map.metersPerCell()) * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);
        double Odds1 = 0.0;
        if (map.isCellInGrid(rayEnd1.x, rayEnd1.y))
            Odds1 = static_cast<int>(map(rayEnd1.x, rayEnd1.y));

        if (Odds1 >= 100)
            return 1 * Odds1;
        else if (Odds1 >= 50)
            return 0.3*Odds;

        //take one step further and check if occupied there
        Point<int> rayEnd2;
        rayEnd2.x = static_cast<int>(
            (ray.range + 1.0*map.metersPerCell()) * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayEnd2.y = static_cast<int>(
            (ray.range + 1.0*map.metersPerCell()) * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);
        double Odds2 = 0.0;
        if (map.isCellInGrid(rayEnd2.x, rayEnd2.y))
            Odds2 = static_cast<int>(map(rayEnd2.x, rayEnd2.y));
        
        if (Odds2 >= 100)
            return 1 * Odds2;
        else if (Odds2 >= 50)
            return 0.3 * Odds2;
    }

    return 0.0;
}