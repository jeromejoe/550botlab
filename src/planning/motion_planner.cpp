#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <lcmtypes/robot_path_t.hpp>
#include <cmath>


MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, 
                                     const pose_xyt_t& goal, 
                                     const SearchParams& searchParams) const
{
    // If the goal isn't valid, then no path can actually exist
    if(!isValidGoal(goal))
    {
        robot_path_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);

        std::cout << "INFO: path rejected due to invalid goal\n";        

        return failedPath;
    }
    
    // Otherwise, use A* to find the path
    // std::cout << "!!!!Enter planPath" << std::endl;
    return search_for_path(start, goal, distances_, searchParams);
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, const pose_xyt_t& goal) const
{
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const pose_xyt_t& goal) const
{
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);
    // std::cout << "^^^^^^^^^^^^^^^^^^^" << std::endl;
    // std::cout << goal.x << ", " << goal.y << std::endl;
    // std::cout << prev_goal.x << ", " << prev_goal.y << std::endl;
    // std::cout << distanceFromPrev << std::endl;
    // std::cout << "^^^^^^^^^^^^^^^^^^^" << std::endl;
    // std::cout << "****distances_: " << distances_.widthInCells() << ", " << distances_.heightInCells() << std::endl;
    //if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    if(num_frontiers != 1 && distanceFromPrev < 2 * searchParams_.minDistanceToObstacle) return false;

    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);
    // std::cout << "***isCellInGrid: " << distances_.isCellInGrid(goalCell.x, goalCell.y) 
    //         << " --goalCell: " << goalCell.x << " ," << goalCell.y << std::endl;
    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        // std::cout << "?????????????????????????" << (distances_(goalCell.x, goalCell.y) > 0.25) << std::endl;
        return distances_(goalCell.x, goalCell.y) >= 0.25;
    }
    // std::cout << "$$$$$$$$$$$$$$$$$$$" << std::endl;
    // A goal must be in the map for the robot to reach it
    return false;
}


bool MotionPlanner::isPathSafe(const robot_path_t& path) const
{

    ///////////// TODO: Implement your test for a safe path here //////////////////

    for (auto& pose : path.path)
    {
        auto poseCell = global_position_to_grid_cell(Point<double>(pose.x, pose.y), distances_);
        if (distances_(poseCell.x, poseCell.y) <= 0.25)
        {
            return false;
        }
    }

    return true;
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    // searchParams_.minDistanceToObstacle = 2*params_.robotRadius;
    searchParams_.minDistanceToObstacle = 0.1; //0.1
    // searchParams_.maxDistanceWithCost = 10 * searchParams_.minDistanceToObstacle;
    searchParams_.maxDistanceWithCost = 0.8;  //0.35
    searchParams_.distanceCostExponent = 1;
}
