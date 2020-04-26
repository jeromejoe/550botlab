#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <common/grid_utils.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>
#include <algorithm>


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
robot_path_t path_to_frontier(const frontier_t& frontier, 
                              const pose_xyt_t& pose, 
                              const OccupancyGrid& map,
                              const MotionPlanner& planner);
pose_xyt_t nearest_navigable_cell(pose_xyt_t pose, 
                                  Point<float> desiredPosition, 
                                  const OccupancyGrid& map,
                                  const MotionPlanner& planner);
pose_xyt_t search_to_nearest_free_space(Point<float> position, const OccupancyGrid& map, const MotionPlanner& planner);
double path_length(const robot_path_t& path);


std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map, 
                                           const pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;
    
    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);
  
    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            
            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);
                
                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontiers;
}


robot_path_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers, 
                                   const pose_xyt_t& robotPose,
                                   const OccupancyGrid& map,
                                   const MotionPlanner& planner)
{
    ///////////// TODO: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */
    // ObstacleDistanceGrid obstacleDistances = planner.obstacleDistances();  // >0.25 to be a valid free space
    robot_path_t emptyPath;

    std::vector<Point<float>> frontierMidPoints;
    std::vector<float> disVec;
    //find a target point for each frontier
    for (auto& frontier : frontiers)
    {
        float sumX = 0.0, sumY = 0.0;
        for (auto& point : frontier.cells)
        {
            // float dx = point.x - robotPose.x;
            // float dy = point.y - robotPose.y;
            // float distance = std::sqrt(dx*dx + dy*dy);
            // distances.push_back(distance);
            sumX += point.x;
            sumY += point.y;
        }
        Point<float> midPoint;
        midPoint.x = sumX / frontier.cells.size();
        midPoint.y = sumY / frontier.cells.size();
        frontierMidPoints.push_back(midPoint);
        float dx = midPoint.x - robotPose.x;
        float dy = midPoint.y - robotPose.y;
        float distance = std::sqrt(dx*dx + dy*dy);
        disVec.push_back(distance);
        
    }
    int minElementIndex = std::min_element(disVec.begin(),disVec.end()) - disVec.begin(); // find closest frontier
    std::cout << "CCCCCCChosing frontier: " << minElementIndex << std::endl;
    std::vector<robot_path_t> pathes;
    for (auto& targetPoint : frontiers[minElementIndex].cells)
    {
        Point<int> targetCell = global_position_to_grid_cell(targetPoint, map);
        std::cout << "*******TTTTargetCell: " << targetCell << std::endl;
        
        // create a rectangle searching area to find free cell
        pose_xyt_t goal = robotPose;
        int recSize = 8;
        for (int i = recSize; i >= -recSize; i--)
        {
            for (int j = recSize; j >= -recSize; j--)
            {
                std::cout << "iiiiiijjjjjj  " << i << ". " << j << std::endl;
                Point<int> cell;
                cell.x = targetCell.x + j;
                cell.y = targetCell.y + i;
                Point<double> cellGlobalPos = grid_position_to_global_position(cell, map);
                goal.x = cellGlobalPos.x;
                goal.y = cellGlobalPos.y;
                robot_path_t path = planner.planPath(robotPose, goal);
                if (path.path_length < 2) continue;
                // goal cannot be too close
                float dx = path.path.back().x - robotPose.x;
                float dy = path.path.back().y - robotPose.y;
                float diss = std::sqrt(dx*dx + dy*dy);
                if (diss < 0.3) continue;
                std::cout << "~~~~~~~~~planner.isValidGoal(goal): " << planner.isValidGoal(goal) << "planner.isPathSafe(path): " << planner.isPathSafe(path) << std::endl;
                // if (planner.isValidGoal(goal) && planner.isPathSafe(path))
                // {
                //     return path;
                // }
                pathes.push_back(path);
            }
        }
    }
    /*
    robot_path_t maxPath;
    if (pathes.size() != 0)
    {
        maxPath = pathes[0];
        for (auto& p : pathes)
        {
            if (p.path_length > maxPath.path_length)
            {
                maxPath = p;
            }
        }
        return maxPath;
    }
    */
    robot_path_t minPath;
    std::cout << "----------------pathNum: " << pathes.size() << std::endl;
    if (pathes.size() != 0)
    {
        minPath = pathes[0];
        for (auto& p : pathes)
        {
            if (p.path_length < minPath.path_length)
            {
                minPath = p;
            }
        }
        return minPath;
    }
    
    return emptyPath;
}


bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell if a frontier if it has log-odds 0 and a neighbor has log-odds < 0
    
    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }
    
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }
    
    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);
    
    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };
 
    frontier_t frontier;
    
    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end()) 
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontier;
}