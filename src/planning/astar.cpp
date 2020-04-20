#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/angle_functions.hpp>
#include <queue>
#include <vector>

typedef Point<int> cell_t;

struct PathNode
{
    cell_t cell;
    cell_t parent;
    float gCost;
    float hCost;

    bool operator<(const PathNode& rhs) const
    {
        return (gCost + hCost) > (rhs.gCost + rhs.hCost);
    }

    bool operator==(const PathNode& rhs) const
    {
        return cell == rhs.cell;
    }

    bool operator!=(const PathNode& rhs) const
    {
        return cell != rhs.cell;
    }
};

bool isNodeInList(PathNode& node, std::vector<PathNode>& list);
PathNode getNodeFromList(PathNode& node, std::vector<PathNode>& list);
float calHCost(PathNode& node, PathNode& goal, const ObstacleDistanceGrid& grid);
void expandNode(PathNode &node,
                PathNode &goalNode,
                std::priority_queue<PathNode> &openList,
                std::vector<PathNode> &closedList,
                std::vector<PathNode> &searcheddList,
                const ObstacleDistanceGrid& distances,
                const SearchParams& params);
void extractPath(PathNode& node, 
                 PathNode& startNode,
                 std::vector<PathNode> &closedList,
                 robot_path_t& path,
                 const ObstacleDistanceGrid& grid);

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    robot_path_t path; 
    path.utime = start.utime;
    path.path.push_back(start);
    
    PathNode startNode, goalNode, currentNode;
    //construct start node
    cell_t startCell(static_cast<int>((start.x - distances.originInGlobalFrame().x) * distances.cellsPerMeter()),
                     static_cast<int>((start.y - distances.originInGlobalFrame().y) * distances.cellsPerMeter()));
    startNode.cell = startCell;
    //construct goal node
    cell_t goalCell(static_cast<int>((goal.x - distances.originInGlobalFrame().x) * distances.cellsPerMeter()),
                    static_cast<int>((goal.y - distances.originInGlobalFrame().y) * distances.cellsPerMeter()));
    goalNode.cell = goalCell;

    //construct lists and priority queue
    std::vector<PathNode> closedList, searchedList;
    std::priority_queue<PathNode> openList;
    openList.push(startNode);
    currentNode = openList.top();
    openList.pop();

    bool isPathFound = false;
    while (currentNode != goalNode)
    {
        closedList.push_back(currentNode);
        expandNode(currentNode, goalNode, openList, closedList, searchedList, distances, params);
        if (openList.empty())
        {
            break;
        }
        currentNode = openList.top();
        openList.pop();
        if (currentNode == goalNode)
        {
            isPathFound = true;
        }
    }
    if (isPathFound)
    {
        extractPath(currentNode, startNode, closedList, path, distances);
        //set start theta
        float dx1 = path.path[1].x - path.path[0].x;
        float dy1 = path.path[1].y - path.path[0].y;
        path.path[0].theta = wrap_to_pi(std::atan2(dy1, dx1));
        //set end theta
        float dx2 = goal.x - path.path.back().x;
        float dy2 = goal.y - path.path.back().y;
        path.path.back().theta = wrap_to_pi(std::atan2(dy2, dx2));
        path.path.push_back(goal);

        std::cout << "***************" << path.path.size() << std::endl;
        for (int j = 0; j<path.path.size(); j++)
            std::cout << path.path[j].x << " " << path.path[j].y << " " << path.path[j].theta << std::endl;
    }
    else
    {
        std::cout << "No path found!" << std::endl;
    }

    path.path_length = path.path.size();
    return path;
}

bool isNodeInList(PathNode& node, std::vector<PathNode>& list)
{
    for (auto& n : list)
    {
        if (n.cell == node.cell)
        {
            return true;
        }
    }
    return false;
}

PathNode getNodeFromList(PathNode& node, std::vector<PathNode>& list)
{
    if (isNodeInList(node, list))
    {   
        for (auto& n : list)
        {
            if (n.cell == node.cell)
            {
                return n;
            }
        }
    }
}

void expandNode(PathNode &node,
                PathNode &goalNode,
                std::priority_queue<PathNode> &openList,
                std::vector<PathNode> &closedList,
                std::vector<PathNode> &searchedList,
                const ObstacleDistanceGrid& distances,
                const SearchParams& params)
{
    int xDeltas[4] = {1, -1, 0, 0};
    int yDeltas[4] = {0, 0, 1, -1};
    for (int i = 0; i < 4; i++)
    {
        PathNode neighbor;
        neighbor.gCost = 0;
        neighbor.hCost = 0;
        neighbor.cell.x = node.cell.x + xDeltas[i];
        neighbor.cell.y = node.cell.y + yDeltas[i];
        if (isNodeInList(neighbor, searchedList))
        {
            neighbor = getNodeFromList(neighbor, searchedList);
        }
        if ((!isNodeInList(neighbor, closedList)) 
            && (distances(neighbor.cell.x, neighbor.cell.y) >= 3*params.minDistanceToObstacle)
            && distances.isCellInGrid(neighbor.cell.x, neighbor.cell.y))
        {
            if (!isNodeInList(neighbor, searchedList))
            {
                if (distances(neighbor.cell.x, neighbor.cell.y) >= params.maxDistanceWithCost)
                {
                    neighbor.gCost += distances.metersPerCell();
                }
                else
                {
                    neighbor.gCost = neighbor.gCost + distances.metersPerCell() + 10*abs(std::pow(params.maxDistanceWithCost 
                                                                                         - distances(neighbor.cell.x, neighbor.cell.y), 
                                                                                         params.distanceCostExponent));
                }
                neighbor.hCost = calHCost(neighbor, goalNode, distances);
                neighbor.parent = node.cell;
                openList.push(neighbor);
                searchedList.push_back(neighbor);
            }
            else
            {
                float newGCost;
                if (distances(neighbor.cell.x, neighbor.cell.y) >= params.maxDistanceWithCost)
                {
                    newGCost = neighbor.gCost + distances.metersPerCell();
                }
                else
                {
                    newGCost = neighbor.gCost + distances.metersPerCell() + 10*abs(std::pow(params.maxDistanceWithCost 
                                                                                    - distances(neighbor.cell.x, neighbor.cell.y), 
                                                                                    params.distanceCostExponent));
                }
                if (neighbor.gCost > newGCost)
                {
                    neighbor.gCost = newGCost;
                    neighbor.parent = node.cell;
                    openList.push(neighbor);
                }
            }
        }

    }
}

float calHCost(PathNode& node, PathNode& goal, const ObstacleDistanceGrid& grid)
{
    float dx = std::abs(node.cell.x - goal.cell.x);
    float dy = std::abs(node.cell.y - goal.cell.y);
    return grid.metersPerCell() * (dx + dy);
}

void extractPath(PathNode& node, 
                 PathNode& startNode,
                 std::vector<PathNode> &closedList,
                 robot_path_t& path,
                 const ObstacleDistanceGrid& grid)
{
    PathNode curNode = node;
    std::vector<PathNode> tempList;

    while (curNode != startNode)
    {
        tempList.push_back(curNode);
        PathNode parentNode;
        parentNode.cell = curNode.parent;
        parentNode = getNodeFromList(parentNode, closedList);
        curNode = parentNode;
    }

    int n = tempList.size();
    pose_xyt_t nextPose, prevPose;
    prevPose.x = grid.originInGlobalFrame().x + startNode.cell.x*grid.metersPerCell();
    prevPose.y = grid.originInGlobalFrame().y + startNode.cell.y*grid.metersPerCell();
    for (int i = n-1; i >= 0; i--)
    {
        PathNode nextNode = tempList[i];
        
        nextPose.x = grid.originInGlobalFrame().x + nextNode.cell.x*grid.metersPerCell();
        nextPose.y = grid.originInGlobalFrame().y + nextNode.cell.y*grid.metersPerCell();
        float dx = nextPose.x - prevPose.x;
        float dy = nextPose.y - prevPose.y;
        
        prevPose.theta = wrap_to_pi(std::atan2(dy, dx));
        path.path.push_back(prevPose);
        prevPose = nextPose;
    }

}