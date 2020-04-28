#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/angle_functions.hpp>
#include <queue>
#include <vector>
#include <set>
#include <algorithm>

typedef Point<int> cell_t;

struct PathNode
{
    cell_t cell;
    cell_t parent;
    float gCost;
    float hCost;

    bool operator<(const PathNode& rhs) const
    {
        return (gCost + hCost) < (rhs.gCost + rhs.hCost);
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
                std::set<PathNode> &openList,
                std::vector<PathNode> &closedList,
                std::vector<PathNode> &searcheddList,
                const ObstacleDistanceGrid& distances,
                const SearchParams& params);
bool extractPath(PathNode& node, 
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
    // path.path.push_back(start);
    
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
    std::set<PathNode> openList;
    openList.insert(startNode);
    currentNode = *openList.begin();
    openList.erase(openList.begin());
    bool isExtracted = false;
    bool isPathFound = false;
    while (currentNode != goalNode)
    {
        closedList.push_back(currentNode);
        expandNode(currentNode, goalNode, openList, closedList, searchedList, distances, params);
        // std::cout << "!!!!!! openlist length: " << openList.size() << std::endl;
        if (openList.empty())
        {
            break;
        }
        // currentNode = openList.top();
        // openList.pop();
        currentNode = *openList.begin();
        openList.erase(openList.begin());
        // std::cout << "!!!!!!!!!!!curnode: " << currentNode.cell << std::endl;
        if (currentNode == goalNode)
        {
            isPathFound = true;
            // std::cout << "!!!!isPathFound: " << isPathFound << std::endl;
        }
    }
    if (isPathFound)
    {   
        // std::cout << "!!!!isExtraced????: " << isExtracted << std::endl;
        isExtracted = extractPath(currentNode, startNode, closedList, path, distances);
        // std::cout << "!!!!isExtraced: " << isExtracted << std::endl;
        //set start theta
        // float dx1 = path.path[1].x - path.path[0].x;
        // float dy1 = path.path[1].y - path.path[0].y;
        // path.path[0].theta = wrap_to_pi(std::atan2(dy1, dx1));
        //set end theta
        float dx2 = goal.x - path.path.back().x;
        float dy2 = goal.y - path.path.back().y;
        path.path.back().theta = wrap_to_pi(std::atan2(dy2, dx2));
        path.path.push_back(goal);

        // std::cout << "***************" << path.path.size() << std::endl;
        // for (int j = 0; j<int(path.path.size()); j++)
        //     std::cout << path.path[j].x << " " << path.path[j].y << " " << path.path[j].theta << std::endl;
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
    // for (auto& n : list)
    // {
    //     if (n.cell == node.cell)
    //     {
    //         return true;
    //     }
    // }
    // std::set<PathNode>::iterator it;
    if (std::find(list.begin(), list.end(), node) != list.end()) return true;
    // it = list.find(node);
    // if (it != list.end())
    //     return true;

    return false;
}

PathNode getNodeFromList(PathNode& node, std::vector<PathNode>& list)
{
    if (isNodeInList(node, list))
    {   
        // for (auto& n : list)
        // {
        //     if (n.cell == node.cell)
        //     {
        //         return n;
        //     }
        // }
        std::vector<PathNode>::iterator it;
        it = std::find(list.begin(), list.end(), node);
        // it = list.find(node);
        // std::cout << "**** getNode: " << it->cell << std::endl;
        return *it;
    }
    return node;
}

void expandNode(PathNode &node,
                PathNode &goalNode,
                std::set<PathNode> &openList,
                std::vector<PathNode> &closedList,
                std::vector<PathNode> &searchedList,
                const ObstacleDistanceGrid& distances,
                const SearchParams& params)
{
    int xDeltas[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    int yDeltas[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    for (int i = 0; i < 8; i++)
    {
        PathNode neighbor;
        neighbor.gCost = node.gCost;
        neighbor.cell.x = node.cell.x + xDeltas[i];
        neighbor.cell.y = node.cell.y + yDeltas[i];
        neighbor.hCost = calHCost(neighbor, goalNode, distances);
        // if (isNodeInList(neighbor, searchedList))
        // {
        //     neighbor = getNodeFromList(neighbor, searchedList);
        // }
        if ((!isNodeInList(neighbor, closedList)) 
            && (distances(neighbor.cell.x, neighbor.cell.y) >= 1.5*params.minDistanceToObstacle)
            && distances.isCellInGrid(neighbor.cell.x, neighbor.cell.y))
        {
            if (!isNodeInList(neighbor, searchedList))
            {
                if (distances(neighbor.cell.x, neighbor.cell.y) >= params.maxDistanceWithCost)
                {
                    neighbor.gCost += distances.metersPerCell() + (i>=4)*0.414*distances.metersPerCell();
                }
                else
                {
                    neighbor.gCost += distances.metersPerCell() + (i>=4)*0.414*distances.metersPerCell()+ 10*abs(std::pow(params.maxDistanceWithCost 
                                                                                                            - distances(neighbor.cell.x, neighbor.cell.y), 
                                                                                                            params.distanceCostExponent));
                }
                neighbor.hCost = calHCost(neighbor, goalNode, distances);
                neighbor.parent = node.cell;
                openList.insert(neighbor);
                searchedList.push_back(neighbor);
            }
            else //in searched list
            {
                neighbor = getNodeFromList(neighbor, searchedList);
                float newGCost;
                if (distances(neighbor.cell.x, neighbor.cell.y) >= params.maxDistanceWithCost)
                {
                    newGCost = node.gCost + distances.metersPerCell() + (i>=4)*0.414*distances.metersPerCell();
                }
                else
                {
                    newGCost = node.gCost + distances.metersPerCell() + (i>=4)*0.414*distances.metersPerCell() + 10*abs(std::pow(params.maxDistanceWithCost 
                                                                                                                    - distances(neighbor.cell.x, neighbor.cell.y), 
                                                                                                                    params.distanceCostExponent));
                }
                if (neighbor.gCost > newGCost)
                {
                    neighbor.gCost = newGCost;
                    neighbor.parent = node.cell;
                    openList.erase(neighbor);
                    openList.insert(neighbor);
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

bool extractPath(PathNode& node, 
                 PathNode& startNode,
                 std::vector<PathNode> &closedList,
                 robot_path_t& path,
                 const ObstacleDistanceGrid& grid)
{
    PathNode curNode = node;
    std::vector<PathNode> tempList;
    
    while (curNode != startNode)
    {
        // std::cout << "Enter while" << std::endl;
        tempList.push_back(curNode);
        PathNode parentNode;
        parentNode.cell = curNode.parent;
        parentNode = getNodeFromList(parentNode, closedList);
        curNode = parentNode;
    }
    std::reverse(tempList.begin(), tempList.end());
    // std::cout << "reverse done" << std::endl;
    int n = tempList.size();
    pose_xyt_t nextPose, prevPose;
    prevPose.x = grid.originInGlobalFrame().x + startNode.cell.x*grid.metersPerCell();
    prevPose.y = grid.originInGlobalFrame().y + startNode.cell.y*grid.metersPerCell();
    for (int i = 0; i < n; i++)
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

    return true;
}