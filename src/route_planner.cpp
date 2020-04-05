#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // convert inputs to percentage
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // finding the closest nodes to the starting and ending coordinates
    // storing the nodes in the RoutePlanner's start_node and end_node attributes
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    start_node->visited = true;
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// Using the distance to the end_node for the h value
float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*end_node);
}

// Expanding the current node by adding all unvisited neighbors to the open list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    // populating current_node.neighbors vector with all the neighbors of the current_node
    current_node->FindNeighbors();

    // for each node in current_node.neighbors, setting the parent, the h_value, the g_value.
    for (auto &node : current_node->neighbors)
    {
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        open_list.push_back(node);
        node->visited = true;
    }
}

bool Compare(const RouteModel::Node *n1, const RouteModel::Node *n2)
{
    return n1->g_value + n1->h_value > n2->g_value + n2->h_value;
}

// Sorting the open list and return the next node
RouteModel::Node *RoutePlanner::NextNode()
{
    // sorting the open_list according to the f value (f_value = h_value + g_value)
    std::sort(open_list.begin(), open_list.end(), Compare);

    // create a pointer to the node in the list with the lowest sum.
    RouteModel::Node *current_node_lowest_f_value = open_list.back();

    // removing that node from the open_list.
    open_list.pop_back();

    return current_node_lowest_f_value;
}

// Taking the current (final) node as an argument and iteratively following the chain of parents of nodes until the starting node is found
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    distance = 0.0f;

    std::vector<RouteModel::Node> path_found;
    path_found.push_back(*current_node);

    while (current_node->parent)
    {
        RouteModel::Node *parent = current_node->parent;
        // summing up the distance from the node to its parent for the whole path
        distance += current_node->distance(*parent);
        path_found.push_back(*parent);
        current_node = parent;
    }

    // Reversing the path vector: the start node should be the first element of the vector, the end node should be the last element
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters
    return path_found;
}

void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = start_node;

    while (current_node != end_node)
    {
        // adding all of the neighbors of the current node to the open_list
        AddNeighbors(current_node);
        // sorting the open_list and returning the next node
        current_node = NextNode();
    }

    // return the final path that was found
    m_Model.path = ConstructFinalPath(current_node);
}