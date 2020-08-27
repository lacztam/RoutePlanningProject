#include "route_planner.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <iterator>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x   *= 0.01;
    end_y   *= 0.01;
    
    start_node = &m_Model.FindClosestNode(start_x,start_y); 
    end_node   = &m_Model.FindClosestNode(end_x,end_y); 
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
  return node->distance(*end_node);
}

//
// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();
    for (auto &neighbour : current_node->neighbors)
    {
      if(neighbour->visited) 
      {
        std::cout << "AddNeighbors() - neighbour->visited == true\n";
      }
      else
      {
      neighbour->parent   = current_node;
      neighbour->h_value  = CalculateHValue(neighbour);
      neighbour->g_value += neighbour->distance(*current_node);

      open_list.push_back(neighbour);
      neighbour->visited = true;
      }
    }    
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool CompareNodes(RouteModel::Node *n1, RouteModel::Node *n2)
{
  float f1 = n1->g_value + n1->h_value;
  float f2 = n2->g_value + n2->h_value;
  return f1 < f2;
}

RouteModel::Node *RoutePlanner::NextNode()
{
  std::sort(open_list.begin(), open_list.end(), CompareNodes);

  RouteModel::Node * nextNode = open_list[0];
  open_list.erase(open_list.begin());

  //if(nextNode == end_node) open_list.clear();
  return nextNode;
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

bool CompareNodeContents(std::vector<RouteModel::Node> path)
{
  int counter = 0;

  for (size_t i = 0; i < path.size() ; i++)
  {
    for (size_t j = 0; j < path.size() ; j++)
    {
      RouteModel::Node* n1 = &path[i];
      RouteModel::Node* n2 = &path[j];

      float x1 = n1->x;
      float y1 = n1->y;
      float x2 = n2->x;
      float y2 = n2->y;
      float g1 = n1->g_value;
      float g2 = n2->g_value;
      float h1 = n1->h_value;
      float h2 = n2->h_value;
      float f1 = h1+g1;
      float f2 = h2+g2;

      if(x1 == x2 && y1 == y2 && f1 == f2) counter++;
    }    
  }

  if(counter == path.size() ) 
  {
    std::cout << "\nThere is no duplicated element in the vector!\n";
    return true;
  }
  else
  {
    std::cout << "\nThere is duplicated element in the vector!\n";
    return false;
  }
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    current_node->h_value = CalculateHValue(end_node);
    current_node->visited = true;

    while (current_node != start_node)
    {
      path_found.push_back(*current_node);

      distance += current_node->distance(*current_node->parent);
      current_node = current_node->parent;
      if(current_node == start_node) path_found.push_back(*current_node);
    }
    
    std::reverse(std::begin(path_found), std::end(path_found));

    if(CompareNodeContents(path_found)) std::cout << "ConstructFinalPath() - CompareNodeContents() done.\n\n";

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch()
{
  RouteModel::Node *current_node = start_node;

  current_node->visited = true;
  current_node->h_value = CalculateHValue(end_node);

  open_list.push_back(current_node);

  while(current_node != end_node)
  {
    AddNeighbors(current_node);
    current_node = NextNode();
    if(current_node == end_node) m_Model.path = ConstructFinalPath(current_node);
  } 
}