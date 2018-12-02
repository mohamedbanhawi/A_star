#include <iostream>
#include <math.h>
#include <vector>
#include "astar.hpp"

int main()
{
  a_star AS;

  // a_star::map UserMap = {
  //               { 1,-1, 0, 0, 0, 0, 0},
  //               { 0,-1, 0, 0, 0, 0, 0},
  //               { 0, 0, 0, -1, -1, -1, 0},
  //               { 0, -1, -1, 2, 0, -1, 0},
  //               { 0,-1,  -1,-1, 0, -1, 0},
  //               { 0,-1,  -1, 0, 0, 0, 0},
  //               { 0,-1,  -1, 0, 0, 0, 0}
  //               };

    a_star::map UserMap = {
                { 1,-1, 0, 0, 0, 0},
                { 0,-1, 0, 0, 2, 0},
                { 0, 0, 0, 0, 0, 0}
                };
  AS.Init(UserMap);

  bool success = AS.Search();

  std::cout << "Search result = "<< success << std::endl;

  AS.ShowGrid();
}