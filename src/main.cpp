#include <iostream>
#include <math.h>
#include <vector>
#include "astar.hpp"

int main()
{
  a_star AS;

  a_star::map UserMap = {
                { 1,-1, 0, 0},
                { 0,-1, 0, 0},
                { 0, 0, 0, -1},
                { 0, -1, 0, 2}
                };
  AS.Init(UserMap);

  AS.Search();

  AS.ShowGrid();
}