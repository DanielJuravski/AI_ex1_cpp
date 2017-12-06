#pragma once
#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <stack>
#include <queue>
#include <map>
#include <list>
#include <set>
#include <cstdlib>
using namespace std;

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

//Just for implement the "aStarSearch" function from the main.
stack<string> aStarSearch(string **grid, int boardSize);
