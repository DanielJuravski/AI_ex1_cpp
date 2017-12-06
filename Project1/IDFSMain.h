#pragma once
#pragma once
//An utility function that calculate the number of the node with its coordinates
int numofnode(int row, int col, int boardSize);
//Iterative DFS algorithm
stack <string> IDFSMain(int boardSize, string **board, int &o_pathWeight);