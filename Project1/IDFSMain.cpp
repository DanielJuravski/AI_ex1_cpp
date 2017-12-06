#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <stack>
#include <queue>
#include <map>
#include "IDFSGraph.h"
#include "IDFSMain.h"
using namespace std;

//An utility function that calculate the number of the node with its coordinates
int numofnode(int row, int col, int boardSize)
{
	return ((boardSize - 2) * (row - 1) + col);
}

//Iterative DFS algorithm
stack<string> IDFSMain(int boardSize, string **board, int &o_pathWeight)
{
	map<string, int> weights;
	weights["R"] = 1;
	weights["D"] = 3;
	weights["H"] = 10;
	weights["G"] = 0;
	weights["S"] = 0;

	IDFSGraph g(pow((boardSize - 2), 2));
	int row = 1;
	int col = 1;
	int numOfNode;
	queue<int> numOfNodeQueue;
	int *visitedNodes = new int[pow((boardSize - 2), 2)];
	fill_n(visitedNodes, pow((boardSize - 2), 2), 0);

	numOfNodeQueue.push(numofnode(row, col, boardSize));
	while (!numOfNodeQueue.empty())
	{
		numOfNode = numOfNodeQueue.front();
		numOfNodeQueue.pop();
		row = numOfNode / (boardSize - 2) + 1;
		col = numOfNode % (boardSize - 2);

		if ((visitedNodes[numOfNode - 1] != 1) && (board[row][col] != "W"))
		{
			node srcNode(numofnode(row, col, boardSize), 0, "None");
			if (board[row][col + 1] != "W")
			{
				node destNode(numofnode(row, col + 1, boardSize), weights.at(board[row][col + 1]), "R");
				g.addEdge(srcNode, destNode);
				numOfNodeQueue.push(numofnode(row, col + 1, boardSize));
				if (board[row + 1][col] != "W")
				{
					if (board[row + 1][col + 1] != "W")
					{
						node destNode(numofnode(row + 1, col + 1, boardSize), weights.at(board[row + 1][col + 1]), "RD");
						g.addEdge(srcNode, destNode);
						numOfNodeQueue.push(numofnode(row + 1, col + 1, boardSize));
					}
				}
			}
			if (board[row + 1][col] != "W")
			{
				node destNode(numofnode(row + 1, col, boardSize), weights.at(board[row + 1][col]), "D");
				g.addEdge(srcNode, destNode);
				numOfNodeQueue.push(numofnode(row + 1, col, boardSize));
				if (board[row][col - 1] != "W")
				{
					if (board[row + 1][col - 1] != "W")
					{
						node destNode(numofnode(row + 1, col - 1, boardSize), weights.at(board[row + 1][col - 1]), "LD");
						g.addEdge(srcNode, destNode);
						numOfNodeQueue.push(numofnode(row + 1, col - 1, boardSize));
					}
				}
			}
			if (board[row][col - 1] != "W")
			{
				node destNode(numofnode(row, col - 1, boardSize), weights.at(board[row][col - 1]), "L");
				g.addEdge(srcNode, destNode);
				numOfNodeQueue.push(numofnode(row, col - 1, boardSize));
				if (board[row - 1][col] != "W")
				{
					if (board[row - 1][col - 1] != "W")
					{
						node destNode(numofnode(row - 1, col - 1, boardSize), weights.at(board[row - 1][col - 1]), "LU");
						g.addEdge(srcNode, destNode);
						numOfNodeQueue.push(numofnode(row - 1, col - 1, boardSize));
					}
				}
			}
			if (board[row - 1][col] != "W")
			{
				node destNode(numofnode(row - 1, col, boardSize), weights.at(board[row - 1][col]), "U");
				g.addEdge(srcNode, destNode);
				numOfNodeQueue.push(numofnode(row - 1, col, boardSize));
				if (board[row][col + 1] != "W")
				{
					if (board[row - 1][col + 1] != "W")
					{
						node destNode(numofnode(row - 1, col + 1, boardSize), weights.at(board[row - 1][col + 1]), "RU");
						g.addEdge(srcNode, destNode);
						numOfNodeQueue.push(numofnode(row - 1, col + 1, boardSize));
					}
				}
			}
			visitedNodes[numOfNode - 1] = 1;
		}
	}

	int pathWeight = 0;
	stack<string> pathDirections;
	stack<string> pathDirectionsToFile;
	stack<string> pathDirectionsToFileReverse;
	int maxDepth = 10;
	node nodeS(1, 0, "START");
	node nodeG((pow((boardSize - 2), 2)), 0, "GOAL");
	g.IDDFS(nodeS, nodeG, maxDepth, &pathWeight, &pathDirections);
	o_pathWeight = pathWeight;
	if (pathDirections.empty())
		return pathDirections;
	pathDirections.pop();
	while (!pathDirections.empty())
	{
		string str = pathDirections.top();
		pathDirections.pop();
		pathDirectionsToFile.push(str);
		pathDirectionsToFile.push("-");
	}
	pathDirectionsToFile.pop();
	pathDirectionsToFile.push(" ");

	while (!pathDirectionsToFile.empty())
	{
		string str = pathDirectionsToFile.top();
		pathDirectionsToFile.pop();
		pathDirectionsToFileReverse.push(str);
	}
	return pathDirectionsToFileReverse;
}