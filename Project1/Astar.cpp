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
#include "Astar.h"

using namespace std;

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int>> pPair;

// A structure to hold the neccesary parameters
struct cell
{
	// Row and Column index of its parent
	// Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	int parent_i, parent_j;
	// f = g + h
	double f, g, h;
};



// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col, int boardSize)
{
	// Returns true if row number and column number
	// is in range
	return (row >= 0) && (row < boardSize) &&
		(col >= 0) && (col < boardSize);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(string **grid, int row, int col)
{
	// Returns true if the cell is not blocked else false
	if (grid[row][col] != "W")
		return (true);
	else
		return (false);
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col, Pair dest)
{
	if (row == dest.first && col == dest.second)
		return (true);
	else
		return (false);
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest)
{
	// Return using the distance formula
	return ((double)sqrt((row - dest.first)*(row - dest.first)
		+ (col - dest.second)*(col - dest.second)));
}

// A Utility Function to trace the path from the source
// to destination
stack<string> tracePath(cell **cellDetails, Pair dest, string **grid)
{
	//weight map, weight of each road feature.
	map<string, int> weights;
	weights["R"] = 1;
	weights["D"] = 3;
	weights["H"] = 10;
	weights["G"] = 0;
	weights["S"] = 0;

	int row = dest.first;
	int col = dest.second;

	stack<Pair> Path;
	stack<string> wayPath;

	while (!(cellDetails[row][col].parent_i == row
		&& cellDetails[row][col].parent_j == col))
	{
		Path.push(make_pair(row, col));
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}

	Path.push(make_pair(row, col));
	int pathSize = Path.size();
	Pair * pathArrey;
	pathArrey = new Pair[pathSize];
	int i = 0;
	double weight = 0.0;
	stack<string> pathStacktoFile;
	while (!Path.empty())
	{
		pair<int, int> p = Path.top();
		pathArrey[i] = p;
		Path.pop();
		i++;
	}

	for (int i = 0; i < pathSize - 1; i++)
	{
		Pair p1 = pathArrey[i];
		Pair p2 = pathArrey[i + 1];
		weight += weights.at(grid[p2.first][p2.second]);
		int y = p2.first - p1.first; //row dest
		int x = p2.second - p1.second; //col dest
		if ((x == 1) && (y == 0))
			pathStacktoFile.push("R");
		else if ((x == 1) && (y == 1))
			pathStacktoFile.push("RD");
		else if ((x == 0) && (y == 1))
			pathStacktoFile.push("D");
		else if ((x == -1) && (y == 1))
			pathStacktoFile.push("LD");
		else if ((x == -1) && (y == 0))
			pathStacktoFile.push("L");
		else if ((x == -1) && (y == -1))
			pathStacktoFile.push("LU");
		else if ((x == 0) && (y == -1))
			pathStacktoFile.push("U");
		else if ((x == 1) && (y == -1))
			pathStacktoFile.push("RU");
		if (i < pathSize - 2)
			pathStacktoFile.push("-");
		else
		{
			pathStacktoFile.push(" ");
			pathStacktoFile.push(to_string(int(weight)));
		}
	}

	stack<string> pathStacktoFileReverse;
	string weightpath = pathStacktoFile.top();
	pathStacktoFile.pop();
	while (!pathStacktoFile.empty())
	{
		string str = pathStacktoFile.top();
		pathStacktoFileReverse.push(str);
		pathStacktoFile.pop();
	}
	pathStacktoFileReverse.push(weightpath);

	return pathStacktoFileReverse;
}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
stack<string> aStarSearch(string **grid, int boardSize)
{
	stack<string> stackStringToReturn;
	//Src, Dest Pairs.
	Pair src = make_pair(0, 0);
	Pair dest = make_pair(boardSize - 1, boardSize - 1);
	//weight map, weight of each road feature.
	map<string, int> weights;
	weights["R"] = 1;
	weights["D"] = 3;
	weights["H"] = 10;
	weights["G"] = 0;
	weights["S"] = 0;
	// If the source is out of range
	if (isValid(src.first, src.second, boardSize) == false)
	{
		stackStringToReturn.push("no path");
		return stackStringToReturn;
	}

	// If the destination is out of range
	if (isValid(dest.first, dest.second, boardSize) == false)
	{
		stackStringToReturn.push("no path");
		return stackStringToReturn;
	}

	// Either the source or the destination is blocked
	if (isUnBlocked(grid, src.first, src.second) == false ||
		isUnBlocked(grid, dest.first, dest.second) == false)
	{
		stackStringToReturn.push("no path");
		return stackStringToReturn;
	}

	// If the destination cell is the same as source cell
	if (isDestination(src.first, src.second, dest) == true)
	{
		stackStringToReturn.push("no path");
		return stackStringToReturn;
	}

	// Declare a 2D array of structure to hold the details
	//of that cell
	cell** cellDetails = new cell*[boardSize];
	for (int i = 0; i < boardSize; ++i)
		cellDetails[i] = new cell[boardSize];

	int i, j;

	for (i = 0; i < boardSize; i++)
	{
		for (j = 0; j < boardSize; j++)
		{
			cellDetails[i][j].f = FLT_MAX;
			cellDetails[i][j].g = FLT_MAX;
			cellDetails[i][j].h = FLT_MAX;
			cellDetails[i][j].parent_i = -1;
			cellDetails[i][j].parent_j = -1;
		}
	}

	// Initialising the parameters of the starting node
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;

	/*
	Create an open list having information as-
	<f, <i, j>>
	where f = g + h,
	and i, j are the row and column index of that cell
	Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	This open list is implenented as a set of pair of pair.*/
	set<pPair> openList;

	// Put the starting cell on the open list and set its
	// 'f' as 0
	openList.insert(make_pair(0.0, make_pair(i, j)));

	// We set this boolean value as false as initially
	// the destination is not reached.
	bool foundDest = false;

	while (!openList.empty())
	{
		pPair p = *openList.begin();

		// Remove this vertex from the open list
		openList.erase(openList.begin());

		// Add this vertex to the open list
		i = p.second.first;
		j = p.second.second;

		// To store the 'g', 'h' and 'f' of the 8 successors
		double gNew, hNew, fNew;

		//----------- 1st Successor (R) ------------

		// Only process this cell if this is a valid one
		if (isValid(i, j + 1, boardSize) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i][j + 1].parent_i = i;
				cellDetails[i][j + 1].parent_j = j;
				foundDest = true;
				return tracePath(cellDetails, dest, grid);
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (isUnBlocked(grid, i, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + weights.at(grid[i][j + 1]);
				hNew = calculateHValue(i, j + 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i][j + 1].f == FLT_MAX ||
					cellDetails[i][j + 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i, j + 1)));

					// Update the details of this cell
					cellDetails[i][j + 1].f = fNew;
					cellDetails[i][j + 1].g = gNew;
					cellDetails[i][j + 1].h = hNew;
					cellDetails[i][j + 1].parent_i = i;
					cellDetails[i][j + 1].parent_j = j;
				}
			}
		}
		//----------- 2nd Successor (RD) ------------

		// Only process this cell if this is a valid one

		if (isValid(i + 1, j + 1, boardSize) == true && isUnBlocked(grid, i, j + 1) == true && isUnBlocked(grid, i + 1, j) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i + 1, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i + 1][j + 1].parent_i = i;
				cellDetails[i + 1][j + 1].parent_j = j;
				foundDest = true;
				return tracePath(cellDetails, dest, grid);
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (isUnBlocked(grid, i + 1, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + weights.at(grid[i + 1][j + 1]);
				hNew = calculateHValue(i + 1, j + 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i + 1][j + 1].f == FLT_MAX ||
					cellDetails[i + 1][j + 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i + 1, j + 1)));

					// Update the details of this cell
					cellDetails[i + 1][j + 1].f = fNew;
					cellDetails[i + 1][j + 1].g = gNew;
					cellDetails[i + 1][j + 1].h = hNew;
					cellDetails[i + 1][j + 1].parent_i = i;
					cellDetails[i + 1][j + 1].parent_j = j;
				}
			}
		}

		//----------- 3rd Successor (D) ------------

		// Only process this cell if this is a valid one
		if (isValid(i + 1, j, boardSize) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i + 1, j, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i + 1][j].parent_i = i;
				cellDetails[i + 1][j].parent_j = j;
				foundDest = true;
				return tracePath(cellDetails, dest, grid);
			}
			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (isUnBlocked(grid, i + 1, j) == true)
			{
				gNew = cellDetails[i][j].g + weights.at(grid[i + 1][j]);
				hNew = calculateHValue(i + 1, j, dest);
				fNew = gNew + hNew;

				if (cellDetails[i + 1][j].f == FLT_MAX ||
					cellDetails[i + 1][j].f > fNew)
				{
					openList.insert(make_pair(fNew, make_pair(i + 1, j)));
					// Update the details of this cell
					cellDetails[i + 1][j].f = fNew;
					cellDetails[i + 1][j].g = gNew;
					cellDetails[i + 1][j].h = hNew;
					cellDetails[i + 1][j].parent_i = i;
					cellDetails[i + 1][j].parent_j = j;
				}
			}
		}
		//----------- 4th Successor (LD) ------------

		// Only process this cell if this is a valid one
		if (isValid(i + 1, j - 1, boardSize) == true && isUnBlocked(grid, i + 1, j) == true && isUnBlocked(grid, i, j - 1) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i + 1, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i + 1][j - 1].parent_i = i;
				cellDetails[i + 1][j - 1].parent_j = j;
				foundDest = true;
				return tracePath(cellDetails, dest, grid);
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (isUnBlocked(grid, i + 1, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + weights.at(grid[i + 1][j - 1]);
				hNew = calculateHValue(i + 1, j - 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i + 1][j - 1].f == FLT_MAX ||
					cellDetails[i + 1][j - 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i + 1, j - 1)));

					// Update the details of this cell
					cellDetails[i + 1][j - 1].f = fNew;
					cellDetails[i + 1][j - 1].g = gNew;
					cellDetails[i + 1][j - 1].h = hNew;
					cellDetails[i + 1][j - 1].parent_i = i;
					cellDetails[i + 1][j - 1].parent_j = j;
				}
			}
		}

		//----------- 5th Successor (L) ------------

		// Only process this cell if this is a valid one
		if (isValid(i, j - 1, boardSize) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i][j - 1].parent_i = i;
				cellDetails[i][j - 1].parent_j = j;
				foundDest = true;
				return tracePath(cellDetails, dest, grid);
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (isUnBlocked(grid, i, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + weights.at(grid[i][j - 1]);
				hNew = calculateHValue(i, j - 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i][j - 1].f == FLT_MAX ||
					cellDetails[i][j - 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i, j - 1)));

					// Update the details of this cell
					cellDetails[i][j - 1].f = fNew;
					cellDetails[i][j - 1].g = gNew;
					cellDetails[i][j - 1].h = hNew;
					cellDetails[i][j - 1].parent_i = i;
					cellDetails[i][j - 1].parent_j = j;
				}
			}
		}

		//----------- 6th Successor (LU) ------------

		// Only process this cell if this is a valid one
		if (isValid(i - 1, j - 1, boardSize) == true && isUnBlocked(grid, i, j - 1) == true && isUnBlocked(grid, i - 1, j) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i - 1, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i - 1][j - 1].parent_i = i;
				cellDetails[i - 1][j - 1].parent_j = j;
				foundDest = true;
				return tracePath(cellDetails, dest, grid);
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (isUnBlocked(grid, i - 1, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + weights.at(grid[i - 1][j - 1]);
				hNew = calculateHValue(i - 1, j - 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i - 1][j - 1].f == FLT_MAX ||
					cellDetails[i - 1][j - 1].f > fNew)
				{
					openList.insert(make_pair(fNew, make_pair(i - 1, j - 1)));
					// Update the details of this cell
					cellDetails[i - 1][j - 1].f = fNew;
					cellDetails[i - 1][j - 1].g = gNew;
					cellDetails[i - 1][j - 1].h = hNew;
					cellDetails[i - 1][j - 1].parent_i = i;
					cellDetails[i - 1][j - 1].parent_j = j;
				}
			}
		}

		//----------- 7th Successor (U) ------------

		// Only process this cell if this is a valid one
		if (isValid(i - 1, j, boardSize) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i - 1, j, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i - 1][j].parent_i = i;
				cellDetails[i - 1][j].parent_j = j;
				foundDest = true;
				return tracePath(cellDetails, dest, grid);
			}
			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (isUnBlocked(grid, i - 1, j) == true)
			{
				gNew = cellDetails[i][j].g + weights.at(grid[i - 1][j]);
				hNew = calculateHValue(i - 1, j, dest);
				fNew = gNew + hNew;

				if (cellDetails[i - 1][j].f == FLT_MAX ||
					cellDetails[i - 1][j].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i - 1, j)));

					// Update the details of this cell
					cellDetails[i - 1][j].f = fNew;
					cellDetails[i - 1][j].g = gNew;
					cellDetails[i - 1][j].h = hNew;
					cellDetails[i - 1][j].parent_i = i;
					cellDetails[i - 1][j].parent_j = j;
				}
			}
		}

		//----------- 8th Successor (RU) ------------

		// Only process this cell if this is a valid one
		if (isValid(i - 1, j + 1, boardSize) == true && isUnBlocked(grid, i - 1, j) == true && isUnBlocked(grid, i + 1, j) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i - 1, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i - 1][j + 1].parent_i = i;
				cellDetails[i - 1][j + 1].parent_j = j;
				foundDest = true;
				return tracePath(cellDetails, dest, grid);
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (isUnBlocked(grid, i - 1, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + weights.at(grid[i - 1][j + 1]);
				hNew = calculateHValue(i - 1, j + 1, dest);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//                OR
				// If it is on the open list already, check
				// to see if this path to that square is better,
				// using 'f' cost as the measure.
				if (cellDetails[i - 1][j + 1].f == FLT_MAX ||
					cellDetails[i - 1][j + 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i - 1, j + 1)));

					// Update the details of this cell
					cellDetails[i - 1][j + 1].f = fNew;
					cellDetails[i - 1][j + 1].g = gNew;
					cellDetails[i - 1][j + 1].h = hNew;
					cellDetails[i - 1][j + 1].parent_i = i;
					cellDetails[i - 1][j + 1].parent_j = j;
				}
			}
		}


		// When the destination cell is not found and the open
		// list is empty, then we conclude that we failed to
		// reach the destiantion cell. This may happen when the
		// there is no way to destination cell (due to blockages)
	}

	stackStringToReturn.push("no path");

	return stackStringToReturn;

}