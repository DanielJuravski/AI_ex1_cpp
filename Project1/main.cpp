#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <stack>
#include <queue>
#include <map>
#include <list>
#include <set>
#include "IDFSGraph.h"
#include "IDFSMain.h"
#include "Astar.h"

using namespace std;

//read the input file, process it, and get
//the algorithm type, size of matrix(with borders), the matrix.
//returned board: matrix with 'W' around.
string** processFileForAstar(ifstream& i_infile, string &o_algorithm, int &o_boardSize)
{
	string boardSizeStr;
	string boardLine;
	string **board = NULL;

	if (i_infile.is_open())
	{
		getline(i_infile, o_algorithm);
		getline(i_infile, boardSizeStr);
		o_boardSize = stoi(boardSizeStr);
		board = new string*[o_boardSize];
		for (int i = 0; i < o_boardSize; i++)
			board[i] = new string[o_boardSize];
		int row = 0;
		while (getline(i_infile, boardLine))
		{
			for (int col = 0; col < o_boardSize; col++)
				board[row][col] = boardLine[col];
			row++;
		}
	}
	else
	{
		board = NULL;
	}
	i_infile.close();

	return board;
}

//add "W" borders to the original board, increse the board size +2
string** processBoardForIDFS(string **i_board, int o_boardSize)
{
	string boardSizeStr;
	string boardLine;
	string **board = NULL;

	//o_boardSize -  size of board with borders
	board = new string*[o_boardSize];
	for (int i = 0; i < o_boardSize; i++)
		board[i] = new string[o_boardSize];

	for (int i = 1; i < o_boardSize - 1; i++)
		for (int j = 1; j < o_boardSize - 1; j++)
			board[i][j] = i_board[i - 1][j - 1];

	for (int col = 0; col < o_boardSize; col++) //initializw first and last rows to 'W'
	{
		board[0][col] = 'W';
		board[o_boardSize - 1][col] = 'W';
	}
	for (int row = 0; row < o_boardSize; row++) //initializw first and last cols to 'W'
	{
		board[row][0] = 'W';
		board[row][o_boardSize - 1] = 'W';
	}

	return board;
}

//print stack of strings
void printStringStack(stack<string> s)
{
	s.pop();
	while (!s.empty())
	{
		string w = s.top();
		cout << w;
		s.pop();
	}
}

void writeToOutputFile(stack<string> way, int weight)
{
	ofstream outFile;
	outFile.open("output.txt");
	if (weight < 1)
	{
		outFile << "no path";
	}
	else
	{
		while (!way.empty())
		{
			string str = way.top();
			way.pop();
			outFile << str;
		}
		outFile << weight;
	}
	outFile.close();
}

int main(char ** argv)
{
	string algorithm;
	int boardSize;
	string **boardIDFS = NULL; //will be with 'W' borders
	string **boardAstar = NULL;
	ifstream file("C:/Users/Juravski/Documents/BIU/Artificial Intelligence/dev/HW1/input1.txt");
	boardAstar = processFileForAstar(file, algorithm, boardSize);
	int pathWeight = 0;
	stack<string> pathWay;

	if (algorithm == "IDS")
	{
		boardIDFS = processBoardForIDFS(boardAstar, boardSize + 2); //add 'W' borders,  boardSize+=2
		pathWay = IDFSMain(boardSize + 2, boardIDFS, pathWeight);
		writeToOutputFile(pathWay, pathWeight);
	}
	else if (algorithm == "A*")
	{
		pathWay = aStarSearch(boardAstar, boardSize);
		if (size(pathWay) > 1)
		{
			pathWeight = stoi(pathWay.top());
			pathWay.pop();
		}
		writeToOutputFile(pathWay, pathWeight);
	}


	return 0;
}