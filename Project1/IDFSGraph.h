#pragma once
#pragma once
#include<iostream>
#include<list>
#include<stack>
using namespace std;

struct node {
	int numOfNode;
	int weightOfNode;
	string wayOfNode;
	node(int i_numOfNode, int i_weightOfNode, string i_wayOfNode)
	{
		numOfNode = i_numOfNode;
		weightOfNode = i_weightOfNode;
		wayOfNode = i_wayOfNode;
	}
};

//Iterative DFS algorithm
class IDFSGraph
{
	int V;    // No. of vertices

			  // Pointer to an array containing
			  // adjacency lists
	list<node> *adj;

	// A function used by IDDFS
	bool DLS(node v, node target, int limit, int *o_weight, stack<string> *o_directions);

public:
	IDFSGraph(int V);   // Constructor
	void addEdge(node v, node w);

	// IDDFS traversal of the vertices reachable from v
	bool IDDFS(node v, node target, int max_depth, int *o_weight, stack<string> *o_directions);
};

