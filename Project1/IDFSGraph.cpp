#include "IDFSGraph.h"
#include<iostream>
#include<bits.h>
#include<list>
#include<stack>
using namespace std;

//Iterative DFS algorithm
IDFSGraph::IDFSGraph(int V)
{
	this->V = V;
	adj = new list<node>[V];
}

//add link between node v and node w
void IDFSGraph::addEdge(node v, node w)
{
	adj[v.numOfNode].push_back(w); // Add w to v’s list.
								   //cout << v.numOfNode << "-" << w.numOfNode<<endl;
}

// A function to perform a Depth-Limited search
// from given source 'src'
bool IDFSGraph::DLS(node src, node target, int limit, int *o_weight, stack<string> *o_directions)
{
	if (src.numOfNode == target.numOfNode)
	{
		o_directions->push(src.wayOfNode);
		*o_weight += src.weightOfNode;
		return true;
	}

	// If reached the maximum depth, stop recursing.
	if (limit <= 0)
		return false;

	// Recur for all the vertices adjacent to source vertex
	for (auto i = adj[src.numOfNode].begin(); i != adj[src.numOfNode].end(); ++i)
		if (DLS(*i, target, limit - 1, o_weight, o_directions) == true)
		{
			*o_weight += src.weightOfNode;
			o_directions->push(src.wayOfNode);
			return true;
		}

	return false;
}

// IDDFS to search if target is reachable from v.
// In addition, calculate the path weight and directions.
bool IDFSGraph::IDDFS(node src, node target, int max_depth, int *o_weight, stack<string> *o_directions)
{
	// Repeatedly depth-limit search till the
	// maximum depth.
	for (int i = 0; i <= max_depth; i++)
	{
		if (DLS(src, target, i, o_weight, o_directions) == true)
			return true;
	}

	return false;
}

