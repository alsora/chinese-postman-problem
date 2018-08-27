#ifndef __BRANCH_BOUND_H__
#define __BRANCH_BOUND_H__


#include "graph/graph.h"

struct BranchNBoundStruct
{
	Graph graph;
	std::set<int> remainedElements;
	float cost;

	//Default constructor
	BranchNBoundStruct();
	//Constructor
	BranchNBoundStruct(Graph graph, std::set<int> remainedElements);

};


inline bool operator >(const BranchNBoundStruct &A, const BranchNBoundStruct& B) {
	if (A.cost > B.cost) {
		return true;
	}
	else return false;
}



#endif