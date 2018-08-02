#ifndef __BRANCH_BOUND_H__
#define __BRANCH_BOUND_H__


#include "graph/graph.h"


struct BranchNBoundStruct
{
	Graph graph;
	std::set<int> remainedElements;
	std::vector<int> circuit;
	float cost;

	//Default constructor
	BranchNBoundStruct();
	//Constructor
	BranchNBoundStruct(Graph graph, std::set<int> remainedElements);
	//Copy constructor
	//BranchNBoundStruct(const BranchNBoundStruct& other);
	//Assignement operator
	//BranchNBoundStruct& BranchNBoundStruct::operator=(const BranchNBoundStruct& other);
	//Destructor
	//~BranchNBoundStruct();

};


inline bool operator >(const BranchNBoundStruct &A, const BranchNBoundStruct& B) {
	if (A.cost > B.cost) {
		return true;
	}
	else return false;
}





#endif