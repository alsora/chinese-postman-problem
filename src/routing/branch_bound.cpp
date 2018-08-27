#include "branch_bound.h"



BranchNBoundStruct::BranchNBoundStruct()
{
}


BranchNBoundStruct::BranchNBoundStruct(Graph g, std::set<int> elements)
{

	graph = g;
	remainedElements = elements;

	cost = -1;
}

