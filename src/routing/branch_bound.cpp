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

/*
BranchNBoundStruct & BranchNBoundStruct::operator=(const BranchNBoundStruct & other)
{
	if (this == &other) {
		return *this;
	}


	this->graph = other.graph;
	this->circuit = other.circuit;
	this->cost = other.cost;
	this->remainedElements = other.remainedElements;


	return *this;
}
*/

/*
bool BranchNBoundStruct::LowerCostOtp(const int& lhs, const int& rhs) const
{

	return graph.edge(lhs)->cost() < graph.edge(rhs)->cost();

}
*/
