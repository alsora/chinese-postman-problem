#include "branch_bound.h"



BranchNBoundStruct::BranchNBoundStruct()
{
}


BranchNBoundStruct::BranchNBoundStruct(Graph graph, std::set<int> remainedElements)
{

	this->graph = graph;
	this->circuit = std::vector<int>();
	this->cost = -1;
	this->remainedElements = remainedElements;

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