
#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <vector>
#include <iostream>
#include <algorithm>
#include <set>


std::vector<std::vector<int>> allAssignments(int N, int K);

std::vector<std::vector<int>> allPermutations(int N, int K);

void compute(std::set<int>& set, std::vector<int>& currentResults, std::vector<std::vector<int>>& results);



#endif