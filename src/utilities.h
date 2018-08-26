
#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <vector>
#include <iostream>
#include <algorithm>
#include <set>


std::vector<std::vector<int>> allAssignments(int N, int K);

void compute(std::set<int>& set, std::vector<int>& currentResults, std::vector<std::vector<int>>& results);

float vector_mean(std::vector<int> v);

float vector_std_dev(std::vector<int> v);

#endif