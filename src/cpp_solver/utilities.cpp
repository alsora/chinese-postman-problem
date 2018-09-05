#include "utilities.h"





std::vector<std::vector<int>> allAssignments(int N, int K)
{   

    //Current implementation works only with N even and K = 2 

    std::vector<std::vector<int>> output;

    std::set<int> numbers;

    for (int i = 0;i < N; i++){
        numbers.insert(i);
    }


    std::vector<int> currentResult;

    compute(numbers, currentResult, output);


    return output;
}



void compute(std::set<int>& set, std::vector<int>& currentResults, std::vector<std::vector<int>>& results)
{

    if (set.size() < 2){
        //std::copy (currentResults.begin(), currentResults.end(), std::back_inserter(results));
        results.push_back(currentResults);
        return;
    }

    std::vector<int> list;
    list.assign(set.begin(), set.end());

    int first = list.back();
    list.pop_back();
    for (int i = 0; i < list.size(); i ++){

        int second = list[i];
        std::set<int> nextSet(std::make_move_iterator(list.begin()),
              std::make_move_iterator(list.end()));
        
        nextSet.erase(second);

        currentResults.push_back(first);
        currentResults.push_back(second);

        compute(nextSet, currentResults, results);

        currentResults.pop_back();
        currentResults.pop_back();
    }

}

float vector_mean(std::vector<int> v)
{

    float mean = accumulate( v.begin(), v.end(), 0.0)/v.size(); 

    return mean;
}


float vector_std_dev(std::vector<int> v)
{

    float mean = vector_mean(v);

    float acc = 0;
    for( int n = 0; n < v.size(); n++ ){
        acc += (v[n] - mean) * (v[n] - mean);
    }
    acc /= v.size();

    float std = sqrt(acc);

    return std;

}