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

std::vector<std::vector<int>> allPermutations(int N, int K)
{	
    std::vector<std::vector<int>> output;
    /*
    std::string bitmask(K, 1); // K leading 1's
    bitmask.resize(N, 0); // N-K trailing 0's
    
    do {
        int combo[K];
        int pos = 0;
        for (int i = 0; i < N; ++i) // [0..N-1] integers
        {
            if (bitmask[i]){
                combo[pos] = i;
                pos++;
            }
        }
        output.push_back(combo);
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
    */

    return output;

}