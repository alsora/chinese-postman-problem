#include "utilities.h"





std::vector<std::vector<int>> allAssignments(int N, int K)
{
    std::vector<std::vector<int>> output;
    std::vector<int> index_pair;

    int q = (int) N/K;
    int d = N % K;

    for (int c = q; c > 0; c --){
        for (int j = 0; j < K; j ++){
            index_pair.push_back(c);
        }
    }
    for (int j = 0; j < d; j++){
        index_pair.push_back(0);
    }


    do {
        std::size_t pair[3] = {};

        for (std::size_t i = 0u;  i < index_pair.size();  ++i)
            pair[index_pair[i]] += 1u << (i+1);

        if (pair[1] > pair[2]) {
            output.push_back(index_pair);
        }
    } while (std::prev_permutation(index_pair.begin(), index_pair.end()));
    

    return output;
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