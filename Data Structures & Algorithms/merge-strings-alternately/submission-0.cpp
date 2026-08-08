class Solution {
public:
    string mergeAlternately(string word1, string word2) {

        int w1 = word1.size();
        int w2 = word2.size();

        string res = "";

        int i = 0;
        int j = 0;

        while( i < w1 && j < w2) {
            res.push_back(word1[i]);
            i++;
            res.push_back(word2[j]);
            j++;

        }

        while (i < w1) {
            res.push_back(word1[i]);
            i++;
        }

        while (j < w2) {
            res.push_back(word2[j]);
            j++;
        }

        return res;
        
    }
};