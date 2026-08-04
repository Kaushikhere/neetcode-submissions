class Solution {
public:
    bool isSubsequence(string s, string t) {
        
        int n = t.size();

        int sp = 0;
        int tp = 0;

        while(tp < n) {
            if(s[sp] == t[tp]) {
                sp++;
                tp++;
            }
            else {
                tp++;
            }
        }
        if(sp == s.size()) {
            return true;
        }
    return false;
    }
};