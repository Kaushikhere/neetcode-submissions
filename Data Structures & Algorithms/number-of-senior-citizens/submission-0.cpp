class Solution {
public:
    int countSeniors(vector<string>& details) {
        
        int elder = 0;

        int n = details.size();

        for(int i = 0; i < n; i++) {
            string s = details[i];
            string age = "";
            for(int j = 11; j < 13; j++) {
                age.push_back(s[j]);
            }

            int a = stoi(age);

            if (a > 60) {
                elder++;
            }
        }
return elder;
    }
};