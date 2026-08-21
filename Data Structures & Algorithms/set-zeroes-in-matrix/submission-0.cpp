class Solution {
public:
    void setZeroes(vector<vector<int>>& matrix) {

        if (matrix.empty()) return;

        int rows = matrix.size();
        int cols = matrix[0].size();

        vector<pair<int,int>> mp;

        // Find all original zeros
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {
                if(matrix[i][j] == 0) {
                    mp.push_back({i, j});
                }
            }
        }

        // Zero corresponding rows and columns
        for(auto p : mp) {

            int row = p.first;
            int column = p.second;

            for(int j = 0; j < cols; j++) {
                matrix[row][j] = 0;
            }

            for(int i = 0; i < rows; i++) {
                matrix[i][column] = 0;
            }
        }
    }
};