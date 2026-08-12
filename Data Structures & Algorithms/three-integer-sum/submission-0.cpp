class Solution {
public:
    vector<vector<int>> threeSum(vector<int>& nums) {
        int n = nums.size();
        int i = 0;

        sort(nums.begin(), nums.end());

        vector<vector<int>> res;

    for(int i = 0; i < nums.size(); i++) {

    // Skip duplicate i
    if (i > 0 && nums[i] == nums[i-1])
        continue;

        int j = i+1;
        int k = nums.size()-1;

        while(j < k) {

            if(nums[i] + (nums[j]) + nums[k] == 0) {
              res.push_back({nums[i], nums[j], nums[k]});
              j++;
              k--;
                          // Skip duplicate j
            while (j < k && nums[j] == nums[j-1])
                j++;

            // Skip duplicate k
            while (j < k && nums[k] == nums[k+1])
                k--;
        }
            

            if (nums[i] + nums[j] + nums[k] < 0) {
                j++;
            }

            if (nums[i] + nums[j] + nums[k] > 0) {
                k--;
            }
    } 
        }
            return res;
    }

};

