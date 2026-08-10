/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 */

class Solution {
public:
    bool hasCycle(ListNode* head) {

        if(head == nullptr || head -> next == nullptr) return false;
        
        ListNode* A = head;  // The slower one
        ListNode* B = head;  // The faster one

        while(B != nullptr && B-> next != nullptr) {

            A = A-> next;
            B = B->next->next;

            if(A == B) {
                return true;
            }
        }
        return false;
    }
};
