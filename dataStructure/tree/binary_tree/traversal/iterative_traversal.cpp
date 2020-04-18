#include <iostream>
#include <vector>
#include <stack>

using namespace std;

// TreeNode
struct TreeNode{
    int val;
    TreeNode *left;
    TreeNode *right;
    TreeNode(int x):val(x),left(NULL),right(NULL){}
};

// interatively insert elements 
void Insert(TreeNode *root,int value){
    TreeNode *temp = root;
    while(1){
        if(value > temp->val){
            if(temp->right==NULL){
                temp->right = new TreeNode(value);
                return;
            }else{ 
                temp = temp->right;
            }
        }else{
            if(temp->left==NULL){
                temp->left = new TreeNode(value);
                return;
            }else{
                temp = temp->left;
            }
        }
    }
}

// build binary tree
TreeNode* BuildTree(vector<int> v){
    if(v.size()==0)return NULL;
    TreeNode *root = new TreeNode(v[0]);
    for(int i=1;i<v.size();++i){
        Insert(root,v[i]);
    }
    return root;
}


// inorder traversal
void inorder(TreeNode *root){
    if(root==NULL)return;
    TreeNode *current = root;
    stack<TreeNode*> stk;

    while(current!=NULL || !stk.empty()){
        while(current!=NULL){
            stk.push(current);
            current = current->left;
        }
        current = stk.top();
        stk.pop();
        cout << current->val << "\n";

        current = current->right;
    }
}

//preorder
void preorder(TreeNode *root){
    if(root==NULL)return;
    stack<TreeNode*> stk;
    stk.push(root);
    while(stk.empty()==false){
        root = stk.top();
        cout << root->val << "\n";
        stk.pop();
        if(root->right){
            stk.push(root->right);
        }
        if(root->left){
            stk.push(root->left);
        }
    }
}

//postorder traversal
void postorder(TreeNode *root){
    stack<TreeNode*> stk;
    stk.push(root);

    stack<int> out;
    while(!stk.empty()){
        TreeNode *current = stk.top();
        stk.pop();

        out.push(current->val);

        if(current->left)stk.push(current->left);
        if(current->right)stk.push(current->right);
    }

    while(!out.empty()){
        cout << out.top() << "\n";
        out.pop();
    }
}

int main(){
    vector<int> v{4,1,2,3,5,7,6,8};
    TreeNode *root = BuildTree(v);

    // inorder(root);
    // preorder(root);
    postorder(root);

    return 0;
}