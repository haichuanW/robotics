#include <iostream>
#include <vector>

using namespace std;

// TreeNode
struct TreeNode{
    int val;
    TreeNode *left;
    TreeNode *right;
    TreeNode(int x):val(x),left(NULL),right(NULL){}
};

// insert element to tree
void Insert(TreeNode *root,int value){
    if(value > root->val){
        // if current node is NULL, insert to current node,otherwises continue
        if(root->right==NULL)root->right = new TreeNode(value);
        else Insert(root->right,value);
    }else{
        if(root->left==NULL)root->left = new TreeNode(value);
        else Insert(root->left,value); 
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
    inorder(root->left);
    cout << root->val << "\n";
    inorder(root->right);
}

// preorder traversal
void preorder(TreeNode *root){
    if(root==NULL)return;
    cout << root->val << endl;
    preorder(root->left);
    preorder(root->right);
}

// postorder traversal
void postorder(TreeNode *root){
    if(root==NULL)return;
    postorder(root->left);
    postorder(root->right);
    cout << root->val << endl;
}

int main(){
    vector<int> v{4,1,2,3,5,7,6,8};
    TreeNode *root = BuildTree(v);

    // inorder(root);
    // preorder(root);
    postorder(root);

    return 0;
}