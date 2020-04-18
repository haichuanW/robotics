#include <iostream>
#include <vector>

using namespace std;

// 2d dimensional points
const int k = 2;

struct Node{
    vector<int> point;
    Node *left,*right;
    Node(vector<int> arr):point(arr),left(NULL),right(NULL){}
};


// insert point to kdtree
void insertRec(Node* root,vector<int> point,unsigned depth){
    unsigned cd = depth % k;
    if(point[cd] < (root->point[cd])){
        if(root->left==NULL)root->left = new Node(point);
        else insertRec(root->left,point,depth+1);
    }else{
        if(root->right==NULL)root->right = new Node(point);
        else insertRec(root->right,point,depth+1);
    }
}

// same as binary tree,just choose different dimensional each time
void insert(Node* root,vector<int> point){
    insertRec(root,point,0);
}

// build kdtree
Node* buildTree(vector<vector<int>> p){
    if(p.size()==0)return NULL;
    Node* root = new Node(p[0]);
    for(int i=1;i<p.size();++i){
        insert(root,p[i]);
    }
    return root;
}

//inorder traverse tree
void inorder(Node* root){
    if(root==NULL)return;
    inorder(root->left);
    cout << "("<< root->point[0] << ","<<root->point[1]<<")\n";
    inorder(root->right);
}

// check whether two vector are equal
bool isEqual(const vector<int> &p1,const vector<int> &p2){
    return (p1.size()==p2.size() && std::equal(p1.begin(),p1.end(),p2.begin()));
}

// recursive search kdtree
bool searchRec(Node* root,vector<int> p,unsigned depth){
    if(root==NULL)return false;
    if(isEqual(p,root->point))return true;
    unsigned cd = depth % k;
    if(p[cd] < (root->point[cd])){
        return searchRec(root->left,p,depth+1);
    }else{
        return searchRec(root->right,p,depth+1);
    }
}

bool search(Node* root,vector<int> p){
    return searchRec(root,p,0);
}

int main(){
    vector<vector<int>> p = {{3, 6}, {17, 15}, {13, 15}, {6, 12}, 
                       {9, 1}, {2, 7}, {10, 19}}; 
    
    Node* root = buildTree(p);
    
    // inorder(root);

    vector<int> p1 = {6,12};
    cout << search(root,p1)<<endl;

    return 0;
}