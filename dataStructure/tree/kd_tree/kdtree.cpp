#include "kdtree.h"

kdtree::kdtree(std::vector<std::vector<int>> points){
    this->points = points;
}

// build kdtree
Node* kdtree::buildTree(){
    // if points is empty,return NULL
    if(this->points.size()==0)return NULL;
    // create root with first elements
    Node* root = new Node(this->points[0]);
    // insert each elements to kdtree
    for(int i=1;i<this->points.size();++i){
        insert(root,points[i]);
    }
    return root;
}

// assign first one with depth 0
void kdtree::insert(Node* root,std::vector<int> point){
    insert(root,point,0);
}
// insert elements to tree
void kdtree::insert(Node* root,std::vector<int> point,std::size_t depth){
    size_t cd = depth / point.size();

    if(point[cd] < (root->point[cd])){
        if(root->left) insert(root->left,point,depth+1);
        else root->left = new Node(point);
    }else if(point[cd] > (root->point[cd])){
        if(root->right) insert(root->right,point,depth+1);
        else root->right = new Node(point);
    }else{
        std::cout << "dim " << cd << "conflicts with "<<point[cd] << std::endl;
        return;
    }
}

bool kdtree::search(Node *root,std::vector<int> point){
    return search(root,point,0);
}

// check whether two vector are same
bool kdtree::isEqual(const std::vector<int> &p1,const std::vector<int> &p2){
    return (p1.size()==p2.size() && std::equal(p1.begin(),p1.end(),p2.begin()));
}

// search node
bool kdtree::search(Node *root,std::vector<int> point,std::size_t depth){
    if(root==NULL)return false;
    if(isEqual(root->point,point))return true;

    size_t cd = depth / point.size();

    if(point[cd] < (root->point[cd])){
        return search(root->left,point,depth+1);
    }else{
        return search(root->right,point,depth+1);
    }
}