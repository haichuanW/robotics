#include <iostream>
#include <vector>

#include "kdtree.h"

using namespace std;

int main(){
    vector<vector<int>> p = {{3, 6}, {17, 15}, {13, 15}, {6, 12}, 
                       {9, 1}, {2, 7}, {10, 19}}; 
    
    kdtree tree(p);
    Node* root = tree.buildTree();

    vector<int> p1 = {6,12};
    cout << tree.search(root,p1)<<endl;

    return 0;
}