/***********************************************************************
 * Software License Agreement (BSD License)
 * 
 * Created on Sun April 20 2020
 * Basical function to create kdtree
 * author Haichuan Wang
 *************************************************************************/

#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <iostream>

struct Node{
    std::vector<int> point;
    Node *right,*left;
    Node(std::vector<int> arr):point(arr),left(NULL),right(NULL){}
};

class kdtree{
private:
    std::vector<std::vector<int>> points;
public:
    kdtree(std::vector<std::vector<int>> points);

    Node *buildTree();

    //insert node to kdtree
    void insert(Node *root,std::vector<int> point);
    void insert(Node *root,std::vector<int> point,std::size_t depth);

    // kd tree search element
    bool search(Node *root,std::vector<int> point);
    bool search(Node *root,std::vector<int> point,std::size_t depth);

    // check whether two vector are equal
    bool isEqual(const std::vector<int> &p1,const std::vector<int> &p2);

};


#endif