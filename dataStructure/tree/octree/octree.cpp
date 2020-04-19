/***********************************************************************
 * Software License Agreement (BSD License)
 * 
 * Created on Sun April 19 2020
 * Basical function to create octree
 * author Haichuan Wang
 *************************************************************************/

#include <iostream>
#include <vector>

using namespace std;

struct Point{
    int x;
    int y;
    int z;
    Point():x(-1),y(-1),z(-1){}
    Point(int a,int b,int c):x(a),y(b),z(c){}
};

// Octree class 
class Octree{
    Point* point;
    // topLeftFront point with minimal value; bottomRightBack point with maximal value
    Point *topLeftFront,*bottomRightBack;
    vector<Octree*> children;

public:
    Octree(){
        // point are (-1,-1,-1);
        point = new Point();
    }
    // initialize octree with constructor
    Octree(int x,int y,int z){
        point = new Point(x,y,z);
    }
    //initialize octree topLeftFront and bottomRightBack points
    Octree(int x1,int y1,int z1,int x2,int y2,int z2){
        if(x2<x1 || y2<y1 || z2<z1){
            cout << "boundary point is invalid" << endl;
            return;
        }
        point = nullptr;
        topLeftFront = new Point(x1,y1,z1);
        bottomRightBack = new Point(x2,y2,z2);
        children.assign(8,nullptr);
        for(int i=0;i<=7;++i){
            children[i] = new Octree();
        }
    }

    void insert(int x,int y,int z){
        if(find(x,y,z)){
            cout << "point already in the tree!\n";
            return;
        }
        if(x<topLeftFront->x || x>bottomRightBack->x || y<topLeftFront->y || y>bottomRightBack->y || z<topLeftFront->z || z>bottomRightBack->z){
            cout << "point is out of bound"<<endl;
            return;
        }
        int midx = (topLeftFront->x + bottomRightBack->x)/2;
        int midy = (topLeftFront->y + bottomRightBack->y)/2;
        int midz = (topLeftFront->z + bottomRightBack->z)/2;

        int pos = (x>midx) + 2*(y>midy)+4*(z>midz);
        if(children[pos]==nullptr){
            children[pos]->insert(x,y,z);
            return;
        }else if(children[pos]->point->x == -1){
            delete children[pos];
            children[pos] = new Octree(x,y,z);
            return;
        }else{
            int x_ = children[pos]->point->x;
            int y_ = children[pos]->point->y;
            int z_ = children[pos]->point->z;
            delete children[pos];
            children[pos]=nullptr;
            if(pos==0){
                children[pos] = new Octree(topLeftFront->x,topLeftFront->y,topLeftFront->z,midx,midy,midz);
            }else if(pos==1){
                children[pos] = new Octree(midx+1,topLeftFront->y,topLeftFront->z,bottomRightBack->x,midy,midz);
            }else if(pos==2){
                children[pos] = new Octree(topLeftFront->x,midy+1,topLeftFront->z,midx,bottomRightBack->y,midz);
            }else if(pos==3){
                children[pos] = new Octree(midx+1,midy+1,topLeftFront->z,bottomRightBack->x,bottomRightBack->y,midz);
            }else if(pos==4){
                children[pos] = new Octree(topLeftFront->x,topLeftFront->y,midz+1,midx,midy,bottomRightBack->z);
            }else if(pos==5){
                children[pos] = new Octree(midx+1,topLeftFront->y,midz+1,bottomRightBack->x,midy,bottomRightBack->z);
            }else if(pos==6){
                children[pos] = new Octree(topLeftFront->x,midy+1,midz+1,midx,bottomRightBack->y,bottomRightBack->z);
            }else if(pos==7){
                children[pos] = new Octree(midx+1,midy+1,midz+1,bottomRightBack->x,bottomRightBack->y,bottomRightBack->z);
            }

            children[pos]->insert(x_,y_,z_);
            children[pos]->insert(x,y,z);
        }
    }

    bool find(int x,int y,int z){
        if(x<topLeftFront->x || x>bottomRightBack->x || y<topLeftFront->y || y>bottomRightBack->y || z<topLeftFront->z || z>bottomRightBack->z){
            return 0;
        }
        int midx = (topLeftFront->x + bottomRightBack->x)/2;
        int midy = (topLeftFront->y + bottomRightBack->y)/2;
        int midz = (topLeftFront->z + bottomRightBack->z)/2;

        int pos = (x>midx) + 2*(y>midy)+4*(z>midz);

        // if an internal node is encountered
        if(children[pos]->point==nullptr){
            return children[pos]->find(x,y,z);
        }

        // if node is empty
        else if(children[pos]->point->x ==-1){
            return 0;
        }else if(x==children[pos]->point->x && y==children[pos]->point->y && children[pos]->point->z){
            return 1;
        }
        return 0;
    }

};

int main(){
    Octree tree(1,1,1,5,5,5);

    tree.insert(1,2,3);
    tree.insert(1,2,3);
    tree.insert(3,4,5);
    tree.insert(5,4,6);

    cout << (tree.find(2,3,4)?"found":"not found")<<endl;
    cout << (tree.find(3,4,5)?"found":"not found")<<endl;
    
    return 0;
}