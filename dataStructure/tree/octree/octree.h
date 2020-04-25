/***********************************************************************
 * Software License Agreement (BSD License)
 * 
 * Created on Sun April 23 2020
 * Basical function to create octree
 * 
 * author Haichuan Wang
 *************************************************************************/

#ifndef _OCTREE_H_
#define _OCTREE_H_

#include <cassert>
#include "Vec3.h"
#include "OctreePoint.h"

template<typename coordinate_type>
std::ostream& operator<<(std::ostream& out, Vec3<coordinate_type>& v){
    out << '(';
    for (size_t i = 0; i < 3; ++i){
        if (i > 0)
            out << ",";
        out << v[i];
    }
    out << ')';
    return out;
}



template<typename coordinate_type>
class Octree{
private:
    Vec3<coordinate_type> origin;
    Vec3<coordinate_type> halfDim;
    Octree* children[8];
    OctreePoint<coordinate_type> *root;

public:
    Octree(Vec3<coordinate_type> origin_,Vec3<coordinate_type> halfDim_):origin(origin_),halfDim(halfDim_),root(NULL){
        for(size_t i=0;i<8;++i) children[i] = nullptr;
    }

    ~Octree(){
        for(size_t i=0;i<8;++i) delete children[i];
    }

    bool isLeafNode() const{return children[0]==NULL;}

    int getOctreePos(const Vec3<coordinate_type> &point) const{
        return 4*(point.x >= origin.x) + 2*(point.y >= origin.y) + (point.z >= origin.z);
    }

    void insert(OctreePoint<coordinate_type>* point){
        if(isLeafNode()){
            if(root==NULL){
                root = point;
                return;
            }else{
 				OctreePoint<coordinate_type> *oldPoint = root;
				root = NULL;
				for(int i=0; i<8; ++i) {
					// Compute new bounding box for this child
					Vec3<coordinate_type> newOrigin = origin;
					newOrigin.x += halfDim.x * (i&4 ? .5f : -.5f);
					newOrigin.y += halfDim.y * (i&2 ? .5f : -.5f);
					newOrigin.z += halfDim.z * (i&1 ? .5f : -.5f);
					children[i] = new Octree(newOrigin, halfDim*.5f);
				}
				children[getOctreePos(oldPoint->getPosition())]->insert(oldPoint);
				children[getOctreePos(point->getPosition())]->insert(point);
            }
        }else{
            children[getOctreePos(point->getPosition())]->insert(point);
        }

    }

    bool insideCube(const Vec3<coordinate_type>& p){
        if(p.x > origin.x + halfDim.x || p.x < origin.x - halfDim.x) return false;
        if(p.y > origin.y + halfDim.y || p.y < origin.y - halfDim.y) return false;
        if(p.z > origin.z + halfDim.z || p.z < origin.z - halfDim.z) return false;
        return true; 
    }

    void knnSearch(const Vec3<coordinate_type>& p,double &best_dist_){
        if(isLeafNode()){
            // std::cout <<"origin: "<<origin << std::endl;
            // std::cout <<"halfDim: "<<halfDim << std::endl;
            best_dist_ = Vec3<coordinate_type>(p.x>origin.x?origin.x+halfDim.x:origin.x-halfDim.x,
                            p.y>origin.y?origin.y+halfDim.y:origin.y-halfDim.y,
                            p.z>origin.z?origin.z+halfDim.z:origin.z-halfDim.z).distance(p);
        }else{
            for(size_t i=0;i<8;++i){
                if(children[i]->insideCube(p)){
                    children[i]->knnSearch(p,best_dist_);
                }
            }
        }
    }


};

#endif