#ifndef OctreePoint_H
#define OctreePoint_H

#include "Vec3.h"

// Simple point data type to insert into the tree.
// Have something with more interesting behavior inherit
// from this in order to store other attributes in the tree.
template<typename coordinate_type>
class OctreePoint {
	Vec3<coordinate_type> position; 
public:
	OctreePoint(){}
	OctreePoint(const Vec3<coordinate_type>& position) : position(position) { }
	inline const Vec3<coordinate_type>& getPosition() const { return position; }
	inline void setPosition(const Vec3<coordinate_type>& p) { position = p; }
};

#endif
