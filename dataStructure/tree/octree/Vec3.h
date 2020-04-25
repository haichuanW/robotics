#ifndef _VEC3_H_
#define _VEC3_H_

#include <cmath>

template<typename coordinate_type>
class Vec3{
private:
public:
	coordinate_type x,y,z;
	
	Vec3(){}
	Vec3(coordinate_type x_,coordinate_type y_,coordinate_type z_):x(x_),y(y_),z(z_){}
	Vec3<coordinate_type> operator+(const Vec3 &v) const{
		return Vec3(x+v.x,y+v.y,z+v.z);
	}

	Vec3<coordinate_type> operator-(const Vec3 &v) const{
		return Vec3(x-v.x,y-v.y,z-v.z);
	}

	Vec3<coordinate_type> operator*(const coordinate_type r) const{
		return Vec3(r*x,r*y,r*z);
	}

	Vec3<coordinate_type> operator/(const coordinate_type r) const{
		return Vec3(x/r,y/r,z/r);
	}

	bool operator==(const Vec3<coordinate_type> &r) const{
		return (x==r.x && y==r.y && z==r.z);
	}

	bool operator!=(const Vec3<coordinate_type> &r) const{
		return (x!=r.x || y!=r.y || z!=r.z);
	}

	Vec3<coordinate_type> cmul(const Vec3 &r) const{
		return Vec3(x*r.x,y*r.y,z*r.z);
	}

	Vec3<coordinate_type> cdiv(const Vec3& r) const{
		return Vec3(x/r.x,y/r.y,z/r.z);
	}

	coordinate_type norm(const Vec3& r) const{
		return sqrt( r.x*r.x + r.y*r.y + r.z*r.z);
	}

	coordinate_type squaredNorm(const Vec3& r) const{
		return r.x*r.x + r.y*r.y + r.z*r.z;
	}

	Vec3<coordinate_type> normalized() const{
		return *this/norm();
	}

	coordinate_type operator[](size_t index){
		if(index==0)return x;
		else if(index==1) return y;
		else if(index==2) return z;
		else return coordinate_type(0);
	}

	double distance(const Vec3<coordinate_type>& r)const{
		return sqrt((x-r.x)*(x-r.x)+(y-r.y)*(y-r.y)+(z-r.z)*(z-r.z));
	}
};



#endif
