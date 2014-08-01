#ifndef BOX3D_H_
#define BOX3D_H_

struct Box3D
{
    double x, y, z;
    double width, height, depth;
	
    Box3D(double x_ = 0, double y_ = 0, double z_ = 0, double width_ = 0, double height_ = 0, double depth_ = 0)
        : x(x_), y(y_), z(z_), width(width_), height(height_), depth(depth_)
	{ }
	
	virtual ~Box3D()
	{ }
};

#endif /*BOX3D_H_*/
