#ifndef VMT_H
#define VMT_H

#include <opencv2/core/core.hpp>

class Vmt
{
public:
    typedef /*long*/ long FrameIndex;
protected:

    int _width;
    int _height;
    int _depth;

    cv::SparseMat _sparseMat;

public:
    //constructor
    Vmt(cv::SparseMat sparseMat);

    //desctructor
    ~Vmt();

    int getWidth() const { return this->_width; }
    int getHeight() const { return this->_height; }
    int getDepth() const { return this->_depth; }
};

#endif // VMT_H
