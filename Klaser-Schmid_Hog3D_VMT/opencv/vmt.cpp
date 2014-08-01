#include "vmt.h"

// std
#include <stdexcept>

// my stuff
#include <opencv/IplImageWrapper.h>
//#include <opencv/functions.h>
//#include <numeric/functions.hpp>

const bool DEBUGOUT = false;

Vmt::Vmt(cv::SparseMat sparseMat)
{
    _width = 0;
    _height = 0;
    _depth = 0;

    _sparseMat = sparseMat;

    if (_sparseMat.nzcount() > 0)
    {
        //FIXME: check the indices..
        _width  = _sparseMat.size()[0];
        _height = _sparseMat.size()[1];
        _depth  = _sparseMat.size()[2];
    }
}

Vmt::~Vmt()
{
    //TODO: clean up
    _width = 0;
    _height = 0;
    _depth = 0;
}
