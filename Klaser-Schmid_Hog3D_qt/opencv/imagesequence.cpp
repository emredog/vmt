#include "imagesequence.h"

// std
#include <stdexcept>

// my stuff
#include <opencv/IplImageWrapper.h>
//#include <opencv/functions.h>
//#include <numeric/functions.hpp>

const bool DEBUGOUT = false;



IplImageWrapper *ImageSequence::currentBuffer() const
{
    return _currentFrame;
}

void ImageSequence::setCurrentBuffer(IplImageWrapper *currentBuffer)
{
    _currentFrame = currentBuffer;
}
void ImageSequence::init()
{
    // prepare...
    QStringList filters;
    if (DEPTH == 1)
        filters << "*.jp2";
    else
        filters << "*.jpg" << "*.png" << "*.tiff";
    // ...and get all "jp2" (depth) files, ordered by name
    this->imageFiles = this->directory.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

    // throw an error if something went wrong
    if (this->imageFiles.isEmpty())
        throw std::runtime_error("Error initializing image sequence!");

    // get width and height of images
    // TODO
//    if (DEBUGOUT) std::cout << "### Width: " << this->_width <<  " Height: " << this->_height << std::endl;

    // get the first frame, then we know the starting time of the first frame
    //if (DEBUGOUT) std::cout << "### startPts: " << _startPts << std::endl;
    this->advance(0);
    //_startPts = _currentFrameIndex;
//    if (DEBUGOUT) std::cout << "### startPts: " << _startPts << std::endl;
}

ImageSequence::~ImageSequence()
{
    this->cleanup();
}

const IplImageWrapper ImageSequence::getFrame() const
{
    if (this->_currentFrame)
        return *(this->_currentFrame);
    else
        return IplImageWrapper();
}

bool ImageSequence::nextFrame()
{
    return this->advance(this->_currentFrameIndex + 1);
}

bool ImageSequence::prevFrame()
{
    return this->advance(this->_currentFrameIndex - 1);
}

bool ImageSequence::seek(ImageSequence::FrameIndex frame)
{
    return this->advance(frame);
}

bool ImageSequence::probeFrame(ImageSequence::FrameIndex frame)
{
    return this->advance(frame);
}



void ImageSequence::cleanup()
{
    // free all allocated objects
//    if (this->_peekBuffer)
//        delete this->_peekBuffer;
    if (this->_currentFrame)
        delete this->_currentFrame;

    // reset internal variables
//    _startPts = 0;
    _currentFrameIndex = 0;
//    _peekFrameIndex = 0;
    _width = 0;
    _height = 0;

    this->imageFiles.clear();
}


bool ImageSequence::advance(FrameIndex frameIndex)
{
    int frame = static_cast<int>(frameIndex);
    // I don't check "whether we need to retrieve the next frame", I simply retrieve it
    QString currentFile;
    if (frame < this->imageFiles.size())
    {
        currentFile = this->directory.absoluteFilePath(this->imageFiles[frame]);
        this->_currentFrameIndex = static_cast<FrameIndex>(frame);
    }
    else
    {
        return false;
    }
//    frame++;

//    if (frame < this->imageFiles.size())
//    {
//        peekFile = this->imageFiles[frame];
//        this->_peekFrameIndex = frame;
//    }


    // if allocated, delete it.
    if (this->_currentFrame)
    {
        delete this->_currentFrame;
        this->_currentFrame = NULL;
    }

    // create a new image
    if (DEPTH == 1)
    {
        this->_currentFrame = new IplImageWrapper(currentFile.toStdString(), CV_16UC1); // 16 bit depth images (jp2)
    }
    else
    {
        if (CHANNELS == 1)
            this->_currentFrame = new IplImageWrapper(currentFile.toStdString(), CV_8UC1); // 8 bit grayscale images (png/jpg/tiff)
        else if (CHANNELS == 3)
            this->_currentFrame = new IplImageWrapper(currentFile.toStdString(), CV_8UC3); // 8 bit RGB images (png/jpg/tiff)
    }

    this->_width = this->_currentFrame->getWidth();
    this->_height = this->_currentFrame->getHeight();

    return true;
}

