#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <QString>
#include <QPoint>
#include <qhash.h>
#include <QDebug>

#include <opencv2/core/core.hpp>

#include <iostream>


#define MIN_Z 10
#define MAX_Z 8000

inline uint qHash(const QPoint& r)
{
    return qHash(QString("%1,%2").arg(r.x()).arg(r.y()));
}

inline float raw_depth_to_meters(int raw_depth)
{
    if (raw_depth < 2047)
        return 1.0 / (raw_depth * -0.0030711016 + 3.3309495161);
    return 0;
}

//parse depth image and fetch all various depth values for each x, y coordinates
QHash<QPoint, QList<unsigned int> > parseDepthImages(const QList<cv::Mat>& depthImages)
{
    cv::Mat temp;

    QHash<QPoint, QList<unsigned int> > pointToDepthList;

    cv::Mat sum(480, 640, CV_16UC1, cv::Scalar(0));

    foreach(cv::Mat image, depthImages)
    {
        //format issues
        image.convertTo(temp, CV_16UC1);
        image.release();
        temp.assignTo(image);
        temp.release();


        cv::bitwise_and(image, cv::Scalar(65504), image);
        cv::divide(image, cv::Scalar(32), image);

        cv::add(sum, image, sum);

        qDebug() << "Image parsed.";
        image.release();
    }

    double scale = 1.0 / (double)depthImages.length();


    cv::Mat mean(480, 640, CV_32FC1);

    mean = sum * scale;

    std::cout << "Mean = " << std::endl << mean << std::endl << std::endl;



}




#endif // HELPERFUNCTIONS_H
