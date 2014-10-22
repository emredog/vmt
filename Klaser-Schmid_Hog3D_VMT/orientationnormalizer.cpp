#include "orientationnormalizer.h"

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

OrientationNormalizer::OrientationNormalizer()
{
}

QList<float> OrientationNormalizer::calculateRotationAngles(Vmt vmt)
{
    //TODO
    //.....

    return QList<float>() << 0.0 << 0.0 << 0.0;
}

Vmt OrientationNormalizer::rotateVmt(QList<float> angles, const Vmt &initialVmt)
{
    if (angles.length() != 3)
    {
        std::cerr << "Angle number is not 3!!\n\n";
        return Vmt();
    }
    //TODO
    //.....

    return Vmt();
}

Vmt OrientationNormalizer::translateVmt(QList<float> offsets, const Vmt &initialVmt)
{
    if (offsets.length() != 3)
    {
        std::cerr << "Offset number is not 3!!\n\n";
        return Vmt();
    }

    //create transformation
    Eigen::Affine3f transformer = Eigen::Affine3f::Identity();
    transformer.translation() << offsets[0] , offsets[1] , offsets[2];

    //create a new cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = initialVmt.getPointCloud();

    //do the translation
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transformer);

    Vmt transformedVmt(transformed_cloud, initialVmt.getWidth(), initialVmt.getHeight(), initialVmt.getDepth());


    transformed_cloud.reset();
    //FIXME: should I clean source_cloud as well? MEMORY LEAK???
//    source_cloud.reset();


    return Vmt();
}
