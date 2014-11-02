#include "orientationnormalizer.h"

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

OrientationNormalizer::OrientationNormalizer()
{
}

Vmt OrientationNormalizer::rotateVmt(QList<float> angles, const Vmt &initialVmt)
{
    if (angles.length() != 3)
    {
        std::cerr << "Angle number is not 3!!\n\n";
        return Vmt();
    }


    //Be carefull if you want to apply several rotations; rotations are not commutative !
    //This means than in most cases: rotA * rotB != rotB * rotA.

    //create transformation
    Eigen::Affine3f transformer = Eigen::Affine3f::Identity();

    if (angles[0] != 0.0) //translation around X axis
    {
        transformer.rotate(Eigen::AngleAxisf(angles[0], Eigen::Vector3f::UnitX()));
    }

    if (angles[1] != 0.0) //translation around Y axis
    {
        transformer.rotate(Eigen::AngleAxisf(angles[1], Eigen::Vector3f::UnitY()));
    }

    if (angles[2] != 0.0) //translation around Z axis
    {
        transformer.rotate(Eigen::AngleAxisf(angles[2], Eigen::Vector3f::UnitZ()));
    }


    //create a new cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = initialVmt.getPointCloud();

    //do the rotation
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transformer);

    Vmt transformedVmt(transformed_cloud);


    transformed_cloud.reset();
    //FIXME: should I clean source_cloud as well? MEMORY LEAK???
//    source_cloud.reset();


    return transformedVmt;
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

    Vmt transformedVmt(transformed_cloud);


    transformed_cloud.reset();
    //FIXME: should I clean source_cloud as well? MEMORY LEAK???
//    source_cloud.reset();


    return transformedVmt;
}
