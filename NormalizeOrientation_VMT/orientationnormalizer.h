#ifndef ORIENTATIONNORMALIZER_H
#define ORIENTATIONNORMALIZER_H

#include <QList>

#include <opencv/vmt.h>

class OrientationNormalizer
{
public:
    OrientationNormalizer();

    QList<float> calculateRotationAngles(Vmt vmt);

    //there should be 3 angle values for around X, Y and Z axises. "1.0" means 1 radian (~57.3 degrees)
    Vmt rotateVmt(QList<float> angles, const Vmt& initialVmt);

    //there should be 3 offset values for X, Y and Z. "1.0" means 1 meter.
    Vmt translateVmt(QList<float> offsets, const Vmt& initialVmt);

};

#endif // ORIENTATIONNORMALIZER_H
