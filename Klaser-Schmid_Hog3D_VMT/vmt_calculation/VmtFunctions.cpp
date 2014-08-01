#include "VmtFunctions.h"
#include "boundingbox.h"
#include "PointCloudFunctions.h"


#include <stdio.h>

#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDir>
#include <QVector>
#include <QtDebug>

#include <opencv2/opencv.hpp> //FIXME: is this necessary? we already have the includes on header file...
#include <opencv2/highgui/highgui.hpp>


VmtFunctions::VmtFunctions(int xSize, int ySize)
//    : toleranceX(tolX), toleranceY(tolY), toleranceZ(tolZ)
{
    this->permittedMinZ = MIN_Z;
    this->permittedMaxZ = MAX_Z;

    this->matrixSize = new int[this->dims];


    //FIXME: is this working ok?
    this->matrixSize[Y] = ySize;
    this->matrixSize[X] = xSize;

    this->saveVolumeObject = false;
    this->saveDelta = false;
    this->saveVmt = false;

    this->downsampleRate = 1;

    this->isTrackPoint = false;

    this->matrixSize[Z] = this->permittedMaxZ - this->permittedMinZ;

    cout << "VMT core constructed with X: [0, " << xSize << "]\tY: [0, " << ySize << "]\tZ: [" << this->permittedMinZ << ", " << this->permittedMaxZ << "]\n";
    cout << "Normalization interval for depth range: [0, " << NORMALIZATION_INTERVAL << "]\n";
}


VmtFunctions::~VmtFunctions(void)
{
    delete[] matrixSize;
}


cv::SparseMat VmtFunctions::constructSparseVMT(QString videoFolderPath, QString trackFilePath)
{
    //read track file
    QFile trackFile(trackFilePath);
    if (!trackFile.open(QFile::ReadOnly))
        return cv::SparseMat();

    QList<BoundingBox> bboxSequence;

    //Parse track file:
    QTextStream reader(&trackFile);
    while (!reader.atEnd())
    {
        QString line = reader.readLine();
        QStringList parts = line.split(" ");

        BoundingBox bbox;
        bbox.frameNr    = parts[0].toInt();
        bbox.x          = parts[1].toInt();
        bbox.y          = parts[2].toInt();
        bbox.width      = parts[3].toInt();
        bbox.height     = parts[4].toInt();

        bboxSequence.append(bbox);
    }

    cout << "First bounding box (x, y, w, h) = (" << bboxSequence.first().x << ", "
                                                  << bboxSequence.first().y << ", "
                                                  << bboxSequence.first().width << ", "
                                                  << bboxSequence.first().height << ")\n";

    //get Depth image paths in the folder
    QStringList filters;
    filters << "*.jp2";
    QDir videoDir(videoFolderPath);
    QStringList depthImgFileNames = videoDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);


    cv::SparseMat prevSparseVolumeObj;
    QList<cv::SparseMat> volumeObjectDifferences;


    if (this->isTrackPoint)
        cout << "Tracking point: (" << this->trackX << ", " << this->trackY << ")\n";

    int counter = 1; //FIXME: delete this
    //calculate volume object for each depth frame
    foreach(BoundingBox bb, bboxSequence)
    {
        cout << counter << " -------------------------------------------------\n";
        QString fileName = depthImgFileNames[bb.frameNr-1];
        if (bb.frameNr >= depthImgFileNames.length() ||
                !fileName.contains(QString::number(bb.frameNr)))
            return cv::SparseMat();

        //Read the file
        cv::Mat depthImg = cv::imread(videoDir.absoluteFilePath(fileName).toStdString(), CV_LOAD_IMAGE_UNCHANGED);

        //Check for invalid input
        if(! depthImg.data )
        {
            cout <<  "Could not open or find the image: " << videoDir.absoluteFilePath(fileName).toStdString() << endl ;
            return cv::SparseMat();
        }

        //cut the bounding box if it's outside of the frame // FIXME--> this shouldn't happen?!
        if (bb.x + bb.width >= depthImg.cols) bb.width = depthImg.cols - bb.x;
        if (bb.y + bb.height >= depthImg.rows) bb.height = depthImg.rows - bb.y;

        //mask depth image with the bounding box
        cv::Rect roi(bb.x, bb.y, bb.width, bb.height);
        cv::Mat mask = cv::Mat(depthImg.rows, depthImg.cols, CV_8UC1, cv::Scalar(0));
        mask(roi) = cv::Scalar(1);
        cv::Mat maskedDepthImg = cv::Mat(depthImg.rows, depthImg.cols, depthImg.type(), cv::Scalar(0));
        depthImg.copyTo(maskedDepthImg, mask);

        mask.release();
        depthImg.release();

        //generate volume object
        cv::SparseMat currentSparseVolumeObj = this->generateSparseVolumeObject(maskedDepthImg, this->downsampleRate);

        maskedDepthImg.release();

        if (this->saveVolumeObject)
        {
            cv::SparseMat nor = this->spatiallyNormalizeSparseMat(currentSparseVolumeObj);
            cv::SparseMat trm = this->trimSparseMat(nor);
            PointCloudFunctions::saveVmtAsCloud(currentSparseVolumeObj, QString("/home/emredog/Documents/output/volObj_%1.pcd").arg(QString::number(counter).rightJustified(2, '0')).toStdString());
            nor.release();
            trm.release();
        }

        //do the subtraction
        if (prevSparseVolumeObj.nzcount() > 0) //there are at least 2 volume objects
        {
            cv::SparseMat delta = this->subtractSparseMat(currentSparseVolumeObj, prevSparseVolumeObj);
            //... and cleanup
            cv::SparseMat cleanedUpDelta = this->cleanUpVolumeObjectDifference(delta);
            delta.release();

            if (this->saveDelta)
            {
                cv::SparseMat nor = this->spatiallyNormalizeSparseMat(cleanedUpDelta);
                cv::SparseMat trm = this->trimSparseMat(nor);
                PointCloudFunctions::saveVmtAsCloud(trm, QString("/home/emredog/Documents/output/diff_%1.pcd").arg(QString::number(counter).rightJustified(2, '0')).toStdString());
                nor.release();
                trm.release();
            }

            volumeObjectDifferences.append(cleanedUpDelta);
        }

        currentSparseVolumeObj.copyTo(prevSparseVolumeObj);
        currentSparseVolumeObj.release();
        counter++;
    }

    cout << "\nCalculating VMT from volume object differences...\n";
    //construct the VMT based on volume object differences over the track file
    cv::SparseMat vmt = this->calculateVMT(volumeObjectDifferences);

    return vmt;
}

void VmtFunctions::setTrackPoint(int x, int y)
{
    if (x >= 0 && y >= 0)
    {
        this->isTrackPoint = true;
        this->trackX = x;
        this->trackY = y;
    }
    else
        this->isTrackPoint = false;
}

cv::SparseMat VmtFunctions::generateSparseVolumeObject(cv::Mat image, int downsamplingRate)
{
    cv::Mat temp;

    //format issues
    image.convertTo(temp, CV_16UC1);
    image.release();
    temp.assignTo(image);
    temp.release();

    //    cv::Mat median(image.dims, image.size, image.type());
    //    cv::medianBlur(image, median, 5);

    //Generate Volume Object:
    cv::SparseMat sparse_mat(this->dims, this->matrixSize, CV_8UC1);

    unsigned short pixelValue;
    unsigned short mostSig13Digits;
    unsigned short depth;
    unsigned int depthInMillimeters;


    //NOTE: because of memory restrictions, if downsamplingRate == 2, x and y are incremented by 2 (~downsampling by 1/4)
    for (int y = 0; y < image.rows; y+=downsamplingRate)
    {
        //FIXME!!!
        const unsigned short* ithRow = image.ptr<unsigned short>(y); //take a whole row
        //        const unsigned short* ithRow = median.ptr<unsigned short>(y); //take a whole row

        for (int x = 0; x < image.cols; x+=downsamplingRate)
        {
            pixelValue = ithRow[x];

            if (pixelValue <= 0) // it's set to zero when extracting silhouette. discard this point, this is background
                continue;


            mostSig13Digits = pixelValue & 65504; // (65504 = 1111 1111 1110 0000)
            depth = mostSig13Digits >> 5;

            depthInMillimeters = (unsigned int)(raw_depth_to_meters((int)depth)*1000);

            if (depthInMillimeters >= this->permittedMinZ && depthInMillimeters <= this->permittedMaxZ) //to discard depth values not between the permitted range
            {
                sparse_mat.ref<uchar>(x, y, depthInMillimeters) = I_MAX;

                if (this->isTrackPoint)
                {
                    if (x == this->trackX && y == this->trackY)
                        cout << "\tVolObj\t\tX: " << x << "\tY: " << y << "\tZ: " << depthInMillimeters << endl;
                }

            }
            //          else
            //              cout << "Point dropped on Z: " << depthInMillimeters << endl;
        }
    }

    image.release();

    return sparse_mat;

}




cv::SparseMat VmtFunctions::subtractSparseMat(const cv::SparseMat& operand1, const cv::SparseMat& operand2) /* current - previous */
{	
    //diff(x, y, z) = |operand2(x,y,z) - operand1(x,y,z)|
    //				= 1 if |1 - 0| or |0 - 1|
    //				= 0 if |0 - 0| or |1 - 1|

    //init difference:
    cv::SparseMat difference = cv::SparseMat(this->dims, this->matrixSize, operand2.type());

    //go through nonzero values of operand2, and set diff to 1 if operand1(x,y,z)==0
    for (cv::SparseMatConstIterator op2It=operand2.begin(); op2It != operand2.end(); ++op2It)
    {
        const cv::SparseMat::Node* n = op2It.node();
        //        uchar val2 = op2It.value<uchar>();
        uchar val1 = operand1.value<uchar>(n->idx);

        if (val1 <= 0) //if no value was found on that location:
        {
            int depthCorrectionCoef = this->depthCorrectionCoefficient(n->idx[Z]);
            int indexOfDepth = this->dynamicTolerance.allDepthValues.indexOf(n->idx[Z]);
            for (int i = indexOfDepth-depthCorrectionCoef; i<= indexOfDepth+depthCorrectionCoef; i++)
            {
                int potentialZ = this->dynamicTolerance.allDepthValues[i];
                for (int offsetX = -(depthCorrectionCoef); offsetX <= depthCorrectionCoef; offsetX++) //search for the neighborhood of x
                {
                    for (int offsetY = -(depthCorrectionCoef); offsetY <= depthCorrectionCoef; offsetY++) //search for the neighborhood of y
                    {
                        uchar temp = operand1.value<uchar>(n->idx[X]+offsetX, n->idx[Y]+offsetY, potentialZ); //if the element did not exist, value methods return 0.
                        if (temp > 0)
                        {
                            val1 = temp;
                            break;
                        }
                    }
                }
            }
        }

        //        int dynamicTolerance = this->dynamicTolerance.GetTolerance(n->idx[Z]);

        //        if (val1 <= 0) //if no value was found on that location:
        //        {
        //            //search for the neighborhood of z (to handle noise from kinect)
        //            for (int offsetZ = -(dynamicTolerance); offsetZ <= dynamicTolerance; offsetZ++) //FIXME: instead of searching, we can just pinpoint the next-previous values for performance
        //            {
        //                for (int offsetX = -(this->toleranceX); offsetX <= (int)this->toleranceX; offsetX++) //search for the neighborhood of x
        //                {
        //                    for (int offsetY = -(this->toleranceY); offsetY <= (int)this->toleranceY; offsetY++) //search for the neighborhood of y
        //                    {
        //                        uchar temp = operand1.value<uchar>(n->idx[X]+offsetX, n->idx[Y]+offsetY, n->idx[Z]+offsetZ); //if the element did not exist, the methods return 0.
        //                        if (temp > 0)
        //                        {
        //                            val1 = temp;
        //                            break;
        //                        }
        //                    }
        //                }
        //            }
        //        }

        if (val1 <= 0) //if still no value (after the neighborhood search)
        {
            difference.ref<uchar>(n->idx) = I_MAX;
        }
    }
    //go through nonzero values of operand1, and set diff to 1 if operand2(x,y,z)==0
    for (cv::SparseMatConstIterator op1It=operand1.begin(); op1It != operand1.end(); ++op1It)
    {
        const cv::SparseMat::Node* n = op1It.node();
        //        uchar val1 = op1It.value<uchar>();
        uchar val2 = operand2.value<uchar>(n->idx);

        if (val2 <= 0) //if no value was found on that location:
        {
            int depthCorrectionCoef = this->depthCorrectionCoefficient(n->idx[Z]);
            int indexOfDepth = this->dynamicTolerance.allDepthValues.indexOf(n->idx[Z]);
            for (int i = indexOfDepth-depthCorrectionCoef; i<= indexOfDepth+depthCorrectionCoef; i++)
            {
                int potentialZ = this->dynamicTolerance.allDepthValues[i];
                for (int offsetX = -(depthCorrectionCoef); offsetX <= depthCorrectionCoef; offsetX++) //search for the neighborhood of x
                {
                    for (int offsetY = -(depthCorrectionCoef); offsetY <= depthCorrectionCoef; offsetY++) //search for the neighborhood of y
                    {
                        uchar temp = operand2.value<uchar>(n->idx[X]+offsetX, n->idx[Y]+offsetY, potentialZ); //if the element did not exist, the methods return 0.
                        if (temp > 0)
                        {
                            val2 = temp;
                            break;
                        }
                    }
                }
            }
        }

        //        int dynamicTolerance = this->dynamicTolerance.GetTolerance(n->idx[Z]);

        //        if (val2 <= 0) //if no value was found on that location:
        //        {
        //            //search for the neighborhood of z (to handle noise from kinect)
        //            for (int offsetZ = -(dynamicTolerance); offsetZ <= dynamicTolerance; offsetZ++)
        //            {
        //                for (int offsetX = -(this->toleranceX); offsetX <= (int)this->toleranceX; offsetX++) //search for the neighborhood of x
        //                {
        //                    for (int offsetY = -(this->toleranceY); offsetY <= (int)this->toleranceY; offsetY++) //search for the neighborhood of y
        //                    {
        //                        uchar temp = operand2.value<uchar>(n->idx[X]+offsetX, n->idx[Y]+offsetY, n->idx[Z]+offsetZ); //if the element did not exist, the methods return 0.
        //                        if (temp > 0)
        //                        {
        //                            val2 = temp;
        //                            break;
        //                        }
        //                    }
        //                }
        //            }
        //        }

        if (val2 <= 0) //if still no value (after the neighborhood search)
        {
            difference.ref<uchar>(n->idx) = I_MAX;
        }
    }

    //all of the remaining points of difference should be equal to 0.

    if (this->isTrackPoint)
    {
        for (unsigned int z=this->permittedMinZ; z<this->permittedMaxZ; z++)
        {
            uchar valDel = difference.value<uchar>(this->trackX, this->trackY, z);
            if (valDel > 0)
            {
                cout << "\tDelta\t\tX: " << this->trackX << "\tY: " << this->trackY << "\tZ: " << z << endl;

                uchar valOp1 = operand1.value<uchar>(this->trackX, this->trackY, z);
                if (valOp1 > 0)
                    cout << "\t\tOper1\tX: " << this->trackX << "\tY: " << this->trackY << "\tZ: " << z << endl;

                uchar valOp2 = operand2.value<uchar>(this->trackX, this->trackY, z);
                if (valOp2 > 0)
                    cout << "\t\tOper2\tX: " << this->trackX << "\tY: " << this->trackY << "\tZ: " << z << endl;
            }
        }
    }

    return difference;
}

cv::SparseMat VmtFunctions::cleanUpVolumeObjectDifference(const cv::SparseMat& volObjDiff) const
{
    cv::SparseMat cleanedUpVolObjDiff(volObjDiff.dims(), volObjDiff.size(), volObjDiff.type());

    QHash<QPoint, int> processedPoints; // <point in [X, Y], Zindex>

    for (cv::SparseMatConstIterator it=volObjDiff.begin(); it != volObjDiff.end(); ++it)
    {
        const cv::SparseMat::Node* n = it.node();

        QPoint curPoint = QPoint(n->idx[X], n->idx[Y]);

        if (processedPoints.contains(curPoint)) //we already saw a point in this [X,Y]
        {
            int currentZIndex = n->idx[Z];
            int existingZIndex = processedPoints.value(curPoint);

            if (currentZIndex < existingZIndex) //if it's nearer to the camera:
            {
                processedPoints[curPoint] = currentZIndex; //set it to the current Z index
                //                cout << "Point updated (" << curPoint.x() << ", " << curPoint.y() << ") Z: " << existingZIndex << " --> " << currentZIndex << endl;
            }
            //else //, just discard it
            //                cout << "Discarded point on clean-up (" << curPoint.x() << ", " << curPoint.y() << ", " << currentZIndex << ")\n";
        }
        else //first time we process a point in this [X, Y]
        {
            processedPoints.insert(curPoint, n->idx[Z]);
        }
    }

    //    cout << "\tAll points parsed. Fetched " << processedPoints.keys().length() << " out of " << (int)volObjDiff.nzcount() << " points.\n";

    //create the cleaned-up sparse mat:
    QHashIterator<QPoint, int> it(processedPoints);
    while (it.hasNext())
    {
        it.next();
        QPoint p = it.key();
        int z = it.value();
        cleanedUpVolObjDiff.ref<uchar>(p.x(), p.y(), z) = volObjDiff.value<uchar>(p.x(), p.y(), z);
        if (this->isTrackPoint)
        {
            if (p.x() == this->trackX && p.y() == this->trackY)
                cout << "\tClean Delta\tX: " << p.x() << "\tY: " << p.y() << "\tZ: " << z << endl;
        }

    }

    //    cout << "\tVolume object created & cleaned-up (" << (int)cleanedUpVolObjDiff.nzcount() << " points)\n";


    return cleanedUpVolObjDiff;
}

double VmtFunctions::magnitudeOfMotion(const cv::SparseMat& sparseMat)
{
    VmtInfo info = getVmtInfo(sparseMat);

    int sizes[3];
    sizes[X] = info.maxX - info.minX;
    sizes[Y] = info.maxY - info.minY;
    sizes[Z] = info.maxZ - info.minZ;

    double normalizedMagnitudeOfMotion = (double)sparseMat.nzcount() / (double)(sizes[X]*sizes[Y]*sizes[Z]);

//    qDebug() << "Number of nz elements: " << (int)sparseMat.nzcount();
//    qDebug() << "Normalized Magnitude of Motion: " << normalizedMagnitudeOfMotion;


    return normalizedMagnitudeOfMotion;
}

//int VmtFunctions::magnitudeOfMotion(const cv::SparseMat& sparseMat)
//{
//    return (int)sparseMat.nzcount();
//}

double VmtFunctions::attenuationConstantForAnAction(const QList<cv::SparseMat>& volumeObjectsDifferences)
{
    double attConst = 0.0;
    double sumOfMagnitutedOfMotion = 0;

    foreach(cv::SparseMat sigma, volumeObjectsDifferences)
    {        
        sumOfMagnitutedOfMotion += magnitudeOfMotion((sigma));
    }

    attConst = ((double)I_MAX - 1.0 ) / (double)sumOfMagnitutedOfMotion;

    return attConst;
}

//double VmtFunctions::attenuationConstantForAnAction(const QList<cv::SparseMat>& volumeObjectsDifferences)
//{
//    double attConst = 0.0;
//    int sumOfMagnitutedOfMotion = 0;
//    int curMagnituteOfMotion = 0;

//    double curAttConst = 0.0;
//    double sumOfAttConst = 0.0;

//    int numberOfDiffs = volumeObjectsDifferences.length();

//    for(int i=0; i< numberOfDiffs; i++)
//    {
//        cv::SparseMat it = volumeObjectsDifferences.at(i); //i'th difference = difference between i-1'th and i'th volume objects
//        curMagnituteOfMotion = magnitudeOfMotion((it)); // magnitude of motion for this volume obj difference
//        sumOfMagnitutedOfMotion += curMagnituteOfMotion; //sum, so far

//        //for each difference object, attenuating constant is calculated over the sum of previous magnitudes of motion
//        curAttConst = (double)(I_MAX - 1) / (double)sumOfMagnitutedOfMotion;
//        sumOfAttConst += curAttConst;
//    }

//    attConst = sumOfAttConst / (double)numberOfDiffs; //attenating constant for this interval is the average of attenuating constants over this interval

//    return attConst; //FIXME!!!!!!!
//}

//double VmtFunctions::AttenuationConstantForAnAction(const QList<cv::SparseMat>& volumeObjectsDifferences)
//{
//    double attConst = 0.0;
//    int sumOfMagnitutedOfMotion = 0;
//    int curMagnituteOfMotion = 0;

//    for(int i=0; i<volumeObjectsDifferences.length(); i++)
//    {
//        cv::SparseMat it = volumeObjectsDifferences.at(i); //i'th difference = difference between i-1'th and i'th volume objects
//        curMagnituteOfMotion = MagnitudeOfMotion((it));
//        sumOfMagnitutedOfMotion += curMagnituteOfMotion;
//    }
//    attConst = (double)(I_MAX - 1) / (double)sumOfMagnitutedOfMotion;

//    return attConst;
//}

cv::SparseMat VmtFunctions::calculateVMT(const QList<cv::SparseMat> &volumeObjectDifferences)
{
    cv::SparseMat curVmt;
    cv::SparseMat prevVmt;
    //Formulation:
    //VMT(x, y, z, t) = 255 if volumeObjectDifference(x, y, z, t) == 1 (--> it's a recent point, start it with high intensity)
    //				  = max(0, VMT(x, y, z, t-1)-(attenuationConstant)*(magnitudeOfMotion(t)), otherwise (it's an old point, decrease its intensity, or set it to 0)

    //calculate attenuation constant for the set:
    double attConst = attenuationConstantForAnAction(volumeObjectDifferences);
    cout << "Attenuating constant: " << attConst << endl;
    double curMagnituteOfMotion = 0.0;

    //repeat for all volume differences:
    for(int i=0; i<volumeObjectDifferences.length(); i++)
    {
        cv::SparseMat deltaT = volumeObjectDifferences.at(i);
        curVmt.clear();
        curVmt.release();
        curVmt = cv::SparseMat(this->dims, deltaT.size(), deltaT.type()); //create a sparse matrix
        curMagnituteOfMotion = magnitudeOfMotion(deltaT); //calculate magnitute of motion of current volume difference

        //for all, set points of current VMT to I_MAX where difference is non-zero:
        for(cv::SparseMatConstIterator dit = deltaT.begin(); dit != deltaT.end(); ++dit)
        {
            const cv::SparseMat::Node* n = dit.node();
            curVmt.ref<uchar>(n->idx) = I_MAX;
        }

        //for all delta, except the first one:
        if (i > 0)
        {
            double disappearRate = attConst*curMagnituteOfMotion;
                        cout << "Disappearing rate at t= " << i << ": " << disappearRate << endl;
            //for all nonzero values of previous VMT
            for(cv::SparseMatConstIterator pit = prevVmt.begin(); pit != prevVmt.end(); ++pit)
            {
                const cv::SparseMat::Node* n = pit.node();

                if (deltaT.value<uchar>(n->idx) <= 0) //if difference is zero at that point
                {
                    //and if value is bigger than [attenuation constant times magnitute of motion]
                    int newValue = (int)((double)pit.value<uchar>() - disappearRate);
                    if (newValue > 0)
                    {
                        curVmt.ref<uchar>(n->idx) = (uchar)newValue;
                    }
                }
            }

            if (this->isTrackPoint)
            {
                for (unsigned int z=this->permittedMinZ; z<this->permittedMaxZ; z++)
                {
                    uchar val = curVmt.value<uchar>(this->trackX, this->trackY, z);
                    if (val > 0)
                        cout << "\tVMT " << i << "\tX: " << this->trackX << "\tY: " << this->trackY << "\tZ: " << z << "\tIntensity: " << (int)val << endl;
                }
            }
        }

        if (this->saveVmt)
        {
            cv::SparseMat nor = this->spatiallyNormalizeSparseMat(curVmt);
            cv::SparseMat trm = this->trimSparseMat(nor);
            PointCloudFunctions::saveVmtAsCloud(trm, QString("/home/emredog/Documents/output/VMT_%1.pcd").arg(QString::number(i).rightJustified(2, '0')).toStdString());
            nor.release();
            trm.release();
        }

        //clear prevVmt and copy curVmt --> prevVmt
        prevVmt.clear();
        prevVmt.release();
        curVmt.copyTo(prevVmt); //"The destination will be reallocated if needed."
    }

    return curVmt;
}




//unused methods------------------------------------------------------------------------------------------------
cv::SparseMat VmtFunctions::calculateD_Old(cv::SparseMat lastVolumeObject, cv::SparseMat firstVolumeObject) //equation (8) from the paper
{
    cv::SparseMat d_Old(lastVolumeObject.dims(), lastVolumeObject.size(), lastVolumeObject.type());

    //only condition where d_old is non-zero is first>0 and last=0 (see below)
    for (cv::SparseMatConstIterator opIt=lastVolumeObject.begin(); opIt != lastVolumeObject.end(); ++opIt)
    {
        const cv::SparseMat::Node* n = opIt.node();
        uchar lastVal = opIt.value<uchar>();
        uchar firstVal = firstVolumeObject.value<uchar>(n->idx);
        if (firstVal - lastVal > 0)
            d_Old.ref<uchar>(n->idx) = firstVal - lastVal;
    }

    return d_Old;

    //d_Old(x, y, z) = max(0, first(x,y,z) - last(x,y,z)
    //			0	 =			0				0
    //			0	 =			1				1
    //			0    =			0				1
    //			1	 =			1				0

    /*cv::Mat firstMat;
    cv::Mat lastMat;
    cv::Mat difference;
    cv::Mat zeros;
    cv::Mat max;

    lastVolumeObject.convertTo(lastMat, lastVolumeObject.type());
    firstVolumeObject.convertTo(firstMat, firstVolumeObject.type());
    zeros = cv::Mat(firstMat.dims, firstMat.size, firstMat.type(), 0);

    cv::subtract(firstMat, lastMat, difference);
    firstMat.release();
    lastMat.release();

    max = cv::max(difference, zeros);
    difference.release();
    zeros.release();


    cv::SparseMat sparseMax(max);
    max.release();
    return sparseMax;*/
}

cv::SparseMat VmtFunctions::calculateD_New(cv::SparseMat lastVolumeObject, cv::SparseMat firstVolumeObject) //equation (7) from the paper
{
    cv::SparseMat d_New(lastVolumeObject.dims(), lastVolumeObject.size(), lastVolumeObject.type());

    //only condition where d_old is non-zero is first>0 and last=0 (see below)
    for (cv::SparseMatConstIterator opIt=lastVolumeObject.begin(); opIt != lastVolumeObject.end(); ++opIt)
    {
        const cv::SparseMat::Node* n = opIt.node();
        uchar lastVal = opIt.value<uchar>();
        uchar firstVal = firstVolumeObject.value<uchar>(n->idx);
        if (lastVal - firstVal > 0)
            d_New.ref<uchar>(n->idx) = lastVal - firstVal;
    }

    return d_New;

    //d_New(x, y, z) = max(0, last(x,y,z) - first(x,y,z)
    //			0	 =			0				0
    //			0	 =			1				1
    //			0    =			0				1
    //			1	 =			1				0
}

cv::Vec3i VmtFunctions::calculateMomentVector(cv::SparseMat volumeObjectSparse) //equation (9) from the paper
{
    int numOfAllElements = (int)volumeObjectSparse.nzcount(); //number of all elements of a volumeObject is equal to number of nonzero (=1) elements

    cv::Vec3i sumVector(0, 0, 0);
    for(cv::SparseMatConstIterator it = volumeObjectSparse.begin(); it != volumeObjectSparse.end(); ++it)
    {
        const cv::SparseMat::Node* n = it.node();
        cv::Vec3i currentVector(n->idx[0], n->idx[1], n->idx[2]);
        sumVector += currentVector;
    }

    cv::Vec3i momentVector((int)(sumVector.val[0]/numOfAllElements), (int)(sumVector.val[1]/numOfAllElements), (int)(sumVector.val[2]/numOfAllElements));

    return momentVector;
}

//double VmtFunctions::calculateAlpha(cv::Vec3i motionVector) //equation (14) from the paper
//{
//    //Indices: y=0, x=1, z=2
//    //norm of projection of vector (a, b, c) to axis y is 'a' (because index of y=0)
//    int y = motionVector.val[0],
//            z = motionVector.val[2];
//    return 1.0/cos(-(HelperFunctions::sgn(y))*(HelperFunctions::sgn(z))*((double)y / (double)(y - z)));
//}

//double VmtFunctions::calculateBeta(cv::Vec3i motionVector) //equation (15) from the paper
//{
//    //Indices: y=0, x=1, z=2
//    //norm of projection of vector (a, b, c) to axis y is 'a' (because index of y=0)
//    int x = motionVector.val[1],
//            z = motionVector.val[2];
//    return 2.0/CV_PI - 1.0/cos(-(HelperFunctions::sgn(z))*HelperFunctions::sgn(x)*((double)z / (double)(z-x)));
//}

//double VmtFunctions::calculateTheta(cv::Vec3i motionVector) //equation (16) from the paper
//{
//    //Indices: y=0, x=1, z=2
//    //norm of projection of vector (a, b, c) to axis y is 'a' (because index of y=0)
//    int x = motionVector.val[1],
//            y = motionVector.val[0];

//    return 1.0/cos(-(HelperFunctions::sgn(x))*HelperFunctions::sgn(y)*((double)x / (double)(x-y)));
//}

cv::Matx33d VmtFunctions::calculateRotationX_alpha(double alpha) //equation (11) from the paper
{
    return
            cv::Matx33d(	1.0,		0.0,			0.0,
                            0.0,		cos(alpha),		sin(alpha),
                            0.0,		-sin(alpha),	cos(alpha)		);

}

cv::Matx33d VmtFunctions::calculateRotationY_beta(double beta) //equation (12) from the paper
{
    return
            cv::Matx33d(	cos(beta),	0.0,		-sin(beta),
                            0.0,		1.0,		0.0,
                            sin(beta),	0.0,		cos(beta)		);
}

cv::Matx33d VmtFunctions::calculateRotationZ_theta(double theta) //equation (13) from the paper
{
    return
            cv::Matx33d(	cos(theta),		sin(theta),		0.0,
                            -sin(theta),	cos(theta),		0.0,
                            0.0,			0.0,			1.0		);
}

cv::SparseMat VmtFunctions::rotateVMT(const cv::SparseMat& vmt, const cv::Matx33d& rotationMatrix)
{	
    //	int newSizes[] = {5000, 5000, 5000}; //FIXME
    cv::SparseMat rotatedVmt(3, vmt.size(), vmt.type());
    cv::SparseMat shiftedRotatedVmt(3, vmt.size(), vmt.type());

    cv::Matx13d curVec;
    cv::Matx13d rotVec;
    int idx[3];
    int minY = INT_MAX, /*maxY = INT_MIN,*/ minX = INT_MAX/*, maxX = INT_MIN, minZ = INT_MAX, maxZ = INT_MIN*/;
    int translationX = 0, translationY = 0;

    for(cv::SparseMatConstIterator it = vmt.begin(); it != vmt.end(); ++it)
    {
        //for each point
        const cv::SparseMat::Node* n = it.node();
        //create a 3d vector:
        curVec = cv::Matx13d(n->idx[0], n->idx[1], n->idx[2]);
        //rotate it with the matrix
        rotVec = curVec * rotationMatrix;

        //find min-max of rotated points
        minY = min(minY, idx[0]);
        //maxY = max(maxY, idx[0]);
        minX = min(minX, idx[1]);
        //maxX = max(maxX, idx[1]);
        //minZ = min(minZ, idx[2]);
        //maxZ = max(maxZ, idx[2]);

        idx[0] = (int)rotVec.val[0];
        idx[1] = (int)rotVec.val[1];
        idx[2] = (int)rotVec.val[2];
        //set rotated point to rotated VMT
        rotatedVmt.ref<uchar>(idx) = it.value<uchar>();

    }
    //we try to shift the minimum point to 0
    translationX = -minX;
    translationY = -minY;
    cout << "translation x: " << translationX << ", translation y: " << translationY << endl;

    //shift the rotated VMT into the image frame:
    for(cv::SparseMatConstIterator rit = rotatedVmt.begin(); rit != rotatedVmt.end(); ++rit)
    {
        const cv::SparseMat::Node* n = rit.node();
        idx[0] = n->idx[0] + translationY;
        idx[1] = n->idx[1] + translationX;
        idx[2] = n->idx[2];
        shiftedRotatedVmt.ref<uchar>(idx) = rit.value<uchar>();
    }

    rotatedVmt.release();

    //cout << "[" << minY << ", " << maxY << "], [" << minX << ", " << maxX << "], [" << minZ << ", " << maxZ << "]\n";

    return shiftedRotatedVmt;
}

cv::Mat VmtFunctions::projectVMTOntoXY(const cv::SparseMat& vmt)
{
    //cv::Mat projectedMatrix(480, 640, vmt.type()); //create an empty matrix
    //FIXME:
    cv::Mat projectedMatrix(1000, 800, vmt.type());
    projectedMatrix = cv::Scalar(0); //set all values to zero


    for(cv::SparseMatConstIterator it = vmt.begin(); it != vmt.end(); ++it)
    {
        //for each point in 3D
        const cv::SparseMat::Node* n = it.node();
        //FIXME
        if (n->idx[0] >=0 && n->idx[0] < 800 && n->idx[1]>= 0 && n->idx[1] < 800)
        {
            projectedMatrix.at<uchar>(n->idx[0], n->idx[1])
                    = max(projectedMatrix.at<uchar>(n->idx[0], n->idx[1]), it.value<uchar>());
        }
    }

    return projectedMatrix;
}

//other methods------------------------------------------------------------------------------------------------
void VmtFunctions::print3x3Matrix(const cv::Matx33d& mat)
{
    cv::Mat M(mat);
    for(int i=0; i<3; ++i)
    {
        for (int j=0; j<3; ++j)
        {
            cout << M.at<double>(i, j) << "\t";
        }
        cout << endl;
    }


}

void VmtFunctions::print3DSparseMatrix(const cv::SparseMat &sparse_mat)
{		
    cv::Mat denseMat;
    sparse_mat.convertTo(denseMat, CV_8UC1);

    int idx[3];
    for (int i=0; i<sparse_mat.size()[X] ; ++i)
    {
        for (int j=0; j<sparse_mat.size()[Y] ; ++j)
        {
            for (int k=0; k<sparse_mat.size()[Z] ; ++k)
            {
                //FIXME: it isnt the same indices??!
                idx[0] = i; idx[1] = j; idx[2] = k;
                uchar value = denseMat.at<uchar>(idx);
                if (value > 0)
                {
                    cout << "(" << i << ",\t" << j << ",\t" << k << "):\t" << value << endl;
                }
            }
        }
    }
}

void VmtFunctions::save3DSparseMatrix(const cv::SparseMat &sparse_mat, QString filePath)
{
    QFile myfile(filePath);
    if (!myfile.open(QFile::ReadWrite))
    {
        cout << "ERROR!" << endl;
        return;
    }

    QTextStream stream(&myfile);

    cv::Mat denseMat;
    sparse_mat.convertTo(denseMat, CV_8UC1); //FIXME!!!!!!

    //write header:
    stream << sparse_mat.size()[X] << "x" << sparse_mat.size()[Y] << "x" << sparse_mat.size()[Z] << endl;

    //write non-zero values:
    int idx[3];
    for (int i=0; i<sparse_mat.size()[X] ; ++i)
    {
        for (int j=0; j<sparse_mat.size()[Y] ; ++j)
        {
            for (int k=0; k<sparse_mat.size()[Z] ; ++k)
            {
                //FIXME: it isnt the same indices??!
                idx[X] = j; idx[Y] = i; idx[Z] = k;
                uchar value = denseMat.at<uchar>(idx);
                if (value > 0)
                {
                    stream << i << ";" << j << ";" << k << ";" << value << endl;
                }
            }
        }
    }

    //stream.flush();
    myfile.close();
}

void VmtFunctions::save3DMatrix(const cv::Mat &mat, QString filePath)
{
    QFile myfile(filePath);
    if (!myfile.open(QFile::ReadWrite))
    {
        cout << "ERROR!" << endl;
        return;
    }

    QTextStream stream(&myfile);

    //write header:
    stream << mat.size[X] << "x" << mat.size[Y] << "x" << mat.size[Z] << endl;

    //write non-zero values:
    int idx[3];
    for (int i=0; i<mat.size[X] ; ++i)
    {
        for (int j=0; j<mat.size[Y] ; ++j)
        {
            for (int k=0; k<mat.size[Z] ; ++k)
            {
                //FIXME check the indices
                idx[X] = i; idx[Y] = j; idx[Z] = k;
                uchar value = mat.at<uchar>(idx);
                if (value > 0)
                {
                    stream << i << ";" << j << ";" << k << ";" << value << endl;
                }
            }
        }
    }
    myfile.close();
}

VmtFunctions::VmtInfo VmtFunctions::getVmtInfo(const cv::SparseMat &vmt) const
{
    VmtInfo info;
    //Get size of vmt
    const int* sizes = vmt.size();
    //FIXME: check if it matches
    info.sizeInX = sizes[X];
    info.sizeInY = sizes[Y];
    info.sizeInZ = sizes[Z];

    //Get number of points
    info.numberOfPoints = (int)vmt.nzcount();

    int minX = INT_MAX, minY = INT_MAX, minZ = INT_MAX,
            maxX = INT_MIN, maxY = INT_MIN, maxZ = INT_MIN;

    //Get boundaries
    for (cv::SparseMatConstIterator it=vmt.begin(); it != vmt.end(); ++it)
    {
        const cv::SparseMat::Node* n = it.node();
        int x = n->idx[0];
        int y = n->idx[1];
        int z = n->idx[2];

        if (x > maxX) maxX = x;
        if (y > maxY) maxY = y;
        if (z > maxZ) maxZ = z;
        if (x < minX) minX = x;
        if (y < minY) minY = y;
        if (z < minZ) minZ = z;
    }

    //FIXME: chech if itmatches
    info.maxX = maxX;
    info.maxY = maxY;
    info.maxZ = maxZ;
    info.minX = minX;
    info.minY = minY;
    info.minZ = minZ;

    return info;
}

int VmtFunctions::saveVmtAsImageSequence(const cv::SparseMat &vmt, QString outputFolder) const
{
    //first check if the output folder exists:
    if (!QDir(outputFolder).exists())
        QDir().mkdir(outputFolder);

    int totalDepth = vmt.size()[Z]; //FIXME: this should be a constant value (=NORMALIZATION_INTERVAL) for all created vmt
    int counter = 0;
    int ptCounter = 0;

    qDebug() << "Points in VMT: " << vmt.nzcount();

    //FIXME: could this loop be optimized? using regular matrices maybe?
    //foreach depth
    for (int z = 0; z < totalDepth; z++)
    {
        cv::Mat frame = cv::Mat(vmt.size()[Y], vmt.size()[X], vmt.type(), cv::Scalar(0)); //Create a new frame
        for (int x = 0; x < frame.cols; x++)
            for (int y = 0; y < frame.rows; y++)
            {
                uchar val = vmt.value<uchar>(x, y, z); //check all values in this depth
                if (val > 0)
                {
                    frame.at<uchar>(x, y) = val; //Set to the new frame if non-zero value is found
                    ptCounter++;
                }
            }

        //Save this frame as
        if (cv::imwrite(QString("%1/VMT_%2.jpg").arg(outputFolder).arg(QString::number(z).rightJustified(2, '0')).toStdString(), frame))
            counter++;

    }

    qDebug() << counter << " frames were saved with a total of " << ptCounter << " points.";

    return counter;

}

cv::SparseMat VmtFunctions::trimSparseMat(const cv::SparseMat &vmt) const
{
    VmtInfo info = getVmtInfo(vmt);

    cout << "\tSize before trimming: " << vmt.size()[X] << "x" << vmt.size()[Y] << "x" << vmt.size()[Z] << endl;

    int sizes[3];
    sizes[X] = info.maxX - info.minX;
    sizes[Y] = info.maxY - info.minY;
    sizes[Z] = info.maxZ - info.minZ;

    cv::SparseMat trimmed(vmt.dims(), sizes, vmt.type());

    //parse & copy points to new locations
    for (cv::SparseMatConstIterator it=vmt.begin(); it != vmt.end(); ++it)
    {
        const cv::SparseMat::Node* n = it.node();

        int newX = n->idx[X] - info.minX;
        int newY = n->idx[Y] - info.minY;
        int newZ = n->idx[Z] - info.minZ;

        trimmed.ref<uchar>(newX, newY, newZ) = vmt.value<uchar>(n->idx);
    }


    cout << "\tSize after trimming: " << trimmed.size()[X] << "x" << trimmed.size()[Y] << "x" << trimmed.size()[Z] << endl;
    return trimmed;
}

cv::SparseMat VmtFunctions::spatiallyNormalizeSparseMat(cv::SparseMat vmt) const
{
    cv::SparseMat normalizedVmt(vmt.dims(), vmt.size(), vmt.type());

    double normCoef = (double)(NORMALIZATION_INTERVAL)/(double)(vmt.size()[Z]);
    //    qDebug() << "Normalization coef: " << normCoef;

    //parse & copy points to new locations
    for (cv::SparseMatConstIterator it=vmt.begin(); it != vmt.end(); ++it)
    {
        const cv::SparseMat::Node* n = it.node();

        //FIXME: check if indices match
        int newX = n->idx[X];
        int newY = n->idx[Y];
        int newZ = (int)(normCoef * (double)n->idx[Z]);

        normalizedVmt.ref<uchar>(newX, newY, newZ) = vmt.value<uchar>(n->idx);
        //        qDebug() << n->idx[Z] << "\t" << newZ;
    }

    return normalizedVmt;
}

cv::Mat VmtFunctions::extractSilhouette(const cv::Mat &mat) const
{
    cv::Mat temp, matGrayscale = cv::Mat(mat.dims, mat.size, CV_8UC1), thresholded;
    //format issues
    mat.convertTo(temp, CV_16UC1);



    std::vector<uchar> values;
    unsigned short pixelValue;
    unsigned short mostSig13Digits;
    unsigned short depth;
    unsigned int depthInMillimeters;

    //Create a 8bit matrix from depth image
    for (int y = 0; y < temp.rows; y++)
    {
        const unsigned short* ithRow = temp.ptr<unsigned short>(y); //take a whole row
        for (int x = 0; x < temp.cols; x++)
        {
            pixelValue = ithRow[x];
            mostSig13Digits = pixelValue & 65504; // (65504 = 1111 1111 1110 0000)
            depth = mostSig13Digits >> 5;
            depthInMillimeters = (unsigned int)(raw_depth_to_meters((int)depth)*1000);
            uchar newValue = (256.0 / (double)(this->permittedMaxZ-this->permittedMinZ)) * depthInMillimeters;
            matGrayscale.at<uchar>(y, x) =newValue;
            if (newValue > 0 && newValue < 255 && depthInMillimeters >= this->permittedMinZ && depthInMillimeters <= this->permittedMaxZ)
                values.push_back(newValue); //add accepted values to the list
        }
    }

    //calculate Otsu threshold value on the accepted values
    double calculatedOtsu = cv::threshold(values, values, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    //apply standart threshold with calculated otsu threshold value
    cv::threshold(matGrayscale, thresholded, calculatedOtsu, 255, CV_THRESH_BINARY_INV);

    //    cv::namedWindow("debug");
    //    cv::imshow("debug", mat);

    // mask the original depth image with thresholded binary image
    cv::Mat silhouette = cv::Mat::zeros(mat.size[0], mat.size[1], mat.type());

    //    cv::imshow("debug", thresholded);
    //    cv::waitKey(0);

    //bitwise_and didnt work as expected!!!

    //do it by hand:
    for (int x=0; x<mat.rows; x++)
        for (int y=0; y<mat.cols; y++)
        {
            uchar val = thresholded.at<uchar>(x, y);
            if (val == 255)
                silhouette.at<unsigned short>(x, y) = mat.at<unsigned short>(x, y);
        }

    //    cv::namedWindow("sil");
    //    cv::imshow("sil", silhouette);
    //    cv::waitKey(0);

    //Cleanup
    temp.release();
    matGrayscale.release();
    thresholded.release();

    return silhouette;
}

int VmtFunctions::depthCorrectionCoefficient(int depthInMm) const
{
    //FIXME: find a proper method to perform this
    if (depthInMm < 1405)
        return 1;
    else if (depthInMm < 1735)
        return 1;
    else if (depthInMm < 2531)
        return 1;
    else if (depthInMm < 3083)
        return 4;
    else if (depthInMm < 4364)
        return 4;
    else if (depthInMm < 5652)
        return 4;
    else if (depthInMm < 8017)
        return 9;
    else if (depthInMm < 9984)
        return 9;
    else
        return 9;
}

//vector<cv::SparseMat> VmtFunctions::CalculateVolumeObjectDifferencesSparse(const vector<cv::SparseMat>& volumeObjects, int depthTolerance)
//{
//    cv::SparseMat operand1, operand2;
//    vector<cv::SparseMat> volumeObjectDifferences;
//    short counter = 2;

//    for (vector<cv::SparseMat>::const_iterator it = volumeObjects.begin() ; it != volumeObjects.end(); ++it)
//    {
//        if (it == volumeObjects.begin()) //skip the first one
//        {
//            it->assignTo(operand2);
//            continue;
//        }

//        operand1.release();
//        operand2.copyTo(operand1);
//        operand2.release();
//        it->assignTo(operand2);

//        // now operand2 is current volume object, operand1 is previous volume object
//        //diff(x, y, z) = |operand2(x,y,z) - operand1(x,y,z)|
//        //				= 1 if |1 - 0| or |0 - 1|
//        //				= 0 if |0 - 0| or |1 - 1|

//        //init difference:
//        cv::SparseMat difference = cv::SparseMat(this->dims, this->matrixSize, operand2.type());

//        //go through nonzero values of operand2, and set diff to 1 if operand1(x,y,z)==0
//        for (cv::SparseMatConstIterator op2It=operand2.begin(); op2It != operand2.end(); ++op2It)
//        {
//            const cv::SparseMat::Node* n = op2It.node();
//            uchar val2 = op2It.value<uchar>();
//            uchar val1 = 0;

//            //search for the neighborhood of z (to handle noise from kinect)
//            for (int offset = -depthTolerance; offset<depthTolerance; offset++)
//            {
//                //WARNING: do not replace following idx[0-1-2] with idx[X-Y-Z]
//                uchar temp = operand1.value<uchar>(n->idx[0], n->idx[1], n->idx[2]+offset);
//                if (temp > 0)
//                {
//                    val1 = temp;
//                    //cout << "Found one in the neighborhood! (1)" << endl;
//                    break;
//                }
//            }

//            if (val1 <= 0)
//                difference.ref<uchar>(n->idx) = val2;
//        }
//        //go through nonzero values of operand1, and set diff to 1 if operand2(x,y,z)==0
//        for (cv::SparseMatConstIterator op1It=operand1.begin(); op1It != operand1.end(); ++op1It)
//        {
//            const cv::SparseMat::Node* n = op1It.node();
//            uchar val1 = op1It.value<uchar>();
//            uchar val2 = 0;

//            //search for the neighborhood of z (to handle noise from kinect)
//            for (int offset = -depthTolerance; offset<depthTolerance; offset++)
//            {
//                //WARNING: do not replace following idx[0-1-2] with idx[X-Y-Z]
//                uchar temp = operand2.value<uchar>(n->idx[0], n->idx[1], n->idx[2]+offset);
//                if (temp > 0)
//                {
//                    val2 = operand2.value<uchar>(n->idx[0], n->idx[1], n->idx[2]+offset);
//                    //cout << "Found one in the neighborhood! (2)" << endl;
//                    break;
//                }
//            }

//            if (val2 <= 0)
//                difference.ref<uchar>(n->idx) = val1;
//        }

//        //all of the remaining points of difference should be equal to 0.

//        cout << "Difference calculated [" << counter << ", " << counter - 1 << "], nonzero: " << (int)difference.nzcount() << endl;
//        volumeObjectDifferences.push_back(difference);
//        counter++;
//        difference.release();
//    }
//    operand1.release();
//    operand2.release();

//    return volumeObjectDifferences;
//}

//vector<cv::SparseMat> VmtFunctions::ConstructVMTs(const vector<cv::SparseMat>& volumeObjectDifferences)
//{
//    vector<cv::SparseMat> vmtList;
//    cv::SparseMat curVmt;
//    cv::SparseMat prevVmt;
//    //Formulation:
//    //VMT(x, y, z, t) = 255 if volumeObjectDifference(x, y, z, t) == 1
//    //				  = max(0, VMT(x, y, z, t-1)-(attenuationConstant)*(magnitudeOfMotion(t)), otherwise

//    //calculate attenuation constant for the set:
//    double attConst = AttenuationConstantForAnAction(volumeObjectDifferences);
//    int curMagnituteOfMotion = 0;

//    //repeat for all volume differences:
//    for(vector<cv::SparseMat>::const_iterator deltaIt=volumeObjectDifferences.begin(); deltaIt != volumeObjectDifferences.end(); ++deltaIt)
//    {
//        curVmt = cv::SparseMat(this->dims, deltaIt->size(), deltaIt->type()); //create a sparse matrix
//        curMagnituteOfMotion = MagnitudeOfMotion(*deltaIt); //calculate magnitute of motion of current volume difference

//        //for all delta, except the first one:
//        if (deltaIt != volumeObjectDifferences.begin())
//        {
//            //take the previous VMT
//            prevVmt = vmtList.back();
//            double constant = attConst*curMagnituteOfMotion;
//            //for all nonzero values of previous VMT
//            for(cv::SparseMatConstIterator pit = prevVmt.begin(); pit != prevVmt.end(); ++pit)
//            {
//                const cv::SparseMat::Node* n = pit.node();

//                if (deltaIt->value<uchar>(n->idx) <= 0) //if difference is zero at that point
//                {
//                    //and if value is bigger than attenuation constant times magnitute of motion
//                    uchar newValue = pit.value<uchar>() - (uchar)(constant);
//                    if (newValue > 0)
//                    {
//                        curVmt.ref<uchar>(n->idx) = newValue;
//                    }
//                }
//            }
//        }

//        //for all, set points of current VMT to I_MAX where difference is non-zero:
//        for(cv::SparseMatConstIterator dit = deltaIt->begin(); dit != deltaIt->end(); ++dit)
//        {
//            const cv::SparseMat::Node* n = dit.node();
//            curVmt.ref<uchar>(n->idx) = I_MAX;
//        }
//        cout << "VMT constructed. nonzero: " << (int)curVmt.nzcount() << endl;
//        vmtList.push_back(curVmt);
//    }

//    return vmtList;
//}

//cv::SparseMat VmtFunctions::GenerateSparseVolumeObject(QString imagePath, int downsamplingRate)
//{
//    cv::SparseMat emptySparseMat;
//    cv::Mat image, temp;

//    //Read the file
//    image = cv::imread(imagePath.toStdString(), CV_LOAD_IMAGE_UNCHANGED);   //ATTENTION: LIRIS DATA HAS 16BIT DEPTH IMAGES
//    //Check for invalid input
//    if(! image.data )
//    {
//        cout <<  "Could not open or find the image: " << imagePath.toStdString() << endl ;
//        return emptySparseMat;
//    }

//    image.convertTo(temp, CV_16UC1);
//    image.release();
//    temp.assignTo(image);
//    temp.release();

//    //THRESHOLD TO GET SILHOUETTE
//    //cv::Mat image8bit;
//    //image.convertTo(image8bit, CV_8UC1, 1.0/256.0);
//    //cv::adaptiveThreshold(image8bit, binImage, I_MAX, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 3, 0);

//    //Generate Volume Object:
//    cv::SparseMat sparse_mat(this->dims, this->matrixSize, CV_8UC1);

//    unsigned short pixelValue;
//    unsigned short mostSig13Digits;
//    unsigned short depth;
//    unsigned int depthInMillimeters;

//    //NOTE: because of memory restrictions, if downsamplingRate == 2, x and y are incremented by 2 (~downsampling by 1/4)
//    for (int y = 0; y < image.rows-1; y+=downsamplingRate)
//    {
//        const unsigned short* ithRow = image.ptr<unsigned short>(y); //take a whole row

//        for (int x = 0; x < image.cols-1; x+=downsamplingRate)
//        {
//            pixelValue = ithRow[x];
//            mostSig13Digits = pixelValue & 65504; // (65504 = 1111 1111 1110 0000)
//            depth = mostSig13Digits >> 5;

//            depthInMillimeters = (unsigned int)(raw_depth_to_meters((int)depth)*1000);

//            if (depthInMillimeters >= this->permittedMinZ && depthInMillimeters <= this->permittedMaxZ) //to discard depth values not between the permitted range
//            {
//                //FIXME: verify this translation!!
//                depthInMillimeters -= this->permittedMinZ; //translate all points to fit between min-max permitted range
//                sparse_mat.ref<uchar>(y, x, depthInMillimeters) = I_MAX;
//            }
//        }
//    }

//    image.release();

//    return sparse_mat;
//}

//FIXME: this function is not tested
//cv::Mat VmtFunctions::GenerateVolumeObject(cv::Mat image, int downsamplingRate)
//{
//    cv::Mat volumeObject(this->dims, this->matrixSize, CV_8UC1);
//    cv::Mat temp;
//    //format issues
//    image.convertTo(temp, CV_16UC1);
//    image.release();
//    temp.assignTo(image);
//    temp.release();

//    unsigned short pixelValue;
//    unsigned short mostSig13Digits;
//    unsigned short depth;
//    unsigned int depthInMillimeters;

//    //NOTE: because of memory restrictions, if downsamplingRate == 2, x and y are incremented by 2 (~downsampling by 1/4)
//    for (int y = 0; y < image.rows-1; y+=downsamplingRate)
//    {
//        const unsigned short* ithRow = image.ptr<unsigned short>(y); //take a whole row

//        for (int x = 0; x < image.cols-1; x+=downsamplingRate)
//        {
//            pixelValue = ithRow[x];
//            mostSig13Digits = pixelValue & 65504; // (65504 = 1111 1111 1110 0000)
//            depth = mostSig13Digits >> 5;

//            depthInMillimeters = (unsigned int)(raw_depth_to_meters((int)depth)*1000);

//            if (depthInMillimeters >= this->permittedMinZ && depthInMillimeters <= this->permittedMaxZ) //to discard depth values not between the permitted range
//            {
//                //FIXME: verify this translation!!
//                depthInMillimeters -= this->permittedMinZ; //translate all points to fit between min-max permitted range

//                volumeObject.at<uchar>(y, x, depthInMillimeters) = I_MAX;


//                //ref<uchar>(y, x, depthInMillimeters) = I_MAX;
//            }
//        }
//    }

//    image.release();

//    return volumeObject;
//}

//vector<cv::SparseMat> VmtFunctions::CalculateVolumeObjectDifferencesSparse(const vector<cv::SparseMat>& volumeObjects)
//{
//    cv::SparseMat operand1, operand2;
//    vector<cv::SparseMat> volumeObjectDifferences;
//    short counter = 2;

//    for (vector<cv::SparseMat>::const_iterator it = volumeObjects.begin() ; it != volumeObjects.end(); ++it)
//    {
//        if (it == volumeObjects.begin()) //skip the first one
//        {
//            it->assignTo(operand2);
//            continue;
//        }

//        operand1.release();
//        operand2.copyTo(operand1);
//        operand2.release();
//        it->assignTo(operand2);

//        // now operand2 is current volume object, operand1 is previous volume object
//        //diff(x, y, z) = |operand2(x,y,z) - operand1(x,y,z)|
//        //				= 1 if |1 - 0| or |0 - 1|
//        //				= 0 if |0 - 0| or |1 - 1|

//        //init difference:
//        cv::SparseMat difference = cv::SparseMat(this->dims, this->matrixSize, operand2.type());

//        //go through nonzero values of operand2, and set diff to 1 if operand1(x,y,z)==0
//        for (cv::SparseMatConstIterator opIt=operand2.begin(); opIt != operand2.end(); ++opIt)
//        {
//            const cv::SparseMat::Node* n = opIt.node();
//            uchar val2 = opIt.value<uchar>();
//            uchar val1 = operand1.value<uchar>(n->idx);
//            if (val1 <= 0)
//                difference.ref<uchar>(n->idx) = val2;
//        }
//        //go through nonzero values of operand1, and set diff to 1 if operand2(x,y,z)==0
//        for (cv::SparseMatConstIterator opIt=operand1.begin(); opIt != operand1.end(); ++opIt)
//        {
//            const cv::SparseMat::Node* n = opIt.node();
//            uchar val1 = opIt.value<uchar>();
//            uchar val2 = operand2.value<uchar>(n->idx);
//            if (val2 <= 0)
//                difference.ref<uchar>(n->idx) = val1;
//        }

//        //all of the remaining points of difference should be equal to 0.

//        cout << "Difference calculated [" << counter << ", " << counter - 1 << "], nonzero: " << (int)difference.nzcount() << endl;
//        volumeObjectDifferences.push_back(difference);
//        counter++;
//        difference.release();
//    }
//    operand1.release();
//    operand2.release();

//    return volumeObjectDifferences;
//}
