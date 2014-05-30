#include <QCoreApplication>
#include <QDir>
#include <iostream>
#include <QTextStream>
#include <QtDebug>
#include "../AnalyzeAnnotations/video.h"


using namespace std;



#define INSTANCE_COUNT 1315

QList<int> resultFileToClassList(QString file);
Video mergeSuccessiveActions(Video vid);

QMap<int, QString> g_ClassNames;

int main(int argc, char *argv[])
{

    QCoreApplication a(argc, argv);

    QDir trackDir("/home/emredog/LIRIS-data/test_tracklets_20140424");
    QString resultFile = "/home/emredog/LIRIS-data/SVM-20140526/results/result15-nuSVC-RBF";
    QDir targetDir(QString("%1-Annotations").arg(resultFile));
    if (!targetDir.exists())
        targetDir.mkdir(targetDir.absolutePath());
    QDir noactionDir(QString("%1/NoAction").arg(targetDir.absolutePath()));
    if (!noactionDir.exists())
        noactionDir.mkdir(noactionDir.absolutePath());



    QList<int> classIndices = resultFileToClassList(resultFile);

    if (classIndices.length() != INSTANCE_COUNT)
    {
        cout << "[ERROR] Obtained prediction result count is different than " << INSTANCE_COUNT << endl;
        return -1;
    }
    cout << "Obtained " << classIndices.length() << " prediction result from file." << endl;

    QStringList filters;
    filters << "*.track";

    QStringList trackFiles = trackDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

    if (trackFiles.length() != INSTANCE_COUNT)
    {
        cout << "[ERROR] Obtained *.track file count is different than " << INSTANCE_COUNT << endl;
        return -1;
    }
    cout << "Obtained " << trackFiles.length() << " *.track files." << endl;

    int lastActionIndex = 1;
    int lastNoActionIndex = 1;
    Video currentVideo;
    Video noActionVideo;

    QRegExp rx("(\\_|\\(|\\-|\\)|\\.)");


    for(int i=0; i<trackFiles.length(); i++)
    {
        QString fileName = trackFiles[i];
        Action actionFromTrack;
        actionFromTrack.activityClass = classIndices[i];

        if (!actionFromTrack.fillBoxesFromTrackFile(trackDir.absoluteFilePath(fileName)))
        {
            cout << "ERROR reading Action from *.track file for " << fileName.toStdString() << endl;
            continue;
        }

        QStringList elmnts = fileName.split(rx, QString::SkipEmptyParts);

        QString vidName = elmnts[0];
        if (vidName.endsWith(".xml"))
            vidName.chop(4);
        if (vidName.startsWith("d1"))
            vidName = vidName.remove(0, 3);

        //if it's a new video (vid03 ---> vid05)
        if (vidName.compare(currentVideo.name) != 0)
        {
            if (!currentVideo.actions.keys().isEmpty())
            {
                //check for successive actions
                Video mergedCurrentVid = mergeSuccessiveActions(currentVideo);
                //write video to annotation
                mergedCurrentVid.writeWithAnnotationFormat(targetDir.absoluteFilePath(QString("%1.xml").arg(mergedCurrentVid.name)));
            }
            else
            {
                currentVideo.writeWithAnnotationFormat(targetDir.absoluteFilePath(QString("%1.xml").arg(currentVideo.name)));
            }

            if (!noActionVideo.actions.keys().isEmpty())
            {
                //check for successive actions
                Video mergedNoActionVid = mergeSuccessiveActions(noActionVideo);
                //write video to annotation
                mergedNoActionVid.writeWithAnnotationFormat(noactionDir.absoluteFilePath(QString("%1_NoAction.xml").arg(mergedNoActionVid.name)));

            }

            lastActionIndex = 1;
            lastNoActionIndex = 1;
            currentVideo.actions.clear();
            currentVideo.name = vidName;
            noActionVideo.actions.clear();
            noActionVideo.name = vidName;

        }

        if (actionFromTrack.activityClass == 0) //no action class
        {
            actionFromTrack.number = lastNoActionIndex;
            noActionVideo.actions.insert(lastNoActionIndex, actionFromTrack);
            lastNoActionIndex++;
        }
        else
        {
            actionFromTrack.number = lastActionIndex;
            currentVideo.actions.insert(lastActionIndex, actionFromTrack);
            lastActionIndex++;
        }

        cout << ".";
    }

    cout << "Completed." << endl;
    //return a.exec();
}


Video mergeSuccessiveActions(Video vid)
{
    if (vid.actions.keys().length() == 1) //just one action, no need to check
        return vid;

    Video mergedVideo;
    mergedVideo.name = vid.name;

    int lastActionIndex = 1;


    QList<int> keys = vid.actions.keys();

    foreach(int keyCurrent, keys)
    {

        QList<int> restOfKeys = keys;
        restOfKeys.removeAll(keyCurrent);

        foreach(int keyOther, restOfKeys)
        {
            // keys are ordered but there're gaps (2,3,5,6,9,...)
            Action currentAct = vid.actions[keyCurrent];
            Action nextAct = vid.actions[keyOther];

            //if same activity
            if (currentAct.activityClass == nextAct.activityClass)
            {
                QMap<int, BoundingBox>::iterator lastIt = currentAct.boundingBoxes.end();
                lastIt--;
                int lastFrame = lastIt.key();
                int firstFrame = nextAct.boundingBoxes.begin().key();
               // qDebug() << "Checking frames: " << lastFrame << " vs. " << firstFrame << "\n";

                //if successive
                if (qAbs(lastFrame-firstFrame) < 1)
                {
                    //qDebug() << "\tFound one to merge!" << endl;
                    Action mergedAct;
                    mergedAct.activityClass = currentAct.activityClass;
                    mergedAct.number = lastActionIndex;
                    mergedAct.boundingBoxes = currentAct.boundingBoxes; //take all from current action

                    QMapIterator<int, BoundingBox> itNext(nextAct.boundingBoxes); //take one by one from next action
                    while (itNext.hasNext())
                    {
                        itNext.next();
                        mergedAct.boundingBoxes.insert(itNext.key(), itNext.value());
                    }

                    mergedVideo.actions.insert(lastActionIndex, mergedAct);
                    lastActionIndex++;
                }
            }
            else //no successive activities
            {
                currentAct.number = lastActionIndex;
                mergedVideo.actions.insert(lastActionIndex, currentAct);
                lastActionIndex++;
            }
        }


    }

    return mergedVideo;
}

QList<int> resultFileToClassList(QString file)
{
    QFile txtFile(file);

    if (!txtFile.open(QFile::ReadOnly))
        return QList<int>();

    QTextStream txtStream(&txtFile);

    QList<int> classNumbers;

    while(!txtStream.atEnd())
    {
        QString line = txtStream.readLine();
        classNumbers.append(line.toInt());
    }

    return classNumbers;
}
