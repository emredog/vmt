#include <QCoreApplication>
#include <QDir>
#include <iostream>
#include <QTextStream>
#include <QtDebug>
#include "../AnalyzeAnnotations/video.h"


using namespace std;



#define INSTANCE_COUNT 694

QList<int> resultFileToClassList(QString file);
Video mergeSuccessiveActions(Video vid);

QMap<int, QString> g_ClassNames;

int main(int argc, char *argv[])
{

    QCoreApplication a(argc, argv);

    QDir trackDir("/home/emredog/LIRIS-data/test_tracklets_20140424/track_stationary_cam_subset_for_20140922");
    QString resultFile = "/home/emredog/LIRIS-data/20141104-SVM/args16-K1000/RBF-CSVC.prediction";
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

    QRegExp rx("(\\_|\\(|\\-|\\)|\\.)");

    QMultiMap<QString, QPair<QString, int> > videoNameToTrackNameAndClass;
    for(int i=0; i<trackFiles.length(); i++)
    {
        QStringList elmnts = trackFiles[i].split(rx, QString::SkipEmptyParts);
        QString vidName = elmnts[0];
        if (vidName.endsWith(".xml"))
            vidName.chop(4);
        if (vidName.startsWith("d1"))
            vidName = vidName.remove(0, 3);
        videoNameToTrackNameAndClass.insert(vidName, qMakePair(trackFiles[i], classIndices[i]));
    }


    foreach(QString vidName, videoNameToTrackNameAndClass.uniqueKeys())
    {
        Video currentVideo; //annotation to write in targetDir
        Video currentNoActionVideo; //annotation to write in noActionDir

        currentVideo.name = vidName;
        currentNoActionVideo.name = vidName;


        QList<QPair<QString, int> > trackFilesToClass = videoNameToTrackNameAndClass.values(vidName);

        int actionCounter = 1, noActionCounter = 1;
        for(int i=0; i<trackFilesToClass.length(); i++)
        {
            Action currentAction;
            currentAction.activityClass = trackFilesToClass[i].second; //get class
            currentAction.fillBoxesFromTrackFile(trackDir.absoluteFilePath(trackFilesToClass[i].first)); //fetch bounding boxes

            //check action class
            if (currentAction.activityClass == 0)  //if class is "no action"
            {
                currentAction.number = noActionCounter;
                currentNoActionVideo.actions.insert(noActionCounter, currentAction);
                noActionCounter++;
            }
            else // if another class
            {
                currentAction.number = actionCounter;
                currentVideo.actions.insert(actionCounter, currentAction);
                actionCounter++;
            }
        }
        //all track files are processed for this video

        //proceed to merging successive actions in videos
        Video mergedCurrentVid = mergeSuccessiveActions(currentVideo);
        Video mergedCurrentNoActVid = mergeSuccessiveActions(currentNoActionVideo);

        //and write them to disk
        mergedCurrentVid.writeWithAnnotationFormat(targetDir.absoluteFilePath(QString("%1.xml").arg(mergedCurrentVid.name)));
        mergedCurrentNoActVid.writeWithAnnotationFormat(noactionDir.absoluteFilePath(QString("%1.xml").arg(mergedCurrentNoActVid.name)));

        cout << "Processed: " << vidName.toStdString() << endl;
    }

    cout << "Completed." << endl;

    return EXIT_SUCCESS;
}


Video mergeSuccessiveActions(Video vid)
{
    if (vid.actions.keys().length() <= 1) //just one action, no need to check
        return vid;        

    Video mergedVideo;
    mergedVideo.name = vid.name;
    int mergedVidActionCounter = 1;

    //sort actions by begining frame of bounding boxes:
    QList<Action> actions = vid.actions.values();
    qSort(actions);
    //clear the actions
    vid.actions.clear();
    //rebuild the map
    for (int i=0; i<actions.length(); i++)
    {
        vid.actions.insert(i+1, actions[i]);
    }

    Action prevAction = vid.actions.value(1); //take first action
    //start iterating the remaining actions
    for (int curKey=2; curKey <= vid.actions.keys().last(); curKey++)
    {
        Action curAction = vid.actions.value(curKey);
        if (curAction.activityClass != prevAction.activityClass) //different classes
        {
            //do not merge -- dump prevAction to video
            prevAction.number = mergedVidActionCounter;
            mergedVideo.actions.insert(mergedVidActionCounter, prevAction);
            mergedVidActionCounter++;
            //set cur --> prev
            prevAction = curAction;
            if (curKey == vid.actions.keys().last()) //if last iteration
            {
                curAction.number = mergedVidActionCounter;
                mergedVideo.actions.insert(mergedVidActionCounter, curAction);
            }
        }
        else //same classes
        {
            //we need to check frames of bounding boxes
            int curActBeginFrame = curAction.boundingBoxes.keys().first();
            int prevActBeginFrame = prevAction.boundingBoxes.keys().first();
            int prevActEndFrame = prevAction.boundingBoxes.keys().last();

            if (curActBeginFrame >= prevActBeginFrame-1 &&
                curActBeginFrame <= prevActEndFrame+1) //current action's frames is within or just after the previous frames
            {
                int curActEndFrame = curAction.boundingBoxes.keys().last();
                //enlarge prevAction with curAction's boundingboxes
                int beginFrom = prevActEndFrame > curActBeginFrame ? prevActEndFrame : curActBeginFrame;
                for (int frame = beginFrom; frame <= curActEndFrame; frame++)
                {
                    prevAction.boundingBoxes.insert(frame, curAction.boundingBoxes.value(frame));
                }

                if (curKey == vid.actions.keys().last()) //if last iteration
                {
                    prevAction.number = mergedVidActionCounter;
                    mergedVideo.actions.insert(mergedVidActionCounter, prevAction);
                }

                cout << "+";

                //DO NOT SET cur --> prev ; we already processed curAct & concatenated it to the prevAct
            }
            else //there's temporal gap between the two
            {
                //do not merge -- dump prevAction to video
                mergedVideo.actions.insert(mergedVidActionCounter, prevAction);
                mergedVidActionCounter++;
                //set cur --> prev
                prevAction = curAction;
                if (curKey == vid.actions.keys().last()) //if last iteration
                {
                    curAction.number = mergedVidActionCounter;
                    mergedVideo.actions.insert(mergedVidActionCounter, curAction);
                }
            }
        }
    }
    cout << endl;

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
