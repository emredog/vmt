#include <QCoreApplication>
#include <QDir>
#include <QXmlStreamReader>
#include <iostream>

#include "../AnalyzeAnnotations/video.h"
using namespace std;

#define NO_ACTION 0

QMap<int, QString> g_ClassNames;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    g_ClassNames[0] = "no-action";
    g_ClassNames[1] = "discussion";
    g_ClassNames[2] = "give";
    g_ClassNames[3] = "box-desk";
    g_ClassNames[4] = "enter-leave";
    g_ClassNames[5] = "try-to-enter";
    g_ClassNames[6] = "unlock-enter";
    g_ClassNames[7] = "baggage";
    g_ClassNames[8] = "handshaking";
    g_ClassNames[9] = "typing";
    g_ClassNames[10] = "telephone";

    QStringList filters;
    filters << "*.xml";

    QDir annotationRoot("/home/emredog/LIRIS-data/training-validation_annotations");
    //these files are extracted with Bingbing's code (HumanDetections part),
    //and then tranformed into Annotation XML form with additional Score attribute
    QDir candidBBoxRoot("/home/emredog/LIRIS-data/training-validation_tracklets");

    QDir targetDir("/home/emredog/LIRIS-data/training-validation_annotations-with-NO-ACTION");

    QStringList annotationFiles = annotationRoot.entryList(filters, QDir::Files, QDir::Name);
    QStringList candidateBBoxFiles = candidBBoxRoot.entryList(filters, QDir::Files, QDir::Name);

    if (annotationFiles.length() != candidateBBoxFiles.length())
    {
        cout << "Annotation file count is different than candidate box file count!"  << endl;        
        return EXIT_FAILURE;
    }
    else
    {
        cout << "Obtained " << annotationFiles.length() << " (x2) annotation files." << endl;
    }

    for (int i=0; i<annotationFiles.length(); i++) //for each video
    {
        QString annotFile = annotationRoot.absoluteFilePath(annotationFiles[i]);
        QString cBboxFile = candidBBoxRoot.absoluteFilePath(candidateBBoxFiles[i]);

        Video annotVideo(annotFile);
        Video cBboxVideo(cBboxFile);
        Video mergedVideo;
        mergedVideo.name = annotVideo.name;

        if (!cBboxVideo.name.contains(annotVideo.name) &&
                annotVideo.name.compare(cBboxVideo.name) != 0) //may be "d1/video0002.xml" and "d1/video0002"
        {
            //different names:
            cout << "Video names don't match!!" << endl
                 << annotVideo.name.toStdString() << " vs. " << cBboxVideo.name.toStdString() << endl;
            continue;
        }

        //get unknown actions from bingbing's detections:
        QList<Action> unknownActions = cBboxVideo.actions.values();

        //start processing
        foreach(Action annotatedAction, annotVideo.actions) //for each action in annotated video
        {
            //remove "similar/equal" from unknown actions
           if (unknownActions.contains(annotatedAction))
               unknownActions.removeAll(annotatedAction);
        }

        //video processing is completed. first add the existing actions from annotated video
        mergedVideo.actions.unite(annotVideo.actions);
        int maxKey = mergedVideo.actions.keys().last();

        for (int k=0; k<unknownActions.length(); k++)
        {
            unknownActions[k].activityClass = NO_ACTION; //set it to NO-ACTION
            unknownActions[k].number = maxKey+k+1;
            mergedVideo.actions.insert(maxKey+k+1, unknownActions[k]);
        }

        QString fullPath = targetDir.absoluteFilePath(mergedVideo.name.remove(0,3).append(".xml"));
        mergedVideo.writeWithAnnotationFormat(fullPath);

        cout << annotVideo.name.toStdString() << " is processed. " << annotationFiles.length() - i << " files remaining." << endl;
    }



    cout << "Finished." << endl;
    return a.exec();
}
