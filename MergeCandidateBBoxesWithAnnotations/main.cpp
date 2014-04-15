#include <QCoreApplication>
#include <QDir>
#include <QXmlStreamReader>
#include <iostream>

#include "../AnalyzeAnnotations/video.h"
using namespace std;

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
    QDir candidBBoxRoot("/home/emredog/LIRIS-data/training-validation_candidateBoundingBoxes");

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

        Action noAction;
        noAction.number = 0; //for no-action class, this will always be 0
        noAction.activityClass = 0; //no-action

        if (annotVideo.name.compare(cBboxVideo.name) != 0) //different names
        {
            cout << "Video names don't match!!" << endl
                 << annotVideo.name.toStdString() << " vs. " << cBboxVideo.name.toStdString() << endl;
            continue;
        }

        //there should be only one action in the cBboxVideo
        Action unknownAction = cBboxVideo.actions[-1];

        //start processing
        foreach(Action annotatedAction, annotVideo.actions) //for each action in annotated video
        {
            //get bounding boxes from annotated video
            QMultiMap<int, BoundingBox> annotatedBboxes = annotatedAction.boundingBoxes;

            //QList<int> frameNrs =  annotatedBboxes.keys();
            QList<int> frameNrs =  unknownAction.boundingBoxes.keys();

            //for each frame
            foreach(int frameNr, frameNrs)
            {
                BoundingBox annotatedBbox = annotatedBboxes.value(frameNr); //there should be only one or zero in the annotatedBboxes
                //if there's no annotated box for this frame, default constructor sets the framenr to -1

                QList<BoundingBox> candidateBboxes = unknownAction.boundingBoxes.values(frameNr);

                //if any of the candidate bounding box does NOT overlap with the annotated bounding box
                foreach (BoundingBox bbox, candidateBboxes)
                {
                    //if there's no overlap OR no ground truth for this frame
                    if (!bbox.overlaps(annotatedBbox) || annotatedBbox.frameNr == -1)
                    {
                        //add it to NoAction class
                        noAction.boundingBoxes.insert(frameNr, bbox);
                    }
                }
            }
        }

        //video processing is completed. first add the existing actions from annotated video
        mergedVideo.actions.unite(annotVideo.actions);
        mergedVideo.actions.insert(noAction.number, noAction);

        QString fullPath = targetDir.absoluteFilePath(mergedVideo.name.remove(0,3).append(".xml"));
        mergedVideo.writeWithAnnotationFormat(fullPath);

        cout << annotVideo.name.toStdString() << " is processed. " << annotationFiles.length() - i << " files remaining." << endl;
    }



    cout << "Finished." << endl;
    return a.exec();
}
