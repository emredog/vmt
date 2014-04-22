#include <QDir>
#include <iostream>
#include <QTextStream>

#include "../AnalyzeAnnotations/action.h"
#include "../AnalyzeAnnotations/video.h"
#include "../AnalyzeAnnotations/boundingbox.h"

#define OVERLAP 0.5
#define FIXED_DURATION 40

using namespace std;

QMap<int, QString> g_ClassNames;

int main(int argc, char *argv[])
{
    const int slideAmount = OVERLAP*FIXED_DURATION;

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

    QDir annotationRoot("/home/emredog/LIRIS-data/test_tracklets");
    QStringList filters;
    filters << "*.xml";

    QStringList annotFiles = annotationRoot.entryList(filters, QDir::Files | QDir::NoDotAndDotDot);

    foreach(QString annotFile, annotFiles)
    {
        cout << "Processing " << annotFile.toStdString() << endl;

        Video vid(annotationRoot.absoluteFilePath(annotFile));

        if (vid.actions.isEmpty())
            continue;

        QString name = vid.name;
        if (name.endsWith(".xml"))
            name.chop(4);
        if (name.startsWith("d1"))
            name = name.remove(0, 3);

        foreach(Action action, vid.actions) //for each action
        {
            QMap<int, BoundingBox> bboxes = action.boundingBoxes;

            int beginSW = bboxes.keys().first(); //set the begining of the sliding windows as the first frame
            int endSW = 0;

            while(beginSW < bboxes.keys().last() && endSW != bboxes.keys().last()) //for each sliding window
            {
                endSW = beginSW + FIXED_DURATION; // set the ending of a sliding window

                if (endSW > bboxes.keys().last()) //if remaining frames are less than duration
                    endSW = bboxes.keys().last();  //set the end as the last frame


                QFile trackfile(annotationRoot.absoluteFilePath(QString("%1_%2_(%3-%4).track")
                                                                .arg(name)
                                                                .arg(QString::number(action.number))
                                                                .arg(beginSW)
                                                                .arg(endSW)));

                if (!trackfile.open(QFile::WriteOnly))
                {
                    cout << "CAN'T OPEN FILE FOR WRITING: " << trackfile.fileName().toStdString();
                }

                QTextStream txtStream(&trackfile);

                //<frame-number> <top-left-x-position> <top-left-y-position> <width> <height>
                for (int key=beginSW; key<=endSW; key++)
                {
                    BoundingBox bbox = action.boundingBoxes.value(key);
                    if (bbox.frameNr == -1) //no bbox with this frame index
                    {
                        cout << "Warning: No BBOX found for Vid: " << vid.name.toStdString()
                             << " Act: " << g_ClassNames.value(action.activityClass).toStdString()
                             << " Frame: " << key << endl;
                        continue;
                    }

                    txtStream << bbox.frameNr << " "
                              << bbox.x << " "
                              << bbox.y << " "
                              << bbox.width << " "
                              << bbox.height << endl;
                }

                txtStream.flush();
                trackfile.close();


                beginSW += slideAmount;
            }
        }
    }

}
