#include <iostream>
#include <QDir>
#include <QMap>

#include "video.h"
#include "action.h"

using namespace std;

void printPerActivityDuration(QList<Video> videos);
QMap<int, QString> g_ClassNames;

int main(int argc, char *argv[])
{
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

    if (argc != 2)
    {
        cout << "\n\nUsage: ./AnalyzeAnnotations <annotationFolder>\n\n";
        return -1;
    }

    QDir annotationDir(argv[1]);

    QStringList filters;
    filters << "*.xml";

    QList<Video> videos;

    QStringList annotationFiles = annotationDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot);

    if (annotationFiles.isEmpty())
    {
        cout << "No XML files found within provided directory.\n";
        return -1;
    }

    foreach(QString annotFilePath, annotationFiles)
    {
        Video vid(annotationDir.absoluteFilePath(annotFilePath));
        vid.printSummary();
        videos.append(vid);
    }

    printPerActivityDuration(videos);


}

void printPerActivityDuration(QList<Video> videos)
{
    //init
    QMap<QString, int> activityCounts;
    QMap<QString, int> activityDurationSum;
    for(int i=0; i<g_ClassNames.keys().length(); i++)
    {
        activityCounts[g_ClassNames[i]] = 0;
        activityDurationSum[g_ClassNames[i]] = 0;
    }

    foreach(Video vid, videos)
    {
        QMapIterator<int, Action> it(vid.actions);

        while(it.hasNext()) //for each action:
        {
            it.next();
            Action act = it.value();

            activityCounts[g_ClassNames[act.activityClass]]++;
            int duration = act.getDuration();
            activityDurationSum[g_ClassNames[act.activityClass]] += duration;
        }
    }

    cout << endl << endl << "Average durations by activity: " << endl << endl;
    for(int i=0; i<g_ClassNames.keys().length(); i++)
    {
        if (activityCounts[g_ClassNames[i]] == 0) continue;

        cout << g_ClassNames[i].toStdString() << ": " << activityDurationSum[g_ClassNames[i]] / activityCounts[g_ClassNames[i]] <<
                " (" << activityCounts[g_ClassNames[i]] << " actions)" << endl;
    }
}
