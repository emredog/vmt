#include <QCoreApplication>
#include <QDir>
#include <QMap>
#include <QPair>
#include <iostream>

using namespace std;


#define TARGET_COUNT 100


QMap<int, QString> g_ClassNames;
QMultiMap<int, QString> actionToTrack; //<actionNr, trackFileName>

int main()
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

    int counters[] = {0,0,0,0,0,0,0,0,0,0,0};

    QDir trackFolder("/home/emredog/LIRIS-data/training-validation_annotations-with-NO-ACTION-SLIDING_WINDOWS/balanced_2");

    QStringList filters;
    filters << "*.track";

    //get track files
    QStringList trackFiles = trackFolder.entryList(filters, QDir::Files | QDir::NoDotAndDotDot);

    //count track files
    QStringList nameParts;
    foreach (QString trackName, trackFiles)
    {
        nameParts = trackName.split("_");
        counters[g_ClassNames.key(nameParts[2])]++;
        actionToTrack.insert(g_ClassNames.key(nameParts[2]), trackName);
    }

    //print counts before balancing
    cout << endl << endl << "Track counts before balancing:" << endl;
    for (int i=0; i<11; i++)
        cout << "\t" << g_ClassNames[i].toStdString() << ": " << counters[i] << endl;

    //balance the data
    QList<int> keys = actionToTrack.uniqueKeys();
    foreach (int actionNr, keys)
    {
        if ((counters[actionNr] < TARGET_COUNT && actionNr != 4) || //FIXME: special treatment for ENTER/LEAVE action
                (actionNr == 4 && counters[4] < TARGET_COUNT/2)
                ) //track count is less than the target count
            continue; //dont balance for this action type

        QStringList tracksForThisAction = actionToTrack.values(actionNr);
        tracksForThisAction.sort();

        int index = 0;
        int aimedCount = actionNr == 4 ? TARGET_COUNT/2 : TARGET_COUNT; //FIXME: special treatment for ENTER/LEAVE action
        while (tracksForThisAction.length() > aimedCount)
        {
            cout << "+";
            QString videoActionPrefix = tracksForThisAction[index];
            videoActionPrefix.chop(videoActionPrefix.length() - 10);

            int plusRange = 0;
            while (index+plusRange < tracksForThisAction.length() &&
                   tracksForThisAction[index+plusRange].startsWith(videoActionPrefix))
                plusRange++;

            for (int range=0; range<plusRange && index+range<tracksForThisAction.length(); range++) //for components of this action
            {
                if (range%2 == 1 && range != plusRange-1) //delete the first and last one, skip every other
                    continue;

                QFile file(trackFolder.absoluteFilePath(tracksForThisAction[index+range]));
                if (file.exists())
                    file.remove();

                tracksForThisAction.removeAt(index+range);
                cout << "-";
            }




            index += plusRange;
            if (index >= tracksForThisAction.length()) index = 0;
        }

        //update counters
        counters[actionNr] = tracksForThisAction.length();

        cout << endl;
    }

    //print counts after balancing
    cout << endl << endl << "Track counts after balancing:" << endl;
    for (int i=0; i<11; i++)
        cout << "\t" << g_ClassNames[i].toStdString() << ": " << counters[i] << endl;




}
