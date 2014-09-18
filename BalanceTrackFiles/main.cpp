#include <QCoreApplication>
#include <QDir>
#include <QMap>
#include <QPair>
#include <iostream>

using namespace std;


#define TARGET_COUNT 100
#define BALANCE_VMTs


QMap<int, QString> g_ClassNames;
QMultiMap<int, QString> actionToFile; //<actionNr, trackFileName>

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

    QDir fileFolder("/home/emredog/LIRIS-data/training-validation_features/20140917_args32x32x32-2x2x2_icosa/balanced_20140918");


    QStringList filters;
    filters << "*.out";

    //get track files
    QStringList targetFiles = fileFolder.entryList(filters, QDir::Files | QDir::NoDotAndDotDot);

    foreach (QString fileName, targetFiles)
    {
        QString actionName;
        int i = 0;
        while (i <= 10 && actionName.isEmpty())
        {
            if (fileName.contains(g_ClassNames[i]))
                actionName = g_ClassNames[i];
            i++;
        }

        counters[g_ClassNames.key(actionName)]++;
        actionToFile.insert(g_ClassNames.key(actionName), fileName);
    }


    //print counts before balancing
    cout << endl << endl << "Counts before balancing:" << endl;
    for (int i=0; i<11; i++)
        cout << "\t" << g_ClassNames[i].toStdString() << ": " << counters[i] << endl;

    //balance the data
    QList<int> keys = actionToFile.uniqueKeys();
    foreach (int actionNr, keys)
    {
        if ((counters[actionNr] <= TARGET_COUNT /*&& actionNr != 4) || //FIXME: special treatment for ENTER/LEAVE action
                (actionNr == 4 && counters[4] < TARGET_COUNT/2*/)
                ) //track count is less than the target count
            continue; //dont balance for this action type

        QStringList filesForThisAction = actionToFile.values(actionNr);
        filesForThisAction.sort();

        int index = 0;
        int aimedCount = /*actionNr == 4 ? TARGET_COUNT/2 :*/ TARGET_COUNT;
        while (filesForThisAction.length() > aimedCount)
        {
            index = qrand();

            index = index % filesForThisAction.length();

            QFile file(fileFolder.absoluteFilePath(filesForThisAction[index]));

            if (file.exists())
                file.remove();

            filesForThisAction.removeAt(index);
            cout << "-";
        }

        //update counters
        counters[actionNr] = filesForThisAction.length();

        cout << endl;
    }

    //print counts after balancing
    cout << endl << endl << "Track counts after balancing:" << endl;
    for (int i=0; i<11; i++)
        cout << "\t" << g_ClassNames[i].toStdString() << ": " << counters[i] << endl;




}
