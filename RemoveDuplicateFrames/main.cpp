#include <QCoreApplication>
#include <QDir>
#include <QStringList>

#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    if (argc != 2)
    {
        cout << "Usage: ./RemoveDuplicateFrames <dir>\n";
        return -1;
    }

    QString argument =  QString::fromAscii(argv[1]);

    QDir dirToClean(argument);

    QStringList filters;
    filters << "*.png";

    QStringList entries = dirToClean.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

    cout << "Processing files: " << entries.length() << endl;


    quint64 prevSize = 0;

    int removeCounter = 0;

    foreach (QString fileName, entries)
    {
        QFile file(dirToClean.absoluteFilePath(fileName));
        quint64 size = file.size();

        if (size == prevSize)
        {
            removeCounter++;
            file.remove();
        }

        prevSize = size;
    }

    cout << "Duplicates removed: " << removeCounter << endl << endl;

    return 0;
}
