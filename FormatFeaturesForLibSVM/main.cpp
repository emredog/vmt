#include <QCoreApplication>
#include <QStringList>
#include <QMap>
#include <QDir>
#include <QTextStream>
#include <iostream>

using namespace std;

int main(/*int argc, char *argv[]*/)
{
    //parameters
    QString pathToBoWs = "/home/emredog/LIRIS-data/training-validation_BoW/with_K-Means_s500K_k4000_C100_e0.1/";
    QString dataFileForLibSVM = "data.dat";

    //prepare class names
    QMap<QString, int> classNames;
    classNames["discussion"] = 0;
    classNames["give"] = 1;
    classNames["box-desk"] = 2;
    classNames["enter-leave"] = 3;
    classNames["try-to-enter"] = 4;
    classNames["unlock-enter"] = 5;
    classNames["baggage"] = 6;
    classNames["handshaking"] = 7;
    classNames["typing"] = 8;
    classNames["telephone"] = 9;

    //get BoW files
    QDir dirBoWs(pathToBoWs);
    QStringList filters;  filters << "*.BoW";
    QStringList BoWFileNames = dirBoWs.entryList(filters, QDir::Files | QDir::NoDotAndDotDot);

    cout << "Obtained " << BoWFileNames.count() << " BoW files."  << endl;

    //create & prepare the output file
    QFile outputFile(dirBoWs.absoluteFilePath(dataFileForLibSVM));
    if (!outputFile.open(QIODevice::Truncate | QIODevice::WriteOnly))
        cout << "ERROR opening file: " << dirBoWs.absoluteFilePath(dataFileForLibSVM).toStdString() << endl;
    QTextStream dataOut(&outputFile);

    //TODO: should I only take 60% (for validation purposes) ?
    foreach(QString bowName, BoWFileNames)
    {
        QString curClassName = bowName.mid(10); //remove vidxxxx_x_ part
        curClassName.truncate(curClassName.indexOf('.')); //removes after the first point

        //read data from bow file
        QFile bowFile(dirBoWs.absoluteFilePath(bowName));
        if (!bowFile.open(QIODevice::ReadOnly))
        {
            cout << "ERROR opening file: " << dirBoWs.absoluteFilePath(bowName).toStdString() << endl;
            continue;
        }
        QTextStream in(&bowFile);

        //write <label>
        dataOut << classNames[curClassName] << " ";

        //write <index>:<value> pairs
        int index = 0;
        while (!in.atEnd())
        {
            //write <index>:
            dataOut << index << ":";
            bool ok = false;
            int val = in.readLine().toInt(&ok);

            //write <value>
            if (ok)
                dataOut << val << " ";

            index++;
        }
        //write endline
        dataOut << "\n";

        cout << ".";
    }

    cout << endl << "Data file wrote successfully.." << endl;


}
