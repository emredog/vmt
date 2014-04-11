#include <QCoreApplication>
#include <QStringList>
#include <QMap>
#include <QDir>
#include <QTextStream>
#include <iostream>

using namespace std;

#define TRAINING 0

void formatTrainingData(QString pathToBows, QString outFile);
void formatTestData(QString pathToBoWs, QString outFile);

int main(/*int argc, char *argv[]*/)
{


    //parameters
//    QString pathToBoWs = "/home/emredog/LIRIS-data/training-validation_BagOfWords/training-validation_BoW_params03/with_K-Means_s500K_k4000_C100_e0.1";
//    QString dataFileForLibSVM = "training-validation_data.dat";
    QString pathToBoWs = "/home/emredog/LIRIS-data/test_BagOfWords/test_BoWs_params03/with_K-Means_s500K_k4000_C100_e0.1";
    QString dataFileForLibSVM = "test_data.dat";

    if (TRAINING)
        formatTrainingData(pathToBoWs, dataFileForLibSVM);
    else
        formatTestData(pathToBoWs, dataFileForLibSVM);




}

void formatTestData(QString pathToBoWs, QString outFile)
{
    //get BoW files
    QDir dirBoWs(pathToBoWs);
    QStringList filters;  filters << "*.BoW";
    QStringList BoWFileNames = dirBoWs.entryList(filters, QDir::Files | QDir::NoDotAndDotDot);

    cout << "Obtained " << BoWFileNames.count() << " BoW files."  << endl;

    //create & prepare the output file
    QFile outputFile(dirBoWs.absoluteFilePath(outFile));
    if (!outputFile.open(QIODevice::Truncate | QIODevice::WriteOnly))
        cout << "ERROR opening file: " << dirBoWs.absoluteFilePath(outFile).toStdString() << endl;
    QTextStream dataOut(&outputFile);

    foreach(QString bowName, BoWFileNames)
    {
        //read data from bow file
        QFile bowFile(dirBoWs.absoluteFilePath(bowName));
        if (!bowFile.open(QIODevice::ReadOnly))
        {
            cout << "ERROR opening file: " << dirBoWs.absoluteFilePath(bowName).toStdString() << endl;
            continue;
        }
        QTextStream in(&bowFile);

        //write <label>
        dataOut << "0 ";    //"Labels in the testing file are only used
                            // to calculate accuracy or errors. If they are unknown, just fill the
                            // first column with any numbers."

        //write <index>:<value> pairs
        int index = 1; // from svm-scale help: minimal feature index is 0, but indices should start from 1
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

void formatTrainingData(QString pathToBoWs, QString outFile)
{
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
    QFile outputFile(dirBoWs.absoluteFilePath(outFile));
    if (!outputFile.open(QIODevice::Truncate | QIODevice::WriteOnly))
        cout << "ERROR opening file: " << dirBoWs.absoluteFilePath(outFile).toStdString() << endl;
    QTextStream dataOut(&outputFile);

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
        int index = 1; // from svm-scale help: minimal feature index is 0, but indices should start from 1
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
