#include <QCoreApplication>
#include <QStringList>
#include <QMap>
#include <QDir>
#include <QTextStream>
#include <iostream>

using namespace std;

//#define TRAINING
//#define ACT_NAME_MISSING
//#define BOW_PATH_withNOACTION_CLASS "/home/emredog/LIRIS-data/training-validation_BagOfWords/3rdRun_BoW_01/with_K-Means_s500K_k4000_C100_e0.1/"

void formatTrainingData(QString pathToBows, QString outFile);
void formatTrainingDataActNameMissing(QString pathToBoWs, QString bowPathWithNoActionClass, QString outFile);
void formatTestData(QString pathToBoWs, QString outFile);

int main(/*int argc, char *argv[]*/)
{

    //parameters
#ifdef TRAINING
    QString pathToBoWs = "/home/emredog/LIRIS-data/20140926_test_with_20/KS/train-val_BoW";
    QString dataFileForLibSVM = "training-validation_data.dat";
#else
    QString pathToBoWs = "/home/emredog/gsu-data/test_BagOfWords/camera2";
    QString dataFileForLibSVM = "test_data.dat";
#endif

#ifdef TRAINING
#ifdef ACT_NAME_MISSING
    formatTrainingDataActNameMissing(pathToBoWs, BOW_PATH_withNOACTION_CLASS, dataFileForLibSVM);
#else
    formatTrainingData(pathToBoWs, dataFileForLibSVM);
#endif
#else
    formatTestData(pathToBoWs, dataFileForLibSVM);
#endif



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

void formatTrainingDataActNameMissing(QString pathToBoWs, QString bowPathWithNoActionClass, QString outFile)
{
    //prepare class names
    QMap<QString, int> classNames;
    classNames["no-action"] = 0;
    classNames["discussion"] = 1;
    classNames["give"] = 2;
    classNames["box-desk"] = 3;
    classNames["enter-leave"] = 4;
    classNames["try-to-enter"] = 5;
    classNames["unlock-enter"] = 6;
    classNames["baggage"] = 7;
    classNames["handshaking"] = 8;
    classNames["typing"] = 9;
    classNames["telephone"] = 10;

    //get BoW files
    QDir dirBoWs(pathToBoWs);
    QDir dirBoWsWithNoAct(bowPathWithNoActionClass);

    QStringList filters;  filters << "*.BoW";
    QStringList BoWFileNames = dirBoWs.entryList(filters, QDir::Files | QDir::NoDotAndDotDot);
    QStringList BoWFileNamesWithNoAct = dirBoWsWithNoAct.entryList(filters, QDir::Files | QDir::NoDotAndDotDot);

    cout << "Obtained " << BoWFileNames.count() << " BoW files."  << endl;
    cout << "Obtained " << BoWFileNamesWithNoAct.count() << " BoW files with No-Action class."  << endl;

    //create & prepare the output file
    QFile outputFile(dirBoWs.absoluteFilePath(outFile));
    if (!outputFile.open(QIODevice::Truncate | QIODevice::WriteOnly))
        cout << "ERROR opening file: " << dirBoWs.absoluteFilePath(outFile).toStdString() << endl;
    QTextStream dataOut(&outputFile);

    foreach(QString bowName, BoWFileNames)
    {
        QString curClassName;
        {   //FIXME
            QStringList parts = bowName.split("_");
            QString searchTerm = QString("%1_%2").arg(parts[0]).arg(parts[1]);
            foreach(QString bowNameWithNoAct, BoWFileNamesWithNoAct)
            {
                if (bowNameWithNoAct.startsWith(searchTerm))
                {
                    parts = bowNameWithNoAct.split("_");
                    curClassName = parts[2];
                    break;
                }
            }
        }

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
        int classIndex = classNames.value(curClassName, -1);

        if (classIndex < 0)
        {
            cout << "ERROR reading class name for: " << dirBoWs.absoluteFilePath(bowName).toStdString() << endl;
            continue;
        }

        dataOut << classIndex << " ";

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
    classNames["no-action"] = 0;
    classNames["discussion"] = 1;
    classNames["give"] = 2;
    classNames["box-desk"] = 3;
    classNames["enter-leave"] = 4;
    classNames["try-to-enter"] = 5;
    classNames["unlock-enter"] = 6;
    classNames["baggage"] = 7;
    classNames["handshaking"] = 8;
    classNames["typing"] = 9;
    classNames["telephone"] = 10;

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
        QString curClassName;
        {
            QStringList parts = bowName.split("_");
            curClassName = parts[2];
        }


        if (curClassName.contains("."))
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
        int classIndex = classNames.value(curClassName, -1);

        if (classIndex < 0)
        {
            cout << "ERROR reading class name for: " << dirBoWs.absoluteFilePath(bowName).toStdString() << endl;
            continue;
        }

        dataOut << classIndex << " ";

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
