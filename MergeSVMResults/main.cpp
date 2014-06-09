#include <QCoreApplication>
#include <QDir>
#include <iostream>
#include <QTextStream>
#include <QDomDocument>


using namespace std;

void modifyAnnotation(QDir trackDir, QString svmPath, QString target, QString actionName, int classIndex); //actionName = "vid0003_1_"
void checkVideoName(QString fullPathToXmlFile);
void chechActionIndices(QString fullPathToXmlFile);

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QDir trackDir("/home/emredog/LIRIS-data/test_tracklets_20140424");

    QString resultFile = "result15-nuSVC-Lin";

    QString svmPath = "/home/emredog/LIRIS-data/SVM-20140526/results/";
    QString resultFullPath = QString("%1%2").arg(svmPath).arg(resultFile);

    QStringList filters;
    filters << "*.track";

    QList<int> predictions;

    QStringList trackFiles = trackDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

    {//read predictions
        QFile txtFile(resultFullPath);
        if (!txtFile.open(QFile::ReadOnly))
            return -1;

        QTextStream txtStream(&txtFile);

        while (!txtStream.atEnd())
        {
            QString line = txtStream.readLine();
            predictions.append(line.toInt());
        }
    }

    if (predictions.length() != trackFiles.length())
        return -1;

    //prepare the map
    QMap<QString, int> trackFileToPrediction; //track file, prediction result
    for (int i=0; i<predictions.length(); i++)
        trackFileToPrediction[trackFiles[i]] = predictions[i];

    QString prefix = "qwertyuiASDASDASD";
    QList<int> votingList;

    for (int i=0; i<trackFiles.length(); i++)
    {
        if (!trackFiles[i].startsWith(prefix)) //new action
        {
            //do the voting
            if (!votingList.isEmpty()) //if there are some votings done
            {
                QList<int> occurences;
                for (int classIndex = 0; classIndex<=10; classIndex++)
                    occurences.append(votingList.count(classIndex)); //count elements

                QList<int> copyOccurences(occurences);
                qSort(copyOccurences);
                int maxOccurence = copyOccurences.last();
                int votedClass = occurences.indexOf(maxOccurence);
                modifyAnnotation(trackDir, svmPath, QString("%1-mergedAnnotations").arg(resultFile), prefix, votedClass);
            }

            //prepare for next action
            prefix = trackFiles[i].mid(0, 10);
            votingList.clear();
        }

        votingList.append(trackFileToPrediction[trackFiles[i]]);
    }

    //do the voting one more time""
    if (!votingList.isEmpty()) //if there are some votings done
    {
        QList<int> occurences;
        for (int classIndex = 0; classIndex<=10; classIndex++)
            occurences.append(votingList.count(classIndex)); //count elements

        QList<int> copyOccurences(occurences);
        qSort(copyOccurences);
        int maxOccurence = copyOccurences.last();
        int votedClass = occurences.indexOf(maxOccurence);
        modifyAnnotation(trackDir, svmPath, QString("%1-mergedAnnotations").arg(resultFile), prefix, votedClass);
    }


    //fix remaining things:

    filters.clear();
    filters << "*.xml";
    QDir resultingXmlDir( QString("%1-mergedAnnotations").arg(resultFullPath));
    QStringList xmlFiles = resultingXmlDir.entryList(filters, QDir::Files | QDir::NoDotAndDotDot, QDir::Name);
    foreach(QString xmlFile, xmlFiles)
    {
        chechActionIndices(resultingXmlDir.absoluteFilePath(xmlFile));
        checkVideoName(resultingXmlDir.absoluteFilePath(xmlFile));
    }


}

void modifyAnnotation(QDir trackDir, QString svmPath, QString target, QString actionName, int classIndex)
{
    QStringList parts = actionName.split("_");
    int actionIndex = parts[1].toInt();
    QString fileName = QString("%1.xml").arg(parts[0]);
    cout << "Set " << trackDir.absoluteFilePath(fileName).toStdString() << " - Action " << actionIndex << " to " << classIndex << endl;

    QString targetDirPath = svmPath + target;

    QDir annotDir(targetDirPath);
    if (!annotDir.exists()) annotDir.mkdir(targetDirPath);

    QFile::copy(trackDir.absoluteFilePath(fileName), annotDir.absoluteFilePath(fileName));
    QFile annotFile(annotDir.absoluteFilePath(fileName));
    annotFile.open(QFile::ReadWrite);

    QDomDocument xmlFile(parts[0]);
    xmlFile.setContent(&annotFile);



    QDomNodeList actNodes = xmlFile.elementsByTagName("action");

    for (int i=0; i<actNodes.length(); i++)
    {
        QDomNode node = actNodes.at(i);
        QDomElement elmnt = node.toElement();
        QString nrStr = elmnt.attribute("nr");
        int nr = nrStr.toInt();
        if (nr != actionIndex)
            continue;

        if (classIndex != 0)
            elmnt.setAttribute("class", classIndex);
        else
        {
            QDomNode parentNode = elmnt.parentNode();
            parentNode.removeChild(elmnt);
        }

        break;
    }    

    // Write changes to same file
    annotFile.resize(0);
    QTextStream stream;
    stream.setDevice(&annotFile);
    xmlFile.save(stream, 4);

    annotFile.close();
}

void checkVideoName(QString fullPathToXmlFile)
{
    QFile annotFile(fullPathToXmlFile);
    annotFile.open(QFile::ReadWrite);

    QDomDocument xmlFile("someName");
    xmlFile.setContent(&annotFile);

    QDomNodeList actNodes = xmlFile.elementsByTagName("videoName");
    QDomNode node = actNodes.at(0);
    QDomElement elmnt = node.toElement();
    QString val = elmnt.text();

    if (val.endsWith(".xml"))
    {

        val.chop(4);
        elmnt.firstChild().setNodeValue(val);
    }

    // Write changes to same file
    annotFile.resize(0);
    QTextStream stream;
    stream.setDevice(&annotFile);
    xmlFile.save(stream, 4);

    annotFile.close();


}

void chechActionIndices(QString fullPathToXmlFile)
{
    QFile annotFile(fullPathToXmlFile);
    annotFile.open(QFile::ReadWrite);

    QDomDocument xmlFile("someName");
    xmlFile.setContent(&annotFile);

    QDomNodeList actNodes = xmlFile.elementsByTagName("action");

    int count = actNodes.length();
    for (int i=0; i<actNodes.length(); i++)
    {
        QDomNode node = actNodes.at(i);
        QDomElement elmnt = node.toElement();
        QString nrStr = elmnt.attribute("nr");
        int nr = nrStr.toInt();
        if (nr != i+1)
            elmnt.setAttribute("nr", i+1);
    }


    // Write changes to same file
    annotFile.resize(0);
    QTextStream stream;
    stream.setDevice(&annotFile);
    xmlFile.save(stream, 4);

    annotFile.close();

}
