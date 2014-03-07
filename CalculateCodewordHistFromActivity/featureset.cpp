#include "featureset.h"

#include <QTextStream>
#include <QStringList>
#include <QFile>
#include <iostream>


FeatureSet::FeatureSet(QString fileName, int preFeatureCount)
{
    this->ignoreFirstFeatures = preFeatureCount;
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        std::cout << "[ERROR] Can't open file: " << file.fileName().toStdString() << std::endl;
        return;
    }

    QTextStream in(&file);
    QString line;
    QStringList fields;
    int lineCounter = 0;
    bool ok = true;

    while (!in.atEnd())
    {
        lineCounter++;

        QList<float> featureVector;

        line = in.readLine();

        fields = line.trimmed().split(" ");
        if (fields.length() != this->featDim + preFeatureCount)
        {
            std::cout << "[ERROR] Feature dimension is not  " << this->featDim << " in "
                      << file.fileName().toStdString() << "line: " << lineCounter << std::endl;
            continue;
        }

        for(int i= preFeatureCount; i < fields.length(); i++)
        {
            QString s = fields.at(i);
            float num = s.toFloat(&ok);
            if (!ok)  { std::cout << "[ERROR] Couldn't read from text as float." << std::endl; break;}
            featureVector.append(num);
        }

        if (featureVector.length() != this->featDim)
            continue;

        this->featureVectors.append(featureVector);
    }

    std::cout << "[INFO] " << this->featureVectors.length() << " features read from "<< file.fileName().toStdString() << std::endl;
}

int FeatureSet::getNrOfFeatures() const
{

    return this->featureVectors.length();
}

const QList<QList<float> > & FeatureSet::getFeatureVectors() const
{
    return this->featureVectors;
}
