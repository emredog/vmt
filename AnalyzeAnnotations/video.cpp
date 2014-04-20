#include "video.h"
#include "action.h"
#include "boundingbox.h"

#include <iostream>
using namespace std;

#include <QFile>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>

extern QMap<int, QString> g_ClassNames;

Video::Video()
{
    this->name = "N/A";
}

Video::Video(QString fullPath)
{    
    QFile annotFile(fullPath);
    this->name = QString("d1/").append(annotFile.fileName());

    if (!annotFile.open(QFile::ReadOnly))
        return;        

    QXmlStreamReader xmlReader(&annotFile);

    int currentActionNumber = -100;

    while (!xmlReader.atEnd() && !xmlReader.hasError())
    {
        QXmlStreamReader::TokenType token = xmlReader.readNext();
        if (token == QXmlStreamReader::StartDocument)
            continue;

        if (token == QXmlStreamReader::StartElement)
        {
            if (xmlReader.name() == "video") continue;
            if (xmlReader.name() == "videoName")
            {
                this->name = xmlReader.readElementText();
                continue;
            }

            if (xmlReader.name() == "action")//found new action:
            {
                Action action;

                QXmlStreamAttributes attributes = xmlReader.attributes();
                if (!attributes.hasAttribute("nr") || !attributes.hasAttribute("class"))
                {
                    std::cout << "Missing attributes in <action> !" << std::endl;
                    continue;
                }


                QString classStr = attributes.value("class").toString(),
                        nrStr = attributes.value("nr").toString();

                if (classStr.compare("N/A") == 0) //unknown action
                {
                    action.activityClass = -1;
                    bool ok;
                    int nr = nrStr.toInt(&ok);
                    action.number = ok? nr : -1;
                }
                else //action is read from proper annotation file
                {
                    bool ok1, ok2;
                    action.activityClass = classStr.toInt(&ok1);
                    action.number = nrStr.toInt(&ok2);
                    if (!ok1 || !ok2)
                    {
                        std::cout << "Read error on <action> 'class' or 'nr' !!!" << std::endl;
                        break;
                    }
                }

                this->actions[action.number] = action;

                currentActionNumber = action.number;
            }

            if (xmlReader.name() == "bbox") //found new bbox for current action
            {
                BoundingBox boundingBox;

                QXmlStreamAttributes attributes = xmlReader.attributes();
                if (!attributes.hasAttribute("x") || !attributes.hasAttribute("y") || !attributes.hasAttribute("width")
                        || !attributes.hasAttribute("height") || !attributes.hasAttribute("framenr") )
                {
                    std::cout << "Missing attributes in <bbox> !" << std::endl;
                    continue;
                }
                //from <bbox ... >
                boundingBox.x = attributes.value("x").toString().toInt();
                boundingBox.y = attributes.value("y").toString().toInt();
                boundingBox.width = attributes.value("width").toString().toInt();
                boundingBox.height = attributes.value("height").toString().toInt();
                boundingBox.frameNr = attributes.value("framenr").toString().toInt();

                //we may have a score attribute:
                QString scoreStr = attributes.value("score").toString();
                if (!scoreStr.isEmpty())
                {
                    boundingBox.score = scoreStr.toFloat();
                }
                //from <action nr="..." class="...">


                this->actions[currentActionNumber].boundingBoxes.insert(boundingBox.frameNr, boundingBox);

            }
        }
    }
}

int Video::getNumberOfActions()
{
    return this->actions.size();
}

void Video::printSummary()
{
    QMapIterator<int, Action> it(actions);

    while(it.hasNext())
    {
        it.next();
        Action act = it.value();

        cout << "[Video: " << this->name.toStdString() << "]" <<
               " [Action: " << g_ClassNames[act.activityClass].toStdString() << "]" <<
               " [Duration: " << act.getDuration() << "]" << endl;
    }
}

void Video::writeWithAnnotationFormat(QString fullPath)
{
    QFile annotationFile(fullPath);
    if (!annotationFile.open(QIODevice::Truncate | QIODevice::WriteOnly))
    {
        cout << "ERROR when opening file to write: " << fullPath.toStdString() << endl;
    }

    QXmlStreamWriter xmlWriter(&annotationFile);

    xmlWriter.setAutoFormatting(true);
    xmlWriter.writeStartDocument();
    xmlWriter.writeStartElement("tagset");
    xmlWriter.writeStartElement("video");
    xmlWriter.writeStartElement("videoName");
    xmlWriter.writeCharacters(this->name);
    xmlWriter.writeEndElement(); //videoName

    QMapIterator<int, Action> it(actions);
    //for each action:
    while(it.hasNext())
    {
        it.next();
        Action act = it.value();
        if (act.boundingBoxes.isEmpty())
            continue;

        xmlWriter.writeStartElement("action");
        xmlWriter.writeAttribute("nr", QString::number(act.number));
        xmlWriter.writeAttribute("class", QString::number(act.activityClass));

//        int maxKey = -1;
//        {
//            QMap<int, BoundingBox>::Iterator mit = act.boundingBoxes.end();
//            mit--;
//            maxKey = mit.key();
//        }

        for (int frame=act.boundingBoxes.keys().first(); frame<act.boundingBoxes.keys().last(); frame++)
        {
            QList<BoundingBox> boxesForThisFrame = act.boundingBoxes.values(frame);
            foreach(BoundingBox box, boxesForThisFrame)
            {
                xmlWriter.writeStartElement("bbox");
                xmlWriter.writeAttribute("x", QString::number(box.x));
                xmlWriter.writeAttribute("y", QString::number(box.y));
                xmlWriter.writeAttribute("width", QString::number(box.width));
                xmlWriter.writeAttribute("height", QString::number(box.height));
                xmlWriter.writeAttribute("framenr", QString::number(frame));
//                xmlWriter.writeAttribute("score", QString::number(box.score));
                xmlWriter.writeEndElement(); //bbox
            }
        }
        xmlWriter.writeEndElement(); //action
    }

    xmlWriter.writeEndElement(); //video
    xmlWriter.writeEndElement(); //tagset

    xmlWriter.writeEndDocument();
}
