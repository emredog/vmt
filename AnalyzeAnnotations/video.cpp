#include "video.h"
#include "action.h"
#include "boundingbox.h"

#include <iostream>
using namespace std;

#include <QFile>
#include <QXmlStreamReader>

extern QMap<int, QString> g_ClassNames;

Video::Video(QString fullPath)
{
    QFile annotFile(fullPath);
    if (!annotFile.open(QFile::ReadOnly))
        return;

    QXmlStreamReader xmlReader(&annotFile);

    int currentActionNumber = -1;

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

                bool ok1, ok2;
                action.activityClass = attributes.value("class").toString().toInt(&ok1);
                action.number = attributes.value("nr").toString().toInt(&ok2);
                if (!ok1 || !ok2)
                {
                    std::cout << "Read error on <action> 'class' or 'nr' !!!" << std::endl;
                    break;
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
                //from <action nr="..." class="...">


                this->actions[currentActionNumber].boundingBoxes[boundingBox.frameNr] = boundingBox;

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
