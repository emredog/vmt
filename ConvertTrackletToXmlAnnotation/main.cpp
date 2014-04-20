#include <QCoreApplication>
#include <QDir>
#include <QXmlStreamWriter>
#include <QTextStream>
#include <QMap>
#include <QRect>


class Tracklet
{
public:
    QMap<int, QRect> bboxes; //<frame number, bounding box>

    Tracklet(QString pathToCSV)
    {
        //open file
        QFile file(pathToCSV);
        if (!file.open(QIODevice::ReadOnly))
            return;

        QTextStream in(&file);
        while (!in.atEnd())
        {
            QString line = in.readLine(); //272,72,303,183,2
            QStringList values = line.split(",");
            QPoint topLeft((int)values[0].toFloat()*2, (int)values[1].toFloat()*2);
            QPoint bottomRight((int)values[2].toFloat()*2, (int)values[3].toFloat()*2);
            int frame = values[4].toInt();

            bboxes.insert(frame, QRect(topLeft, bottomRight));
        }
    }

    bool operator==(const Tracklet& other) const
    {
        int minDuration = qMin(this->bboxes.keys().length(),other.bboxes.keys().length());
        //check for duration difference
        if (qAbs(this->bboxes.keys().length()-other.bboxes.keys().length()) > minDuration/4)
            return false;

        int bigOverlapCounter = 0;
        foreach(int i, this->bboxes.keys())
        {
            if (!other.bboxes.keys().contains(i))
                continue;

            QRect thisRect = this->bboxes[i];
            QRect otherRect = other.bboxes[i];

            QRect intersection = thisRect.intersected(otherRect);

            if (intersection.width()*intersection.height() > qMin(thisRect.width()*thisRect.height(), otherRect.width()*otherRect.height())/8)
            {
                bigOverlapCounter++;
                if (bigOverlapCounter >= minDuration/8)
                    return true;
            }
        }

        return false;
    }
};

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QDir trackletRoot("C:\\Users\\emredog\\Documents\\ADSC_NUS_Harl_result_code_v2\\Tracklets\\test");

    QStringList filters;

    QStringList trackletFolders = trackletRoot.entryList(filters, QDir::AllDirs | QDir::NoDotAndDotDot);

    filters << "*.csv";
    foreach(QString trackletFolder, trackletFolders)
    {
        QDir trackletDir(trackletRoot.absoluteFilePath(trackletFolder));

        //get all csv files in the folder
        QStringList trackletFiles = trackletDir.entryList(filters, QDir::Files);

        if (trackletFiles.empty())
            continue;

        //create XML file for the folder: vid0002
        QFile annotationFile(trackletRoot.absoluteFilePath(trackletFolder.append(".xml")));
        if (!annotationFile.open(QIODevice::Truncate | QIODevice::WriteOnly))
            continue;
        QXmlStreamWriter xmlWriter(&annotationFile);

        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();
        xmlWriter.writeStartElement("tagset");
        xmlWriter.writeStartElement("video");
        xmlWriter.writeStartElement("videoName");
        xmlWriter.writeCharacters(QString("d1/").append(trackletFolder));
        xmlWriter.writeEndElement(); //videoName

        QList<Tracklet> tracklets;

        foreach(QString trackletFile, trackletFiles)
        {
            Tracklet newTracklet(trackletDir.absoluteFilePath(trackletFile));
            if (!trackletFile.startsWith("stand") ||  //if it doesnt start with "stand"
                    (trackletFile.startsWith("stand") &&  !tracklets.contains(newTracklet))) //or it does start with "stand" BUT list doesnt contain any similar tracklet
            {
                tracklets.append(newTracklet);

                xmlWriter.writeStartElement("action");
                xmlWriter.writeAttribute("nr", QString::number(trackletFiles.indexOf(trackletFile)+1));
                xmlWriter.writeAttribute("class", "N/A");

                QList<int> frames = newTracklet.bboxes.keys();
                foreach (int fr, frames)
                {
                    QRect rect = newTracklet.bboxes[fr];
                    xmlWriter.writeStartElement("bbox");
                    xmlWriter.writeAttribute("x", QString::number(rect.topLeft().x()));
                    xmlWriter.writeAttribute("y", QString::number(rect.topLeft().y()));
                    xmlWriter.writeAttribute("width", QString::number(rect.width()));
                    xmlWriter.writeAttribute("height", QString::number(rect.height()));
                    xmlWriter.writeAttribute("framenr", QString::number(fr));
                    xmlWriter.writeEndElement(); //bbox
                }


                xmlWriter.writeEndElement(); //action
            }

        }

        xmlWriter.writeEndElement(); //video
        xmlWriter.writeEndElement(); //tagset

        xmlWriter.writeEndDocument();
    }
}


