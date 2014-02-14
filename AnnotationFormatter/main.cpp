#include <QCoreApplication>
#include <iostream>
#include <QDir>
#include <QXmlStreamReader>
#include <QTextStream>

typedef QMap<QString, int> BBox;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    if (argc != 2)
    {
        std::cout << "Please provide a folder with XML annotations in it." << std::endl;
        return EXIT_FAILURE;
    }

    QDir workingDirectory(argv[1]);
    QStringList nameFilters;
    nameFilters << "*.xml";
    QStringList xmlFiles = workingDirectory.entryList(nameFilters, QDir::NoDotAndDotDot | QDir::Files);
    std::cout << "Files obtained: " << xmlFiles.count() << std::endl;

    foreach (QString fileName, xmlFiles)
    {
        QFile* xmlFile = new QFile(workingDirectory.absoluteFilePath(fileName));
        if (!xmlFile->open(QFile::ReadOnly))
        {
            std::cout << "Couldnt open file: " << workingDirectory.absoluteFilePath(fileName).toStdString();
            delete xmlFile;
            continue;
        }

        QXmlStreamReader xmlReader(xmlFile);
        QList<BBox> boundingBoxes;

        //read the xml
        while (!xmlReader.atEnd() && !xmlReader.hasError())
        {
            QXmlStreamReader::TokenType token = xmlReader.readNext();
            if (token == QXmlStreamReader::StartDocument)
                continue;

            if (token == QXmlStreamReader::StartElement)
            {
                if (xmlReader.name() == "video") continue;
                if (xmlReader.name() == "videoName") continue;
                if (xmlReader.name() == "action") continue;

                if (xmlReader.name() == "bbox")
                {
                    BBox boundingBox;

                    QXmlStreamAttributes attributes = xmlReader.attributes();
                    if (!attributes.hasAttribute("x") || !attributes.hasAttribute("y") || !attributes.hasAttribute("width")
                            || !attributes.hasAttribute("height") || !attributes.hasAttribute("framenr") )
                    {
                        std::cout << "Missing attributes in bbox!" << std::endl;
                        continue;
                    }

                    boundingBox["x"] = attributes.value("x").toString().toInt();
                    boundingBox["y"] = attributes.value("y").toString().toInt();
                    boundingBox["width"] = attributes.value("width").toString().toInt();
                    boundingBox["height"] = attributes.value("height").toString().toInt();
                    boundingBox["framenr"] = attributes.value("framenr").toString().toInt();

                    boundingBoxes.append(boundingBox);
                }
            }

        }
        std::cout << "All bounding boxes completed for " << fileName.toStdString() << ", obtained boxes: " << boundingBoxes.count() << std::endl;

        // write to text in a new format
        QFileInfo fileInfo(*xmlFile);
        QFile* txtFile = new QFile(workingDirectory.absoluteFilePath(fileInfo.baseName() + ".track"));
        if (txtFile->open(QFile::WriteOnly))
        {
            QTextStream txtStream(txtFile);

            //<frame-number> <top-left-x-position> <top-left-y-position> <width> <height>
            foreach( BBox boundingBox, boundingBoxes)
                txtStream << boundingBox["framenr"] << " "
                          << boundingBox["x"] << " "
                          << boundingBox["y"] << " "
                          << boundingBox["width"] << " "
                          << boundingBox["height"] << "\n";

            std::cout << "Text file written: " << workingDirectory.absoluteFilePath(txtFile->fileName()).toStdString() << std::endl;
            txtStream.flush();
            txtFile->close();

        }

        delete xmlFile;
        delete txtFile;
    }

    return EXIT_SUCCESS;
}
