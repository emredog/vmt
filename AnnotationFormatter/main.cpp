#include <QCoreApplication>
#include <iostream>
#include <QDir>
#include <QXmlStreamReader>
#include <QTextStream>

typedef QMap<QString, int> BBox;

bool operator<(const BBox &bbox1, const BBox &bbox2)
{
    //if different action numbers, than compare with action numbers
    if (bbox1["nr"] != bbox2["nr"]) return bbox1["nr"] < bbox2["nr"];
    //else (action number is same), compare with frame numbers
    return bbox1["framenr"] < bbox2["framenr"];

}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QStringList classNames;
    classNames.append("discussion");
    classNames.append("give");
    classNames.append("box-desk");
    classNames.append("enter-leave");
    classNames.append("try-to-enter");
    classNames.append("unlock-enter");
    classNames.append("baggage");
    classNames.append("handshaking");
    classNames.append("typing");
    classNames.append("telephone");


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
        int classIndex = -1, actionNumber = -1;

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
                if (xmlReader.name() == "action")
                {
                    classIndex = -1, actionNumber = -1;

                    QXmlStreamAttributes attributes = xmlReader.attributes();
                    if (!attributes.hasAttribute("nr") || !attributes.hasAttribute("class"))
                    {
                        std::cout << "Missing attributes in <action> !" << std::endl;
                        continue;
                    }

                    bool ok1, ok2;
                    classIndex = attributes.value("class").toString().toInt(&ok1);
                    actionNumber = attributes.value("nr").toString().toInt(&ok2);
                    if (!ok1 || !ok2)
                    {
                        std::cout << "Read error on <action> 'class' or 'nr' !!!" << std::endl;
                        break;
                    }


                }

                if (xmlReader.name() == "bbox")
                {
                    BBox boundingBox;

                    QXmlStreamAttributes attributes = xmlReader.attributes();
                    if (!attributes.hasAttribute("x") || !attributes.hasAttribute("y") || !attributes.hasAttribute("width")
                            || !attributes.hasAttribute("height") || !attributes.hasAttribute("framenr") )
                    {
                        std::cout << "Missing attributes in <bbox> !" << std::endl;
                        continue;
                    }
                    //from <bbox ... >
                    boundingBox["x"] = attributes.value("x").toString().toInt();
                    boundingBox["y"] = attributes.value("y").toString().toInt();
                    boundingBox["width"] = attributes.value("width").toString().toInt();
                    boundingBox["height"] = attributes.value("height").toString().toInt();
                    boundingBox["framenr"] = attributes.value("framenr").toString().toInt();
                    //from <action nr="..." class="...">
                    boundingBox["nr"] = actionNumber;
                    boundingBox["class"] = classIndex;

                    boundingBoxes.append(boundingBox);
                }
            }

        }
        std::cout << "All bounding boxes completed for " << fileName.toStdString() << ", obtained boxes: " << boundingBoxes.count() << std::endl;

        if (boundingBoxes.isEmpty()) //if there's no bounding boxes, continue with next file
            continue;

        //sort by class number
        qSort(boundingBoxes);
        std::cout << "Bounding boxes are sorted by 1) Action Numbers 2) Frame Numbers" << std::endl;

        // write to text in a new format
        QFileInfo fileInfo(*xmlFile);
        int classOfCurrentBox = boundingBoxes.first()["class"];
        int numberOfCurrentBox = boundingBoxes.first()["nr"];
        QFile* txtFile = new QFile(workingDirectory.absoluteFilePath(fileInfo.baseName() + "_" + QString::number(numberOfCurrentBox)
                                                                     + "_" + classNames.at(classOfCurrentBox-1) + ".track"));
        if (txtFile->open(QFile::WriteOnly))
        {
            QTextStream txtStream(txtFile);

            //<frame-number> <top-left-x-position> <top-left-y-position> <width> <height>
            foreach( BBox boundingBox, boundingBoxes)
            {
                if (numberOfCurrentBox != boundingBox["nr"]) //change of action! need different text file
                {
                    std::cout << "Text file written: " << workingDirectory.absoluteFilePath(txtFile->fileName()).toStdString() << std::endl;
                    txtStream.flush();
                    txtFile->close();
                    delete txtFile;
                    classOfCurrentBox = boundingBox["class"];
                    numberOfCurrentBox = boundingBox["nr"];
                    txtFile = new QFile(workingDirectory.absoluteFilePath(fileInfo.baseName() + "_" + QString::number(numberOfCurrentBox)
                                                                         + "_" + classNames.at(classOfCurrentBox-1) + ".track"));
                    txtFile->open(QFile::WriteOnly);
                    txtStream.reset();
                    txtStream.setDevice(txtFile);
                }
                txtStream << boundingBox["framenr"] << " "
                          << boundingBox["x"] << " "
                          << boundingBox["y"] << " "
                          << boundingBox["width"] << " "
                          << boundingBox["height"] << "\n";
            }

            std::cout << "Text file written: " << workingDirectory.absoluteFilePath(txtFile->fileName()).toStdString() << std::endl;
            txtStream.flush();
            txtFile->close();

        }

        delete xmlFile;
        delete txtFile;
    }

    return EXIT_SUCCESS;
}
