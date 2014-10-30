#include "HelperFunctions.h"
#include <qdir.h>


HelperFunctions::HelperFunctions(void)
{
}


HelperFunctions::~HelperFunctions(void)
{
}

void HelperFunctions::getDir(string& d, vector<string> & f)
{
    QDir dir(QString::fromStdString(d));
    QStringList entryList = dir.entryList(QDir::Files);
    foreach(QString s, entryList) f.push_back(s.toStdString());
}
