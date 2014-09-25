#ifndef KLASERSCHMIDTHREADFROMVMT_H
#define KLASERSCHMIDTHREADFROMVMT_H

#include <QThread>
#include <QDir>


class KlaserSchmidThreadFromVMT : public QThread
{
    Q_OBJECT
public:
    explicit KlaserSchmidThreadFromVMT(QStringList vmtFiles, QDir vmtDir, QStringList algoArgs, QDir targetDir, int threadId, QObject *parent = 0);

signals:

public slots:

protected:
    void run();

    QStringList vmtFiles;
    QDir vmtDir;
    QStringList algoArgs;
    QDir targetDir;
    int threadId;
    QString program;



};

#endif // KLASERSCHMIDTHREADFROMVMT_H
