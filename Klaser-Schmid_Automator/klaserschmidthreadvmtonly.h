#ifndef KLASERSCHMIDTHREADVMTONLY_H
#define KLASERSCHMIDTHREADVMTONLY_H

#include <QThread>
#include <QDir>


class KlaserSchmidThreadVmtOnly : public QThread
{
    Q_OBJECT
public:
    explicit KlaserSchmidThreadVmtOnly(QStringList vmtFiles, QDir vmtDir, QStringList algoArgs, QDir targetDir, int threadId, QObject *parent = 0);

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

#endif // KLASERSCHMIDTHREADVMTONLY_H
