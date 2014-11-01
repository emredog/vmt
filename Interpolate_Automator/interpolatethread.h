#ifndef INTERPOLATETHREAD_H
#define INTERPOLATETHREAD_H

#include <QThread>
#include <QDir>
#include <QStringList>

class InterpolateThread : public QThread
{
    Q_OBJECT
public:
    explicit InterpolateThread(const QDir &dataDir, const QStringList &vmtFiles, int maxSegmentLength, int threadId, QObject *parent = 0);

signals:

public slots:
protected:
    void run();

    QDir dataDir;
    QStringList vmtFiles;
    int threadId;
    QString program;
    int maxSegmentLength;

};

#endif // INTERPOLATETHREAD_H
