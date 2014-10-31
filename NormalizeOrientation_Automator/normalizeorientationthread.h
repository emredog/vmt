#ifndef NORMALIZEORIENTATIONTHREAD_H
#define NORMALIZEORIENTATIONTHREAD_H

#include <QThread>
#include <QPair>

typedef QPair<QString, QList<float> > PcdAnglesPair;

class NormalizeOrientationThread : public QThread
{
    Q_OBJECT
public:
    explicit NormalizeOrientationThread(const QList<PcdAnglesPair> &vmtsToAngles, int threadId, QObject *parent = 0);

signals:

public slots:

protected:
    void run();

    int threadId;
    QList<PcdAnglesPair> vmtsToAngles;
    QString program;

};

#endif // NORMALIZEORIENTATIONTHREAD_H
