#ifndef KLASERSCHMIDTHREAD_H
#define KLASERSCHMIDTHREAD_H

#include <QThread>
#include <QDir>
#include <QPair>

typedef QPair<QString, QString> VideoTrackPair;

class KlaserSchmidThread : public QThread
{
    Q_OBJECT
public:
    explicit KlaserSchmidThread(QList<VideoTrackPair> pairs, QStringList algoArgs,
                                QDir dataDir, QDir trackFileDir, QDir targetDir, QObject *parent = 0);

signals:

public slots:

protected:
    void run();

    QDir dataDir;
    QDir trackFileDir;
    QDir targetDir;
    QList<VideoTrackPair> pairs;
    QString program;
    QStringList algoArgs;

};

#endif // KLASERSCHMIDTHREAD_H
