#ifndef SAVEBUFFER_H
#define SAVEBUFFER_H

#include <QObject>
#include <QDebug>
#include <QByteArray>
#include <QBasicTimer>
#include <QTimerEvent>
#include "common.h"

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
class SaveBuffer : public QObject
{
    Q_OBJECT
public:
    SaveBuffer(QObject *parent = 0);
    SaveBuffer(QString port, QString file, int pSize, QObject *parent = 0);
    pcap_t *descr;
    pcap_dumper_t *pd;
    QBasicTimer *timer;
    QString portName;
    QString fileName;
    int packetSize;

public slots:
    void stopSaveBuffer();
    void startRecording();
protected:
    void timerEvent(QTimerEvent *);
};

#endif // SAVEBUFFER_H
