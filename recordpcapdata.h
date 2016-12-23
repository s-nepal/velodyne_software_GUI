#ifndef RECORDPCAPDATA_H
#define RECORDPCAPDATA_H

#include <QObject>
#include <QThread>
#include <QDebug>
#include "common.h"

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
class RecordPcapData : public QThread
{
    Q_OBJECT
public:
    RecordPcapData(QObject *parent = 0);
    RecordPcapData(int pSize = 1248, QObject *parent = 0);
    void run();
    bool record;
    QString portName;
    QString fileName;
    pcap_t *descr;
    pcap_dumper_t *pd;
    int packetSize;         //to store the packet size received from lidar
                            //each lidar has different packet size and this
                            //variable is initialized during object instantiation
private:

};

#endif // RECORDPCAPDATA_H
