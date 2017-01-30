#include "recordpcapdata.h"

RecordPcapData::RecordPcapData(QObject *parent) :
    QThread(parent)
{

}

// ---------------------------------------------------------------------------
//  RecordPcapData Constructor
// ---------------------------------------------------------------------------
RecordPcapData::RecordPcapData(int pSize, QObject *parent) : QThread(parent)
{
    packetSize = pSize;
}

// ---------------------------------------------------------------------------
//  void RecordPcapData::run()
//
//  Summary:    This function will save the data from the ethernet port and save
//              it into a file using pcap library
// ---------------------------------------------------------------------------
void RecordPcapData::run()
{
    char errbuf[PCAP_ERRBUF_SIZE];

    descr = pcap_open_live(portName.toStdString().c_str(), packetSize, 1, 1, errbuf);

    if(descr == NULL){
        qDebug() << "pcap_open_live failed: " << errbuf;
        return;
    }

    // use pcap_dump_open to create and open a .pcap file to store the data in
    if((pd = pcap_dump_open(descr, fileName.toStdString().c_str())) == NULL){
        qDebug() << "error in opening file" << fileName;
        return;
    }

    int pcount = 0;
    while(record){
        if((pcount = pcap_loop(descr, 1, &pcap_dump, (u_char *) pd)) < 0){
            cout << "error in reading packets";
            return;
        }
    }
    pcap_dump_close(pd);
    pcap_close(descr);
}
