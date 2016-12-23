#include "savebuffer.h"

SaveBuffer::SaveBuffer(QObject *parent) : QObject(parent)
{

}

// ---------------------------------------------------------------------------
//  SaveBuffer Constructor
// ---------------------------------------------------------------------------
SaveBuffer::SaveBuffer(QString port, QString file, int pSize, QObject *parent) : QObject(parent)
{
    portName = port;
    fileName = file;
    packetSize = pSize;
}

// ---------------------------------------------------------------------------
//  void SaveBuffer::startRecording()
//
//  Summary:    this method is invoked in bufferBuilder(). This will open a descriptor
//              and pcap_dumpter_t structure used for saving the data received from ethernet port.
//              A timer is started for obtaining the packet and saving.
// ---------------------------------------------------------------------------
void SaveBuffer::startRecording()
{
    char errbuf[PCAP_ERRBUF_SIZE];

    descr = pcap_open_live(portName.toStdString().c_str(), packetSize, 1, 1, errbuf);
    if(descr == NULL){
        qDebug() << "pcap_open_live failed: " << errbuf;
        return;
    }

    if((pd = pcap_dump_open(descr, fileName.toStdString().c_str())) == NULL){
        qDebug() << "error in opening file" ;
        return;
    }

    timer = new QBasicTimer;
    timer->start(0,this);
}

// ---------------------------------------------------------------------------
//  void SaveBuffer::timerEvent(QTimerEvent *event)
//
//  Summary:    This event is called when timerEvent is raised. It saves the data into a file
// ---------------------------------------------------------------------------
void SaveBuffer::timerEvent(QTimerEvent *)
{

    int pcount = 0;
    if((pcount = pcap_loop(descr, 1, &pcap_dump, (u_char *) pd)) < 0){
        cout << "error in reading packets";
        return;
    }

}

// ---------------------------------------------------------------------------
//  void SaveBuffer::stopSaveBuffer()
//
//  Summary:    When buffer is emptied, a signal is emmited in bufferBuilder.
//              On receiving this signal, this slot is invoked. This will stop timer
//              and close the descriptor and pcap_dumper_t.
// ---------------------------------------------------------------------------
void SaveBuffer::stopSaveBuffer()
{
    //qDebug() << "signal received I: exiting thread";
    timer->stop();
    pcap_dump_close(pd);
    pcap_close(descr);
}
