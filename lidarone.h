#ifndef PCAPTHREAD_H
#define PCAPTHREAD_H

#include "common.h"
#include "savebuffer.h"

const int cycle_num_I = 50;     // number of UDP packets per 360 deg frame
const int delay_ms_I = 54;      // delay (in ms) between two successive frames in offline mode

const int num_frame_buffer_I = 200;     // number of 360 deg frames per buffer



static int NumberUpdate = 0;

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
class LidarOne : public QThread
{
    Q_OBJECT
public:
    LidarOne(QObject *parent = 0);
    void run();

    QString fileName;
    bool offline;   //to keep track of mode selection
    bool pause;     //used to pause simulation in offline mode
    QString portName;
    bool startBuffer;
    bool enableBuffer;
    bool stop;      //to stop the infinite while loop
    int curFrame;   //keeps track of frame number
    int frameNumber;    //stores number of frames to store into a file

    pcap_t *descr;

    double prev_azimuth; // dummy value at the start
    int show_cloud_flag; // this flag is set to 1 when the cloud buffer is filled with one full frame of data

signals:
    void updateCloud(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
    void bufferEmptied();

private:
    void data_structure_builder(const struct pcap_pkthdr *pkthdr, const u_char *data, struct data_packet& processed_packet);
    PointCloudTPtr extract_xyz(struct data_packet& processed_packet, PointCloudTPtr cloud);
    void bufferBuilder(const u_char *packet);
    QQueue<QString> bufferLidarOne;

};

#endif // PCAPTHREAD_H
