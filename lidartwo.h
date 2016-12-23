#ifndef PCAPTHREAD1_H
#define PCAPTHREAD1_H

#include "common.h"
#include "savebuffer.h"

const int cycle_num_II = 250; // number of UDP packets per 360 deg frame
const int delay_ms_II = 60; // delay (in ms) between two successive frames in offline mode

const int num_frame_buffer_II = 200; // number of 360 deg frames per buffer

PointCloudTPtr extract_xyz_II(struct data_packet_II& processed_packet, PointCloudTPtr cloud);

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
class LidarTwo : public QThread
{
    Q_OBJECT
public:
    LidarTwo(QObject *parent = 0);
    void run();
    QString fileName;
    bool offline;
    bool pause;
    QString portName;
    bool startBuffer;
    bool enableBuffer;
    bool stop;
    int curFrame;   //to track frame number
    int frameNumber;

signals:
    void updateCloud(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
    void bufferEmptiedII();
private:
    void bufferBuilder(const u_char *packet);
    QQueue<QString> bufferLidarTwo;
};

#endif // PCAPTHREAD1_H
