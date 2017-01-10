#ifndef COMMON_H
#define COMMON_H

#include <QThread>
#include <QObject>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

//pcap header files
#include <pcap.h>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include <tins/tins.h>

#include <QString>
#include <QMutex>
#include <QByteArray>
#include <QQueue>

static QMutex mutex;

struct fire_data {
    uint16_t block_id;
    double azimuth;
    double dist[32];
    double intensity[32];
};

struct get_packet{
    const struct pcap_pkthdr *pkthdr;
    const uchar *packet;
};

struct data_packet {
    uint8_t header[42];
    fire_data payload[12];
    uint8_t footer[6];
};

struct data_packet_II{
    uint8_t header[42];
    uint8_t payload[1452];
    uint8_t footer[4];
};

#define PI 3.14159265

static const double elev_angles[32] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5,
                                11, -3, 13, -1, 15,-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5,
                                11, -3, 13, -1, 15};

const int num_bytes_I = 1248;   // number of bytes per UDP packet
const int num_bytes_II = 1498; // number of bytes per UDP packet

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudTPtr;

void packetHandler(u_char *userData, const struct pcap_pkthdr* pkthdr, const u_char* packet);
void data_structure_builder_I(const struct pcap_pkthdr *pkthdr, const u_char *data, struct data_packet& processed_packet);
void data_structure_builder_II(const struct pcap_pkthdr *pkthdr, const u_char *data, struct data_packet_II& processed_packet);
void colorize_point_cloud(double curr_intensity, pcl::PointXYZRGBA *sample);
void write_to_csv(PointCloudTPtr cloud, int curr_frame, int lidar_id);


#endif // COMMON_H
