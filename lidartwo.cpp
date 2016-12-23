#include "lidartwo.h"
#include "common.h"
#include <QThread>
#include <QMutex>
#include <QDebug>
#include<QByteArray>

pcap_t *descr1;
int global_ctr_II = 0;

// ---------------------------------------------------------------------------
//  LidarTwo constructor
// ---------------------------------------------------------------------------

LidarTwo::LidarTwo(QObject *parent): QThread(parent)
{
    startBuffer = false;
    enableBuffer = false;
    stop = false;
    curFrame = 0; frameNumber = 0;
}

// ---------------------------------------------------------------------------
//  void LidarOne::run()
//
//  Summary:    This funtion is called by start(). This is overloaded function
//              from QThread(). This function extracts the packet from
//              ethernet port or filename using pcap library, processes it by calling
//              data_structure_builder_II, extract_xyz_II and finally emits the signal to
//              update the PCL window in main thread.
// ---------------------------------------------------------------------------
void LidarTwo::run()
{
    //code for pcap
    char errbuf[PCAP_ERRBUF_SIZE];
    if(offline){
        descr1 = pcap_open_offline(fileName.toStdString().c_str(), errbuf);
    }
    else{
        descr1 = pcap_open_live(portName.toStdString().c_str(), num_bytes_II, 1, 1, errbuf);
    }

    if (descr1 == NULL) {
      cout << "pcap_open_live() failed: " << errbuf << endl;
      return;
    }
    struct get_packet *p = new struct get_packet;
    struct data_packet_II processed_packet;
    PointCloudTPtr cloud (new PointCloudT);
    while(!stop)
    {
        while(pause){
            if(stop)
                break;
        }
        pcap_loop(descr1, 1, packetHandler_I, (u_char *) p);
        if(!offline & enableBuffer)
            bufferBuilder(p->packet);
        data_structure_builder_II(p->pkthdr, p->packet, processed_packet);
        cloud = extract_xyz_II(processed_packet, cloud);

        // if the cloud buffer is full
        if(global_ctr_II == cycle_num_II)
        {
            emit updateCloud(1, cloud);
            if(offline){
                this->msleep(delay_ms_II);
            }
            else{
                this->msleep(50);
            }
            if((frameNumber > 0) || (frameNumber < 0)){
                write_to_csv(cloud, curFrame, 1); // lidar id = 1 (3rd argument)
                frameNumber--;
            }
            cloud->points.clear();
            global_ctr_II = 0;
            curFrame++;
        }
    }
    //clear PCL window
    cloud->clear();
    emit updateCloud(1, cloud);

    return;
}

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
PointCloudTPtr extract_xyz_II(struct data_packet_II& processed_packet, PointCloudTPtr cloud)
{
    pcl::PointXYZRGBA sample;

    double curr_azimuth, curr_elev_angle, curr_dist, curr_intensity;

    // i starts at 2 instead of 0 because i = 0 & i = 1 correspond to 0xFFEE
    for(int i = 2; i < 1456; i = i + 10){
        curr_azimuth = double((processed_packet.payload[i] << 8) | processed_packet.payload[i + 1]);
        curr_elev_angle = double((processed_packet.payload[i + 2] << 8) | processed_packet.payload[i + 3]);
        curr_dist = double((processed_packet.payload[i + 4] << 8) | processed_packet.payload[i + 5]);
        curr_intensity = double((processed_packet.payload[i + 6] << 8) | processed_packet.payload[i + 7]);

        /*cout << "Distance: " << curr_dist / 100 << endl; // remove this
        cout << "Azimuth: " << curr_azimuth / 100 << endl; // remove this
        cout << "Elevation: " << curr_elev_angle / 100 << endl; // remove this
        cout << "--------------" << endl;*/

        curr_azimuth = (curr_azimuth / 100) * PI / 180;
        // to be changed back to curr_elev_angle = (curr_elev_angle / 100 - 15) * PI / 180 only for sample velodyne data because of the 15 deg offset
        curr_elev_angle = (curr_elev_angle / 100 - 15) * PI / 180;

        curr_dist = curr_dist / 500; // was 500 in the old version

        sample.x = curr_dist * cos(curr_elev_angle) * sin(curr_azimuth);
        sample.y = curr_dist * cos(curr_elev_angle) * cos(curr_azimuth);
        sample.z = curr_dist * sin(curr_elev_angle);

        //call function to colorize the point cloud
        colorize_point_cloud(curr_intensity, &sample);
        cloud -> points.push_back(sample);
    }

    global_ctr_II++;

    return cloud;
}

// ---------------------------------------------------------------------------
//  void LidarOne::bufferBuilder(const u_char *packet)
//
//  Summary:    bufferBuilder is used only when bufferCheckBox is enabled. This will save
//              each packet extracted by pcap_loop into a Queue datastructure (bufferLidarTwo).
//              If buffer Button is clicked, data save in Queue is poped into a virtual ethernet port,
//              which is collected in another thread (recordThread) and saved into a file.
//              Tins library is used for trasmitting the data from bufferLidarTwo to virtual ethernet port.
//              After buffer is emptied, a signal is emitted to acknowledge main thread and recordThread2.
//
//  Parameters:
//
//      packet  pointer to packet received from ethernet port or file
// ----------------------------------------------------------------------------
void LidarTwo::bufferBuilder(const u_char *packet)
{
    int ctr = 42;

    // If the first byte of the payload is not 0xFF then do not proceed
    if(packet[42] != 0xFF || packet[43] != 0xEE){
        return;
    }

    // Convert the 1206 bytes into a single string consisting of 1206 characters
    QString pkt_buffer;

    for(int i = 0; i < 1456; i++){
        pkt_buffer.append(packet[42+i]);
    }

    //build the buffer
    bufferLidarTwo.enqueue(pkt_buffer);

    //once the buffer is full, do the following
    if(bufferLidarTwo.size() > num_frame_buffer_II * cycle_num_II){
        bufferLidarTwo.dequeue();

        if(startBuffer){
            // definitions needed for UDP packet composition and transmission
            static Tins::PacketSender sender;
            static Tins::NetworkInterface iface("eth21");

            SaveBuffer recordThread2("eth21", fileName.toStdString().c_str(), num_bytes_II);
            QThread *thread2 = new QThread;
            recordThread2.moveToThread(thread2);
            recordThread2.connect(this, SIGNAL(bufferEmptiedII()), SLOT(stopSaveBuffer()), Qt::QueuedConnection);
            recordThread2.connect(thread2, SIGNAL(finished()), SLOT(deleteLater()));
            connect(thread2, SIGNAL(finished()), thread2, SLOT(deleteLater()));
            thread2->start();
            QMetaObject::invokeMethod(&recordThread2, "startRecording");

            while(!bufferLidarTwo.isEmpty()){
                Tins::EthernetII pkt = Tins::EthernetII() / Tins::IP("255.255.255.255") / Tins::UDP() /
                        Tins::RawPDU(bufferLidarTwo.head().toStdString());
                sender.send(pkt, iface);
                bufferLidarTwo.dequeue();
                this->msleep(1);
            }

            emit bufferEmptiedII();
            //thread2->quit();
            thread2->wait(250);
            startBuffer = false;

        }
    }
}