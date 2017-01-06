#include "lidarone.h"
#include "common.h"
#include <QThread>
#include <QMutex>
#include <QDebug>
#include <QByteArray>

pcap_t *descr;
double prev_azimuth_I = 999; // dummy value at the start
int global_ctr_I = 0;		//to print out the packet number
int show_cloud_flag_I = 0; // this flag is set to 1 when the cloud buffer is filled with one full frame of data

// ---------------------------------------------------------------------------
//  LidarOne Constructor
// ---------------------------------------------------------------------------
LidarOne::LidarOne(QObject *parent): QThread(parent)
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
//              from QThread. This function extracts the packet from
//              ethernet port or filename using pcap library, processes it by calling
//              data_structure_builder_I, extract_xyz_I and finally emits the signal to
//              update the PCL window in main thread.
// ---------------------------------------------------------------------------
void LidarOne::run()
{
    //code for pcap
    char errbuf[PCAP_ERRBUF_SIZE];
    if(offline){
        descr = pcap_open_offline(fileName.toStdString().c_str(), errbuf);
    }
    else{

        descr = pcap_open_live(portName.toStdString().c_str(), num_bytes_I, 1, 1, errbuf);
    }

    if (descr == NULL) {
      cout << "pcap_open_live() failed: " << errbuf << endl;
      return;
    }
    struct get_packet *p = new struct get_packet;
    struct data_packet processed_packet;
    PointCloudTPtr cloud (new PointCloudT);
    while(!stop)
    {
        while(pause){
            if(stop)
                break;
        }
        pcap_loop(descr, 1, packetHandler_I, (u_char *) p);
        if(!offline & enableBuffer)     //disable buffer untill it is enabled by user and in offline mode
            bufferBuilder(p->packet);
        data_structure_builder_I(p->pkthdr, p->packet, processed_packet);
        cloud = extract_xyz_I(processed_packet, cloud);

        // emit the cloud to the screen only if cloud has one full 360 deg frame
        if(show_cloud_flag_I == 1)
        {
            emit updateCloud(NumberUpdate, cloud);
            NumberUpdate++;

            if(offline){
                this->msleep(delay_ms_I); //different delay for offline mode
            }
            else{
                this->msleep(50);
            }
            show_cloud_flag_I = 0;
            if((frameNumber > 0) || (frameNumber < 0)){
                write_to_csv(cloud, curFrame, 0); // lidar id = 0 (3rd argument)
                frameNumber--;
            }
            cloud -> points.clear();
            curFrame++;
        }

    }
    //to clear PCL window
    cloud->clear();
    emit updateCloud(1, cloud);

    return;
}

// ---------------------------------------------------------------------------
//  PointCloudTPtr extract_xyz_I(struct data_packet& processed_packet, PointCloudTPtr cloud)
//
//  Summary:    generates x, y, z co-ordinates of the point using elevation angle and azimuth values.
//              Also adds color to each point based on the intensity value.
// ---------------------------------------------------------------------------
PointCloudTPtr extract_xyz_I(struct data_packet& processed_packet, PointCloudTPtr cloud)
{
    pcl::PointXYZRGBA sample;
    double curr_azimuth, curr_dist, curr_intensity, curr_elev_angle;
    for(int i = 0; i < 12; i++){
        curr_azimuth = (processed_packet.payload[i].azimuth) * PI / 180; //convert degrees to radians

        for(int j = 0; j < 32; j++){
            curr_dist = processed_packet.payload[i].dist[j];
            curr_intensity = processed_packet.payload[i].intensity[j];
            curr_elev_angle = (elev_angles[j]) * PI / 180;
            sample.x = curr_dist * cos(curr_elev_angle) * sin(curr_azimuth);
            sample.y = curr_dist * cos(curr_elev_angle) * cos(curr_azimuth);
            sample.z = curr_dist * sin(curr_elev_angle);
            //call function to colorize the point cloud
            colorize_point_cloud(curr_intensity, &sample);
            cloud -> points.push_back(sample);
        }

        // This if conditional checks whether the shift from 360 deg to 0 deg has happened, signifying one full frame
        if(prev_azimuth_I > curr_azimuth && prev_azimuth_I <= 2*PI && curr_azimuth != 0){
            if(prev_azimuth_I > 0.8*2*PI && curr_azimuth < 0.2*2*PI){
                show_cloud_flag_I = 1;
//                cout << "Previous azimuth: " << prev_azimuth_I / PI * 180 << endl;
//                cout << "Current azimuth: " << curr_azimuth / PI * 180 << endl;
            }
        }

        prev_azimuth_I = curr_azimuth;
    }

    return cloud;
}

// ---------------------------------------------------------------------------
//  void LidarOne::bufferBuilder(const u_char *packet)
//
//  Summary:    bufferBuilder is used only when bufferCheckBox is enabled. This will save
//              each packet extracted by pcap_loop into a Queue datastructure (bufferLidarOne).
//              If buffer Button is clicked, data save in Queue is poped into a virtual ethernet port,
//              which is collected in another thread (recordThread) and saved into a file.
//              Tins library is used for trasmitting the data from bufferLidarOne to virtual ethernet port.
//              After buffer is emptied, a signal is emitted to acknowledge main thread.
//
//  Parameters:
//
//      packet  pointer to packet received from ethernet port or file
// ---------------------------------------------------------------------------
void LidarOne::bufferBuilder(const u_char *packet)
{
    int ctr = 42;

    // If the first byte of the payload is not 0xFF then do not proceed
    if(packet[42] != 0xFF || packet[43] != 0xEE){
        return;
    }

    // Convert the 1206 bytes into a single string consisting of 1206 characters
    QString pkt_buffer;

    for(int i = 0; i < 1206; i++){
        pkt_buffer.append(packet[42+i]);
    }

    //build the buffer
    bufferLidarOne.enqueue(pkt_buffer);

    //once the buffer is full, remove the first packet
    if(bufferLidarOne.size() > num_frame_buffer_I * cycle_num_I){
        bufferLidarOne.dequeue();

        if(startBuffer){    //start saving buffer data into a file.
            // definitions needed for UDP packet composition and transmission
            static Tins::PacketSender sender;
            static Tins::NetworkInterface iface("eth20");

            SaveBuffer recordThread("eth20", fileName.toStdString().c_str(), num_bytes_I);
            QThread *thread = new QThread;
            recordThread.moveToThread(thread);
            recordThread.connect(this, SIGNAL(bufferEmptied()), SLOT(stopSaveBuffer()), Qt::QueuedConnection);
            recordThread.connect(thread, SIGNAL(finished()), SLOT(deleteLater()));
            connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
            thread->start();
            QMetaObject::invokeMethod(&recordThread, "startRecording");

            while(!bufferLidarOne.isEmpty()){
                Tins::EthernetII pkt = Tins::EthernetII() / Tins::IP("255.255.255.255") / Tins::UDP() /
                        Tins::RawPDU((bufferLidarOne.head()).toStdString());
                sender.send(pkt, iface);
                bufferLidarOne.dequeue();
                this->msleep(1);
            }

            emit bufferEmptied();
            //thread->quit();
            thread->wait(250);
            startBuffer = false;

        }
    }
}



