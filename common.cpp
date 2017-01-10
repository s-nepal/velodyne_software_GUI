#include "common.h"

// ---------------------------------------------------------------------------
//  void packetHandler_I(u_char *userData, const struct pcap_pkthdr* pkthdr, const u_char* packet)
//
//  Summary:    pcap_loop() will invoke this function by sending packet header and packet data
//              This function will populate the structure p with packet header and packet data.
//
//  Parameters:
//
//      userData    this contains the pointer to get_packet data structure sent to pcap_loop()
//      pkthdr      pointer to packet header extracted by pcap library.
//      packet      poiter to packet data extracted by pcap library
// ---------------------------------------------------------------------------
void packetHandler_live(u_char *userData, const struct pcap_pkthdr* pkthdr, const u_char* packet)
{
    struct get_packet *p = (struct get_packet *) userData;
    p->pkthdr = pkthdr;
    p->packet = packet;
    return;

}


// ---------------------------------------------------------------------------
//  void colorize_point_cloud(double curr_intensity, pcl::PointXYZRGBA *sample)
//
//  Summary:
//
//  Parameters:
// ---------------------------------------------------------------------------
void colorize_point_cloud(double curr_intensity, pcl::PointXYZRGBA *sample)
{
    double intensity_range = 63; //any intensity value above this value will be red
    double wavelength;
    double min_wavelength = 470; // used to discard overtly blue and purple points that are invisible due to the black background

    if(curr_intensity <= intensity_range)
        wavelength = curr_intensity / intensity_range * (780 - min_wavelength) + min_wavelength;
    else
        wavelength = 780;

    if((wavelength >= 380) && (wavelength < 440)){
        sample->r = (-(wavelength - 440) / (440 - 380))*255;
        sample->g = 0;
        sample->b = 255;

    }else if((wavelength >= 440) && (wavelength<490)){
        sample->r = 0;
        sample->g = ((wavelength - 440) / (490 - 440))*255;
        sample->b = 255;

    }else if((wavelength >= 490) && (wavelength<510)){
        sample->r = 0;
        sample->g = 255;
        sample->b = (-(wavelength - 510) / (510 - 490))*255;

    }else if((wavelength >= 510) && (wavelength<580)){
        sample->r = ((wavelength - 510) / (580 - 510))*255;
        sample->g = 255;
        sample->b = 0;

    }else if((wavelength >= 580) && (wavelength<645)){
        sample->r = 255;
        sample->g = (-(wavelength - 645) / (645 - 580))*255;
        sample->b = 0;

    }else if((wavelength >= 645) && (wavelength < 781)){
        sample->r = 255;
        sample->g = 0;
        sample->b = 0;
    }else{
        sample->r = 0;
        sample->g = 0;
        sample->b = 0;
    }

    sample->a = 255;
}


// ---------------------------------------------------------------------------
//  void write_to_csv(PointCloudTPtr cloud, int curr_frame, int lidar_id)
//
//  Summary:
//
//  Parameters:
// ---------------------------------------------------------------------------
void write_to_csv(PointCloudTPtr cloud, int curr_frame, int lidar_id)
{

    ofstream myfile;
    std::string file_name;
    file_name = std::to_string(curr_frame);

    if(lidar_id == 0)
        file_name = "Lidar_1_Frame_" + file_name + ".csv";
    else
        file_name = "Lidar_2_Frame_" + file_name + ".csv";

    myfile.open (file_name);

    for(int i = 0; i < cloud -> size(); i++){
        double x = (double)cloud->at(i).x;
        double y = (double)cloud->at(i).y;
        double z = (double)cloud->at(i).z;
        double r = (double)cloud->at(i).r;
        double g = (double)cloud->at(i).g;
        double b = (double)cloud->at(i).b;

        if(x != 0 && y !=0 && z != 0){
            myfile << x << ",";
            myfile << y << ",";
            myfile << z << ",";
            myfile << r << ",";
            myfile << g << ",";
            myfile << b << ",";
            myfile << endl;
        }
    }

    myfile.close();

}

