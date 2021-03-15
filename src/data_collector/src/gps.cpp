#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <jsoncpp/json/json.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace std;

struct stat st = {0};
string dump_root_path, gps_path;

class GpsDumpper
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ofstream gpsFile_Handler;

public:
  GpsDumpper()
  {
    // Subscrive to input video feed and publish output video feed
    sub_ = nh_.subscribe("/ublox_gps/fix", 50, &GpsDumpper::gpsCb, this);

    string file_path = gps_path + "gps.txt";

    gpsFile_Handler.open(file_path.c_str());
    gpsFile_Handler << "Data format:" << endl;
    gpsFile_Handler << "timestamp status service latitude longitude altitude" << endl << endl;

    cout << "GPS dumpper start." << endl;
  }

  ~GpsDumpper()
  {
    gpsFile_Handler.close();
  }

  void gpsCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    // cout << std::fixed;
    gpsFile_Handler << std::fixed;

    // cout << msg->header.stamp << " " << 0 << " " << msg->status.service << " ";
    // cout << msg->latitude << " " << msg->longitude << " " << msg-> altitude << endl;

    gpsFile_Handler << msg->header.stamp << " " << 0 << " "<< msg->status.service;
    gpsFile_Handler << " " << msg->latitude << " " << msg->longitude << " " 
        << msg-> altitude << endl;
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_dumper");
    ros::NodeHandle nh;
    string run_id;
    if (!nh.getParam("/run_id", run_id))
    {
        return -1;
    }  
    cout << "run id: " << run_id << endl;

    Json::Value genaral_root;
    std::string path = ros::package::getPath("lfi_launch");
    ifstream genaral_file(path + "/../../config/lawn_mover.json");
    genaral_file >> genaral_root;
    dump_root_path = genaral_root["dump_path"].asString() + run_id;
    gps_path = dump_root_path+ "/gps/";
    bool ifGPS = genaral_root["gps_dump"].asBool();

    GpsDumpper *dumpper;

    if(ifGPS)
    {
        if (stat(dump_root_path.c_str(), &st) == -1) 
        {
            cout << "Creating log folder: " << dump_root_path << endl;
            mkdir(dump_root_path.c_str(), 0700);
        }

        if (stat(gps_path.c_str(), &st) == -1) 
        {
            cout << "Creating imu log folder: " << gps_path << endl;
            mkdir(gps_path.c_str(), 0700);
        }

        cout << "Start dumping GPS data at: " << gps_path << endl;
        dumpper = new GpsDumpper;
    }

    if(ifGPS)
    {
        ros::spin();
    }
    else
    {
      cout << "GPS dumpper has no job to do. Quit..." << endl;
    }

    return 0;
}