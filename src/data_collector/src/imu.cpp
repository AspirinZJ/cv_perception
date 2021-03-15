#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <jsoncpp/json/json.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace std;

struct stat st = {0};
string dump_root_path, imu_path;

class ExtIMUDumpper
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ofstream imuFile_Handler;

public:
  ExtIMUDumpper()
  {
    // Subscrive to input video feed and publish output video feed
    sub_ = nh_.subscribe("/IMU_data", 50, &ExtIMUDumpper::extIMUCb, this);
    string file_path = imu_path + "ext.txt";
    imuFile_Handler.open(file_path.c_str());
    imuFile_Handler << "Data format:" << endl;
    imuFile_Handler << "timestamp orientation_x orientation_y orientation_z orientation_w ";
    imuFile_Handler << "angular_velocity_x angular_velocity_y angular_velocity_z ";
    imuFile_Handler << "linear_acceleration.x linear_acceleration.y linear_acceleration.z" << endl << endl;

    cout << "Exteral imu dumpper start." << endl;
  }

  ~ExtIMUDumpper()
  {
    imuFile_Handler.close();
  }

  void extIMUCb(const sensor_msgs::Imu::ConstPtr& msg)
  {
    /*
    cout << msg->header.stamp << " ";
    cout << msg->orientation.x << " "<< msg->orientation.y << " "<< msg->orientation.z << " "<< msg->orientation.w << " ";
    cout << msg->angular_velocity.x << " " << msg->angular_velocity.x << " " << msg->angular_velocity.x << " ";
    cout << msg->linear_acceleration.x << " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z;
    cout << endl;
    */

    imuFile_Handler << msg->header.stamp << " ";
    imuFile_Handler << msg->orientation.x << " "<< msg->orientation.y << " "
        << msg->orientation.z << " "<< msg->orientation.w << " ";
    imuFile_Handler << msg->angular_velocity.x << " " << msg->angular_velocity.x 
        << " " << msg->angular_velocity.x << " ";
    imuFile_Handler << msg->linear_acceleration.x << " " << msg->linear_acceleration.y 
        << " " << msg->linear_acceleration.z;
    imuFile_Handler << endl;
  }
};


class IntIMUDumpper
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ofstream imuFile_Handler;

public:
  IntIMUDumpper()
  {
    // Subscrive to input video feed and publish output video feed
    sub_ = nh_.subscribe("/mobile_base/sensors/imu_data", 50, &IntIMUDumpper::intIMUCb, this);
    string file_path = imu_path + "int.txt";
    imuFile_Handler.open(file_path.c_str());
    imuFile_Handler << "Data format:" << endl;
    imuFile_Handler << "timestamp orientation_x orientation_y orientation_z orientation_w ";
    imuFile_Handler << "angular_velocity_x angular_velocity_y angular_velocity_z ";
    imuFile_Handler << "linear_acceleration.x linear_acceleration.y linear_acceleration.z" << endl << endl;

    cout << "Internal imu dumpper start." << endl;
  }

  ~IntIMUDumpper()
  {
    imuFile_Handler.close();
  }

  void intIMUCb(const sensor_msgs::Imu::ConstPtr& msg)
  {
    /*
    cout << msg->header.stamp << " ";
    cout << msg->orientation.x << " "<< msg->orientation.y << " "<< msg->orientation.z << " "<< msg->orientation.w << " ";
    cout << msg->angular_velocity.x << " " << msg->angular_velocity.x << " " << msg->angular_velocity.x << " ";
    cout << msg->linear_acceleration.x << " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z;
    cout << endl;
    */
   
    imuFile_Handler << msg->header.stamp << " ";
    imuFile_Handler << msg->orientation.x << " "<< msg->orientation.y << " "
        << msg->orientation.z << " "<< msg->orientation.w << " ";
    imuFile_Handler << msg->angular_velocity.x << " " << msg->angular_velocity.x 
        << " " << msg->angular_velocity.x << " ";
    imuFile_Handler << msg->linear_acceleration.x << " " << msg->linear_acceleration.y 
        << " " << msg->linear_acceleration.z;
    imuFile_Handler << endl;
  }
};

class GpsIMUDumpper
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ofstream imuFile_Handler;

public:
  GpsIMUDumpper()
  {
    // Subscrive to input video feed and publish output video feed
    sub_ = nh_.subscribe("/ublox_gps/navheading", 50, &GpsIMUDumpper::gpsIMUCb, this);

    string file_path = imu_path + "gps_imu.txt";

    imuFile_Handler.open(file_path.c_str());
    imuFile_Handler << "Data format:" << endl;
    imuFile_Handler << "timestamp orientation_x orientation_y orientation_z orientation_w ";
    imuFile_Handler << "angular_velocity_x angular_velocity_y angular_velocity_z ";
    imuFile_Handler << "linear_acceleration.x linear_acceleration.y linear_acceleration.z" << endl << endl;

    cout << "GPS imu dumpper start." << endl;
  }

  ~GpsIMUDumpper()
  {
    imuFile_Handler.close();
  }

  void gpsIMUCb(const sensor_msgs::Imu::ConstPtr& msg)
  {
    /*
    cout << msg->header.stamp << " ";
    cout << msg->orientation.x << " "<< msg->orientation.y << " "<< msg->orientation.z << " "<< msg->orientation.w << " ";
    cout << msg->angular_velocity.x << " " << msg->angular_velocity.x << " " << msg->angular_velocity.x << " ";
    cout << msg->linear_acceleration.x << " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z;
    cout << endl;
    */
   
    imuFile_Handler << msg->header.stamp << " ";
    imuFile_Handler << msg->orientation.x << " "<< msg->orientation.y << " "
        << msg->orientation.z << " "<< msg->orientation.w << " ";
    imuFile_Handler << msg->angular_velocity.x << " " << msg->angular_velocity.x 
        << " " << msg->angular_velocity.x << " ";
    imuFile_Handler << msg->linear_acceleration.x << " " << msg->linear_acceleration.y 
        << " " << msg->linear_acceleration.z;
    imuFile_Handler << endl;
  }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_dumper");
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
    imu_path = dump_root_path+ "/imu/";
    bool ifIMU = genaral_root["imu_dump"].asBool();

    ExtIMUDumpper *ext_imu;
    IntIMUDumpper *int_imu;
    GpsIMUDumpper *gps_imu;

    if(ifIMU)
    {
        if (stat(dump_root_path.c_str(), &st) == -1) 
        {
            cout << "Creating log folder: " << dump_root_path << endl;
            mkdir(dump_root_path.c_str(), 0700);
        }

        if (stat(imu_path.c_str(), &st) == -1) 
        {
            cout << "Creating imu log folder: " << imu_path << endl;
            mkdir(imu_path.c_str(), 0700);
        }

        cout << "Start dumping IMU data at: " << imu_path << endl;

        ext_imu = new ExtIMUDumpper;
        int_imu = new IntIMUDumpper;
        gps_imu = new GpsIMUDumpper;
    }

    if(ifIMU)
    {
        ros::spin();
    }
    else
    {
      cout << "IMU dumpper has no job to do. Quit..." << endl;
    }

    return 0;
}