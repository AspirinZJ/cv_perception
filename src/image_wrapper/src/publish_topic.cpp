#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <jsoncpp/json/json.h>
#include <math.h>
#include <string>
#include <stdlib.h>

#include "ImageWrapper.h"

using namespace std;
using namespace cv;

ImageWrapper wrapper;
int center_x = 0;
int center_y = 0;
int dis = 0;
int scaleRatio = 0;
double top_cutoff = 0;
double bottom_cutoff = 0;
int recorded_width, recorded_height;
int calibrated_width, calibrated_height;
double recordRatio_x, recordRatio_y;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher pub = it_.advertise("usb_cam/wimage", 1);

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);

    cout << "Start publishing image data at /usb_cam/wimage" << endl;

    // // cv::namedWindow("Src");
    // cv::namedWindow("Dst");
  }

  ~ImageConverter()
  {
    // cv::destroyAllWindows();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    Mat resizedImage;
    resize(cv_ptr->image, resizedImage, Size((int)(cv_ptr->image.cols / scaleRatio), 
      (int)(cv_ptr->image.rows / scaleRatio)));

    // cv::Rect myROI(213, 97, 290, 290);
    // cout << resizedImage.cols << " " << resizedImage.rows << " " << center_x << " " << center_y << " " << dis << endl;

    cv::Rect myROI(center_x - dis/2, center_y - dis/2, dis, dis);;
    cv::Mat croppedImage = resizedImage(myROI);

    clock_t start_t = clock();
    cv::Mat res = wrapper.getImage(croppedImage);
    clock_t finish_t = clock();
    double fps = 1 / ((double)(finish_t - start_t) / CLOCKS_PER_SEC);
    // cout << "time: " << (double)(finish_t - start_t) / CLOCKS_PER_SEC << endl;

    cv::Rect myROI2(0, (int)(res.rows / top_cutoff), res.cols, (int)(res.rows / bottom_cutoff));
    // cout << res.cols << " " << res.rows << " " << top_cutoff << " " << bottom_cutoff << endl;
    cv::Mat final = res(myROI2);

    // Update GUI Window
    // cv::imshow("Src", croppedImage);
    // cv::imshow("Dst", final);
    // cv::waitKey(30);

    sensor_msgs::ImagePtr msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final).toImageMsg();
    msg_ptr->header.stamp = ros::Time::now();
    pub.publish(msg_ptr);
  }
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "image_converter");

    Json::Value genaral_root;

    std::string path = ros::package::getPath("lfi_launch");
    ifstream genaral_file(path + "/../../config/lawn_mover.json");
    genaral_file >> genaral_root;
    string camera_config_file = genaral_root["current_camera"].asString();
    recorded_width = genaral_root["camera_record_res"][0].asInt();
    recorded_height = genaral_root["camera_record_res"][1].asInt();
    scaleRatio = genaral_root["display_ratio"].asInt();

    cout << "Read configure successful! Using camera: " << camera_config_file << endl;

    ifstream camera_file(path + "/../../config/" + camera_config_file + ".json");
    Json::Value camera_root;
    camera_file >> camera_root;
    cout << "Loaded camera configuration: " << endl;

    // output camera info

    cout << "************************" << endl;
    cout << "Camera name: " << camera_root[0]["camera_name"] << endl;
    cout << "Camera vendor: " << camera_root[0]["vendor"] << endl;
    cout << "Camera manufacture date: " << camera_root[0]["manufacture_date"] << endl;
    cout << "Camera calibrated date: " << camera_root[0]["calibrated_date"] << endl;
    cout << "************************" << endl;
    
    calibrated_width = camera_root[1]["raw_resolution"][0].asInt();
    calibrated_height = camera_root[1]["raw_resolution"][1].asInt();
    recordRatio_x = calibrated_width / recorded_width;
    recordRatio_y = calibrated_height / recorded_height;

    cout << calibrated_width << " " << recorded_width << " " << recordRatio_x << endl;

    if(recordRatio_y != recordRatio_x)
    {
      cout << "Recorded image aspect ratio different with calibrated file! Quit..." << endl;
      return -1;
    }

    dis = (int)(camera_root[1]["distance"].asInt() * 2 / scaleRatio / recordRatio_x);
    center_x = (int)(camera_root[1]["center_x"].asInt() / scaleRatio / recordRatio_x);
    center_y = (int)(camera_root[1]["center_y"].asInt() / scaleRatio / recordRatio_y);

    top_cutoff = camera_root[1]["top_cutoff"].asDouble();
    bottom_cutoff = camera_root[1]["bottom_cutoff"].asDouble();

    bool res = wrapper.initImageWrapper(dis, dis); //290

    if (res)
    {
        cout << "Image wrapper init success!" << endl;
    }
    else
    {
        cout << "Image wrapper init error" << endl;
        return -1;
    }
    
    ImageConverter ic;
    ros::spin();
    return 0;
}
