#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <jsoncpp/json/json.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct stat st = {0};

int rawImageSaveWidth, rawImageSaveHeight;
int panoImageSaveWidth, panoImageSaveHeight;
string imgRaw_path;
string imgPano_path;

class RawImageDumper
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport raw_it_;
	image_transport::Subscriber raw_image_sub_;

public:
	RawImageDumper() : raw_it_(nh_)
	{
		// Subscribe to input video feed and publish output video feed
		raw_image_sub_ = raw_it_.subscribe("/usb_cam/image_raw", 1, &RawImageDumper::rawImageCb, this);
	}

	//	~RawImageDumper() { cv::destroyAllWindows(); }
	~RawImageDumper() = default;

	// callback function
	void rawImageCb(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try { cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); }
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		std::string ts;
		std::stringstream ss;
		ss << msg->header.stamp;
		ss >> ts;
		// cout << imgRaw_path + ts + ".jpg" << endl;

		Mat resizedImage;
		// cout << rawImageSaveWidth << " " << rawImageSaveHeight << endl;
		resize(cv_ptr->image, resizedImage, Size(rawImageSaveWidth, rawImageSaveHeight));

		// cv::Rect myROI(213, 97, 290, 290);
		// cout << resizedImage.cols << " " << resizedImage.rows << " " << center_x << " " << center_y << " " << dis << endl;

		cv::imwrite(imgRaw_path + ts + ".jpg", resizedImage);
	}
};


class PanoImageDumpper
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport pano_it_;
	image_transport::Subscriber pano_image_sub_;

public:
	PanoImageDumpper()
			: pano_it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		pano_image_sub_ = pano_it_.subscribe("/usb_cam/wimage", 1,
											 &PanoImageDumpper::panoImageCb, this);
	}

	~PanoImageDumpper()
	{
		// cv::destroyAllWindows();
	}

	void panoImageCb(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		std::string ts;
		std::stringstream ss;
		ss << msg->header.stamp;
		ss >> ts;
		// cout << imgPano_path + ts + ".jpg" << endl;

		Mat resizedImage;
		// cout << rawImageSaveWidth << " " << rawImageSaveHeight << endl;
		resize(cv_ptr->image, resizedImage, Size(panoImageSaveWidth, panoImageSaveHeight));

		// cv::Rect myROI(213, 97, 290, 290);
		// cout << resizedImage.cols << " " << resizedImage.rows << " " << center_x << " " << center_y << " " << dis << endl;

		cv::imwrite(imgPano_path + ts + ".jpg", resizedImage);
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_dumper");
	ros::NodeHandle nh;
	string run_id;
	if (!nh.getParam("/run_id", run_id))
	{
		std::cerr << "Error: cannot get run_id." << std::endl;
		return -1;
	}

	string dump_root_path;

	Json::Value general_root;
	std::string path = ros::package::getPath("lfi_launch");
	ifstream general_file(path + "/../../config/lawn_mover.json");
	general_file >> general_root;

	bool ifImgRaw, ifImgPano;
	ifImgRaw = general_root["camera_dump_raw"].asBool();
	ifImgPano = general_root["camera_dump_pano"].asBool();

	if (ifImgRaw || ifImgPano)
	{
		dump_root_path = general_root["dump_path"].asString();
		// dump_root_path += run_id;
		rawImageSaveWidth = general_root["camera_dump_raw_width"].asInt();
		rawImageSaveHeight = general_root["camera_dump_raw_height"].asInt();
		panoImageSaveWidth = general_root["camera_dump_pano_width"].asInt();
		panoImageSaveHeight = general_root["camera_dump_pano_height"].asInt();

		if (stat(dump_root_path.c_str(), &st) == -1) // 获取文件信息，不存在则创建新文件夹
		{
			cout << "Creating log folder: " << dump_root_path << endl;
			mkdir(dump_root_path.c_str(), 0700);
		}

		imgRaw_path = dump_root_path + "/img_raw/";
		if (ifImgRaw && stat(imgRaw_path.c_str(), &st) == -1)
		{
			cout << "Creating img_raw log folder: " << imgRaw_path << endl;
			mkdir(imgRaw_path.c_str(), 0700);
		}

		imgPano_path = dump_root_path + "/img_pano/";
		if (ifImgPano && stat(imgPano_path.c_str(), &st) == -1)
		{
			cout << "Creating img_pano log folder: " << imgPano_path << endl;
			mkdir(imgPano_path.c_str(), 0700);
		}
	}

	RawImageDumper *r_dumpper;
	PanoImageDumpper *p_dumpper;

	if (ifImgRaw)
	{
		cout << "Start recording raw image..." << endl;
		r_dumpper = new RawImageDumper;
	}

	if (ifImgPano)
	{
		cout << "Start recording pano image..." << endl;
		p_dumpper = new PanoImageDumpper;
	}

	if (ifImgRaw || ifImgPano)
	{
		ros::spin();
	}
	else
	{
		cout << "Image dumpper has no job to do. Quit..." << endl;
	}
	return 0;
}