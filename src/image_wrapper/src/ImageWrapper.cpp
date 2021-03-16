#include <iostream>
#include <utility>
#include <chrono>
#include <unistd.h>

#include "ImageWrapper.h"
#include "sin_cos_table.h"
#include "arc_length_table.h"

using namespace cv;
using namespace std;

ImageWrapper::ImageWrapper() {}

ImageWrapper::~ImageWrapper()
{
	for (int x = 0; x < unwarpImageCols; ++x)
	{
		delete[] *(this->indTable+x);
	}
	delete[] this->indTable;
}

bool ImageWrapper::initImageWrapper(unsigned cols, unsigned rows)
{
	if (theta_acc != nullptr)
		delete theta_acc;
	if (trho != nullptr)
		delete trho;

	orginalImageRows = cols;
	orginalImageCols = rows;
	int R = std::min(orginalImageRows, orginalImageCols) / 2;
	unwarpImageRows = R * ARC_SCALE;
	unwarpImageCols = 2 * PI * R * REAL_R;
	double d_tmp = 1000.0 / unwarpImageRows, tmp = unwarpImageRows * d_tmp;

	trho = new double[unwarpImageRows];
	for (int y = unwarpImageRows; y > 0; y--)
	{
		trho[unwarpImageRows - y] = unwarpImageRows *
									((tmp - (int) tmp) * (arc_len[(int) tmp + 1] - arc_len[(int) tmp]) +
									 arc_len[(int) tmp]);
		tmp -= d_tmp;
	}
	double theta, d_theta;
	theta = 0.0;
	d_theta = RMAX / R / REAL_R;
	u0 = orginalImageRows / 2.0;
	v0 = orginalImageCols / 2.0;

	theta_acc = new double[unwarpImageCols];
	for (int x = 0; x < unwarpImageCols; x++)
	{
		theta += d_theta;
		theta_acc[x] = theta;
	}

	u0 = orginalImageRows / 2.0;
	v0 = orginalImageCols / 2.0;

	this->indTable = new short *[unwarpImageCols];
	for (size_t x = 0; x < unwarpImageCols; ++x)
	{
		*(this->indTable + x) = new short[2 * unwarpImageRows];
		for (size_t y = 0; y < unwarpImageRows; ++y)
		{
			short xVal = trho[y] * tsin[(int) (theta_acc[x] + 0.5)] + v0;
			short yVal = trho[y] * tcos[(int) (theta_acc[x] + 0.5)] + u0;
			*(*(this->indTable + x) + 2 * y) = xVal;
			*(*(this->indTable + x) + 2 * y + 1) = yVal;
		}
	}

	return true;
}

cv::Mat ImageWrapper::getImage(cv::Mat image)
{
	auto tStart = std::chrono::steady_clock::now();
	srcImage = std::move(image);
	dstImage = cv::Mat(unwarpImageRows, unwarpImageCols, CV_8UC3, Scalar::all(0));



	// 9ms
	for (int x = 0; x < unwarpImageCols; ++x)
	{
		// #pragma omp parallel for
		for (int y = 0; y < unwarpImageRows; ++y)
		{
			dstImage.at<Vec3b>(y, x) = srcImage.at<Vec3b>(*(*(this->indTable + x) + 2 * y), *(*(this->indTable + x) + 2 * y + 1));
		}
	}

	auto tEnd = std::chrono::steady_clock::now();
	std::cout << "Duration: " << std::chrono::duration_cast<std::chrono::microseconds>(tEnd - tStart).count()
			  << std::endl;

	return dstImage;
}