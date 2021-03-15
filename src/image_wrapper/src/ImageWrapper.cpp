#include <iostream>

#include "ImageWrapper.h"
#include "sin_cos_table.h"
#include "arc_length_table.h"

using namespace cv;
using namespace std;

ImageWrapper::ImageWrapper()
{
}

ImageWrapper::~ImageWrapper()
{
}

bool ImageWrapper::initImageWrapper(unsigned cols, unsigned rows)
{
    if(theta_acc != NULL)
        delete theta_acc;
    if(trho != NULL)
        delete trho;

    orginalImageRows = cols;
    orginalImageCols = rows;
    int R = std::min(orginalImageRows, orginalImageCols) / 2;
    unwarpImageRows = R * ARC_SCALE;
    unwarpImageCols = 2 * PI * R * REAL_R;
    double d_tmp = 1000.0 / unwarpImageRows, tmp = unwarpImageRows * d_tmp;

    trho = new double [unwarpImageRows];
    for (int y = unwarpImageRows;y > 0;y--)
    {
        trho[unwarpImageRows - y] = unwarpImageRows * ((tmp - (int)tmp)*(arc_len[(int)tmp + 1] - arc_len[(int)tmp]) + arc_len[(int)tmp]);
        tmp -= d_tmp;
    }
    double theta, d_theta;
    theta = 0.0;
    d_theta = RMAX / R / REAL_R;
    u0 = orginalImageRows / 2.0;
    v0 = orginalImageCols / 2.0;

    theta_acc = new double [unwarpImageCols];
    for (int x = 0;x < unwarpImageCols;x++)
    {
        theta += d_theta;
        theta_acc[x] = theta;
    }

    return true;
}

cv::Mat ImageWrapper::getImage(cv::Mat image)
{
    srcImage = image;

    dstImage = cv::Mat(unwarpImageRows, unwarpImageCols, CV_8UC3, Scalar::all(0));

    u0 = orginalImageRows / 2.0;
    v0 = orginalImageCols / 2.0;

    // #pragma omp parallel for
    for (int x = 0;x < unwarpImageCols;x++)
    {
        // #pragma omp parallel for
        for (int y = 0;y < unwarpImageRows;y++)
        {
            dstImage.at<Vec3b>(y, x)[0] = srcImage.at<Vec3b>(trho[y] * tsin[(int)(theta_acc[x] + 0.5)] + v0,trho[y] * tcos[(int)(theta_acc[x] + 0.5)] + u0)[0];
            dstImage.at<Vec3b>(y, x)[1] = srcImage.at<Vec3b>(trho[y] * tsin[(int)(theta_acc[x] + 0.5)] + v0,trho[y] * tcos[(int)(theta_acc[x] + 0.5)] + u0)[1];
            dstImage.at<Vec3b>(y, x)[2] = srcImage.at<Vec3b>(trho[y] * tsin[(int)(theta_acc[x] + 0.5)] + v0,trho[y] * tcos[(int)(theta_acc[x] + 0.5)] + u0)[2];
        }
    }

    return dstImage;
}