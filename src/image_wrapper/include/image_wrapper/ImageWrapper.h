#include <opencv2/opencv.hpp>

class ImageWrapper
{
public:
	ImageWrapper();
	virtual ~ImageWrapper();

	bool initImageWrapper(unsigned cols, unsigned rows);
	cv::Mat getImage(cv::Mat image);

private:
	cv::Mat srcImage; /**< TODO: describe */
    cv::Mat dstImage; /**< TODO: describe */
    cv::Mat reSizeImage; /**< TODO: describe */

    unsigned orginalImageCols, orginalImageRows, /**< TODO: describe */
                unwarpImageCols, unwarpImageRows; /**< TODO: describe */
    unsigned displayedHeight = 831, displayedWidth = 1591;

    double *trho = NULL; /**< TODO: describe */
    double u0, v0; /**< TODO: describe */

    double *theta_acc = NULL;
};