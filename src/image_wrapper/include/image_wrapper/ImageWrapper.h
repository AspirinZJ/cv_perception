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

	double *trho = nullptr; /**< TODO: describe */
	double u0, v0; /**< TODO: describe */

	/**
	 * @brief table for looking up the pixel indices: srcImage --> dstImage
	 * @details 2D int array
	 * size: [unWarpImageCols][2 x unWarpImagesRows].
	 * *(*(this->indTable + x) + 2 * y) --> xVal;
	 * *(*(this->indTable + x) + 2 * y + 1) --> yVal;
	 */
	short **indTable;

	double *theta_acc = nullptr;
};