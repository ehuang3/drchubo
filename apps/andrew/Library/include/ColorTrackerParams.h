/**
 * \file ColorTrackerParams.h
 * \brief Contains struct for color tracker settings
 * 
 * \author Andrew Price
 */

#include <opencv2/core/core.hpp>

/**
 * \struct ColorTrackerParams
 * \brief Provides configuration settings for a color tracking class
 */
struct ColorTrackerParams
{
	cv::Scalar lowerColorBound;		///< Sets the HSV space lower bound for the color tracker
	cv::Scalar upperColorBound;		///< Sets the HSV space upper bound for the color tracker
	int erosionRadius;				///< Sets the radius to erode the thresholded image (before dilation)
	int dilationRadius;				///< Sets the radius to dilate the thresholded image (after erosion)
	int blurRadius;					///< Sets the radius to gaussian blur the input image (before thresholding)
	bool displayThresholdedImage;	///< Sets whether to display the binary image in a cv::imshow window
};
