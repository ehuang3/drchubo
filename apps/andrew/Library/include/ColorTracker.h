/**
 * \file ColorTracker.h
 * \brief Thresholds an image for a given color and returns the center of mass.
 * 
 * \author Andrew Price
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ColorTrackerParams.h"

/**
 * \class ColorTracker
 * \brief Provides thresholding and CoM detection for color images
 */
class ColorTracker
{
public:
	/// Constructor with default tracking parameters (red color)
	ColorTracker();
	
	/// Constructor with custom tracking parameters
	ColorTracker(ColorTrackerParams params);

	/// Destructor
	~ColorTracker();

	/// Detect colored blobs
	cv::Mat thresholdImage(cv::Mat target);

	/// Threshold the image, erode and dilate it, and compute the central moment
	cv::Point getCoM(cv::Mat target);

	/// Set new tracking parameters
	void setTrackingParameters(ColorTrackerParams params);

	/// Get current tracking parameters
	ColorTrackerParams getTrackingParameters();
	

private:
	ColorTrackerParams mParameters;

};
