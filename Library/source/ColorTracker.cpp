/**
 * \file ColorTracker.cpp
 * \brief Thresholds an image for a given color and returns the center of mass.
 * 
 * \author Andrew Price
 */

#include "ColorTracker.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

ColorTracker::ColorTracker()
{
	ColorTrackerParams params;
	params.lowerColorBound = cv::Scalar(160,65,60);
	params.upperColorBound = cv::Scalar(190,256,256);
	params.erosionRadius = 5;
	params.dilationRadius = 5;
	params.blurRadius = 2;
	params.displayThresholdedImage = false;
	mParameters = params;
}

ColorTracker::ColorTracker(ColorTrackerParams params)
{
	mParameters = params;
}

ColorTracker::~ColorTracker() {}

cv::Mat ColorTracker::thresholdImage(cv::Mat target)
{
	// Convert the image into an HSV image
	cv::Mat imgHSV;
	cv::cvtColor(target, imgHSV, CV_BGR2HSV);
	
	// Stores the matched color pixels
	cv::Mat imgThreshed;

	// Filter the image
	if (mParameters.blurRadius != 0)
	{
		cv::GaussianBlur(imgHSV,imgHSV, 
			cv::Size(mParameters.blurRadius * 2 + 1,mParameters.blurRadius * 2 + 1),
			mParameters.blurRadius * 2 + 1);
	}

	// Threshold the HSV image
	cv::inRange(imgHSV, mParameters.lowerColorBound, mParameters.upperColorBound, imgThreshed);

	imgHSV.release();

	return imgThreshed;
}

cv::Point ColorTracker::getCoM(cv::Mat target)
{
	// Get the thresholded image
	cv::Mat imgThresh = thresholdImage(target);

	// Perform a morphological opening
	cv::Mat eElement = cv::getStructuringElement(
		cv::MORPH_ELLIPSE,
		cv::Size(2*mParameters.erosionRadius+1,2*mParameters.erosionRadius+1),
		cv::Point(mParameters.erosionRadius,mParameters.erosionRadius));
	cv::Mat dElement = cv::getStructuringElement(
		cv::MORPH_ELLIPSE,
		cv::Size(2*mParameters.dilationRadius+1,2*mParameters.dilationRadius+1),
		cv::Point(mParameters.dilationRadius,mParameters.dilationRadius));


	cv::erode(imgThresh, imgThresh, eElement);
	cv::dilate(imgThresh, imgThresh, dElement);

	// Possibly show image
	if (mParameters.displayThresholdedImage)
	{
		cv::imshow("thresh",imgThresh);
	}

	// Calculate the moments to estimate the position of the ball
	cv::Moments moments = cv::moments(imgThresh, 1);
	
	// The actual moment values
	double moment10 = moments.m10;
	double moment01 = moments.m01;
	double area = moments.m00;
	if (area == 0)
	{
		area = 1;
	}

	// Holding the last and current ball positions
	int posX = 0;
	int posY = 0;
	
	posX = (int)(moment10/area);
	posY = (int)(moment01/area);

	imgThresh.release();

	return cv::Point(posX, posY);
}


void ColorTracker::setTrackingParameters(ColorTrackerParams params)
{
	mParameters = params;
}

ColorTrackerParams ColorTracker::getTrackingParameters()
{
	return mParameters;
}

