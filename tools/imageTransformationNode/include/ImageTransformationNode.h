#ifndef ImageTransformationNode_H
#define ImageTransformationNode_H

#include <rosMast/StateMachine.h>
#include "ros/ros.h"
#include "Topics.h"
#include "iostream"

// TODO: #include <DotMatrixNode/DotMatrixNodeSettings.h>

class ImageTransformationNode: public rosMast::StateMachine
{
	public:
		ImageTransformationNode(int equipletID, int moduleID);
		~ImageTransformationNode();
	private:
		int blockSize = 15;
		int maximum = 255;
		int subtract = 15;

		cv::Mat inputImage;
		cv::Mat outputImage;

		ros::NodeHandle NodeHandle;
		image_transport::ImageTransport it;
		image_transport::Publisher pub;
		image_transport::Subscriber cameraSubscriber;
};

#endif