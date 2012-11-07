#ifndef ImageTransformationNode_H
#define ImageTransformationNode_H

#include "iostream"

#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "Topics.h"
#include "Utilities/Utilities.h"
#include "DotMatrixNode/DotMatrixNodeSettings.h"

class ImageTransformationNode
{
	public:
		ImageTransformationNode(int equipletID, int moduleID);
		~ImageTransformationNode();

		void publishImage();
		void run();
	private:
		void transformCallback(const sensor_msgs::ImageConstPtr& msg);

		int blockSize;
		int maximum;
		int subtract;

		cv::Mat inputImage;
		cv::Mat outputImage;

		ros::NodeHandle nodeHandle;
		image_transport::ImageTransport imageTransport;
		image_transport::Publisher pub;
		image_transport::Subscriber cameraSubscriber;
};

#endif