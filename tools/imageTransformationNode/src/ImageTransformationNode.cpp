/**
 * @file ImageTransformationNode.cpp
 * @brief Transforms the image from the camera with a GUI and adjustable adaptive threshold. It sends out the image on a topic.
 * @date 2012-11-06
 *
 * @author Daan Veltman
 *
 * @section LICENSE
 * License: newBSD
 * 
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include "ImageTransformationNode.h"

#define NODE_NAME "ImageTransformationNode"

/**
 * @var WINDOW_NAME
 * Name for the opencv image window.
 **/
static const char WINDOW_NAME[] = "Image window";

/**
 * On mouse click event. 
 *
 * @param event CV_EVENT (eg. CV_EVENT_LBUTTONDOWN).
 * @param x X coordinate of the click.
 * @param y Y coordinate of the click.
 * @param flags CV_EVENT_FLAG.
 * @param param ImageTransformationNode.
 **/
void on_mouse(int event, int x, int y, int flags, void* param) {
	if (event == CV_EVENT_RBUTTONDOWN) {
		((ImageTransformationNode*)param)->publishImage();
	}
}

/**
 * Constructor.
 *
 * @param equipletID Equiplet identifier.
 * @param moduleID Module identifier.
 **/
ImageTransformationNode::ImageTransformationNode(int equipletID, int moduleID) : imageTransport(nodeHandle) {
	blockSize = 15;
	maximum = 255;
	subtract = 15;

	// Advertise the services
	pub = imageTransport.advertise(ImageTransformationNodeTopics::TRANSFORMED_IMAGE, 1);

	// OpenCV GUI
	cv::namedWindow(WINDOW_NAME, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
	cv::createTrackbar("blockSize:", WINDOW_NAME, &blockSize, maximum );
	cv::createTrackbar("subtract :", WINDOW_NAME, &subtract, maximum );

	cvSetMouseCallback(WINDOW_NAME, &on_mouse, this);
}

/**
 * Publishes the outputImage as a CvImage with the cv_bridge.
 **/
void ImageTransformationNode::publishImage(){
	ros::Time time = ros::Time::now();
	cv_bridge::CvImage cvi;
	cvi.header.stamp = time;
	cvi.header.frame_id = "image";
	cvi.encoding = sensor_msgs::image_encodings::MONO8;
	cvi.image = outputImage;
	pub.publish(cvi.toImageMsg());
}

/**
 * Transforms the image on the topic to the correct size and format and publishes to a new topic.
 *
 * @param msg The pointer to the message that contains the camera image.
 **/
void ImageTransformationNode::transformCallback(const sensor_msgs::ImageConstPtr& msg) {
	// Receive image
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	double scale = std::max(
		cv_ptr->image.rows / (DotMatrixNodeSettings::DRAW_FIELD_HEIGHT * DotMatrixNodeSettings::DRAW_FIELD_DOTS_PER_MM), 
		cv_ptr->image.cols / (DotMatrixNodeSettings::DRAW_FIELD_WIDTH * DotMatrixNodeSettings::DRAW_FIELD_DOTS_PER_MM));
	cv::Size outputSize = cv::Size(cv_ptr->image.cols / scale, cv_ptr->image.rows / scale);

	cv::Mat resizedImage;
	cv::resize(cv_ptr->image, resizedImage, outputSize);

	cv::Mat grayImage;
	cv::cvtColor(resizedImage, grayImage, CV_BGR2GRAY);
	
	//Threshold the image, note that blocksize has to be a multiple of 3 and >= 3.
	cv::Mat thresholdedImage;
	cv::adaptiveThreshold(grayImage, outputImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, blockSize < 3 ? 3 : blockSize | 1, subtract);
	//TODO determine if output is a safe image to use for sending onwards
	cv::imshow(WINDOW_NAME, outputImage);
	cv::waitKey(3);
}

/**
 * Blocking function that contains the main loop.
 * Spins in ROS to receive frames. These will execute the callbacks.
 * This function ends when ros receives a ^c
 **/
void ImageTransformationNode::run( ) {
	cameraSubscriber = imageTransport.subscribe("camera/image", 1, &ImageTransformationNode::transformCallback, this, image_transport::TransportHints("compressed"));

	while(ros::ok()) {
		ros::spinOnce();
	}
}

/**
 * Main
 *
 * @param argc Argument count.
 * @param argv Node name, equipletID, moduleID.
 *
 * @return 0 if succesful, -1 if command line arguments are incorrect
 **/
int main(int argc, char** argv){
	ros::init(argc, argv, NODE_NAME);
	int equipletID = 0;
	int moduleID = 0;
	if(argc < 3 || !(Utilities::stringToInt(equipletID, argv[1]) == 0 && Utilities::stringToInt(moduleID, argv[2]) == 0))
	{ 	 	
    	std::cerr << "Cannot read equiplet id and/or moduleId from commandline please use correct values." << std::endl;
 		return -1;
  	} 
	ImageTransformationNode imageTransformationNode(equipletID, moduleID);
	imageTransformationNode.run();
	return 0;
}