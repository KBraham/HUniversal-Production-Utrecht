/*
 * DotMatrix.cpp
 *
 *  Created on: Nov 6, 2012
 *      Author: kbraham
 */

#include "iostream"
#include <assert.h>
#include "ros/ros.h"
#include <DotMatrixNode/DotMatrixNode.h>
#include <DotMatrixNode/DotMatrixNodeSettings.h>
#include <DotMatrixNode/Image.h>
#include <DeltaRobotNode/Services.h>
#include <ImageTransformationNode/Topics.h>

#define NODE_NAME "GripperNode"

DotMatrixNode::DotMatrixNode( ) :
		imageTransport(nodeHandle), deltaRobotClient(nodeHandle.serviceClient<deltaRobotNode::MoveToPoint>(DeltaRobotNodeServices::MOVE_TO_POINT)) {
	moveToPointService.request.motion.x = 0;
	moveToPointService.request.motion.y = 0;
	moveToPointService.request.motion.z = -230;
	moveToPointService.request.motion.speed = 300;
}

DotMatrixNode::~DotMatrixNode( ) {

}

/**
 * Draws a dot at coordinate X, Y with a 0.5mm diameter
 * @param x X coordinate of the dotted pixel
 * @param y Y coordinate of the dotted pixel
 * TODO: offset drawing for centering the image!
 **/
void DotMatrixNode::drawDot(int x, int y) {
	// Move to X, Y, Zhigh
	moveToPointService.request.motion.x = x;
	moveToPointService.request.motion.y = y;
	moveToPointService.request.motion.z = DotMatrixNodeSettings::DRAW_FIELD_Z_HIGH;
	deltaRobotClient.call(moveToPointService);

	// Move to X, Y, Zlow
	// TODO: find the Z for drawing
	moveToPointService.request.motion.x = x;
	moveToPointService.request.motion.y = y;
	moveToPointService.request.motion.z = DotMatrixNodeSettings::DRAW_FIELD_Z_LOW;
	deltaRobotClient.call(moveToPointService);

	// Move to X, Y, Zhigh
	moveToPointService.request.motion.x = x;
	moveToPointService.request.motion.y = y;
	moveToPointService.request.motion.z = DotMatrixNodeSettings::DRAW_FIELD_Z_HIGH;
	deltaRobotClient.call(moveToPointService);

	// Done dotting?

	std::cout << x << " " << y << std::endl;
}


/**
 * Transforms the image on the topic to the correct size and format and publishes to a new topic.
 *
 * @param msg The pointer to the message that contains the camera image.
 **/
void DotMatrixNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	// Receive image
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	std::cout << "Encoding: " << cv_ptr->encoding << std::endl;

	int cols = cv_ptr->image.cols;
	int rows = cv_ptr->image.rows;

		//TODO assert sizewise? fix later for total size?
		assert(cols <= DotMatrixNodeSettings::DRAW_FIELD_WIDTH * DotMatrixNodeSettings::DRAW_FIELD_DOTS_PER_MM);
		assert(rows <= DotMatrixNodeSettings::DRAW_FIELD_HEIGHT * DotMatrixNodeSettings::DRAW_FIELD_DOTS_PER_MM);

		// Calculate X,Y by converting X,Y in pixels to X,Y in mm
		// ===========
		// =+--------=  + = Startpoint of the drawing (0,0)
		// =---------=
		// =----x----=  x = Startpoint of deltarobot (0,0)
		// =---------=
		// =---------=
		// ===========
		double drawX = -(DotMatrixNodeSettings::DRAW_FIELD_WIDTH / 2.0);
		double drawY = DotMatrixNodeSettings::DRAW_FIELD_HEIGHT / 2.0;

		unsigned int pixelPointer = 0;
		for (int j = 0; j < rows; j++) {
			if (j % 2 == 0) {
				for (; (pixelPointer + 1) % cols != 0; pixelPointer++) {
					if(cv_ptr->image.data[pixelPointer] == 0){
						drawDot(drawX, drawY);
					}
					drawX += DotMatrixNodeSettings::DRAW_FIELD_MM_PER_DOTS;
				}
			} else {
				for (; pixelPointer % cols != 0; pixelPointer--) {
					if(cv_ptr->image.data[pixelPointer] == 0){
						drawDot(drawX, drawY);
					}
					drawX -= DotMatrixNodeSettings::DRAW_FIELD_MM_PER_DOTS;
				}
			}
			pixelPointer += cols;
			drawY -= DotMatrixNodeSettings::DRAW_FIELD_MM_PER_DOTS;
		}
}


/**
 * Blocking function that contains the main loop.
 * Spins in ROS to receive frames. These will execute the callbacks.
 * This function ends when ros receives a ^c
 **/
void DotMatrixNode::run( ) {
	imageSubscriber = imageTransport.subscribe(ImageTransformationNodeTopics::TRANSFORMED_IMAGE, 1, &DotMatrixNode::imageCallback, this, image_transport::TransportHints("compressed"));

	while(ros::ok()) {
		ros::spinOnce();
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, NODE_NAME);

	DotMatrixNode dotMatrixNode;
	dotMatrixNode.run();

	return 0;
}

