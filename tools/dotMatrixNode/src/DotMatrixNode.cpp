/**
 * @file DotMatrixNode.cpp
 * @brief Semi dot matrix printer for grayscale image files.
 * @date Created: 2012-11-06
 *
 * @author Koen Braham
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

#include "iostream"
#include <assert.h>
#include "ros/ros.h"
#include <DotMatrixNode/DotMatrixNode.h>
#include <DotMatrixNode/DotMatrixNodeSettings.h>
#include <DeltaRobotNode/Services.h>
#include <ImageTransformationNode/Topics.h>
#include <Utilities/Utilities.h>

#define NODE_NAME "GripperNode"

DotMatrixNode::DotMatrixNode( ) :
		imageTransport(nodeHandle), deltaRobotClient(nodeHandle.serviceClient<deltaRobotNode::MoveToPoint>(DeltaRobotNodeServices::MOVE_TO_POINT)), deltaRobotPathClient(nodeHandle.serviceClient<deltaRobotNode::MovePath>(DeltaRobotNodeServices::MOVE_PATH)) {
	moveToPointService.request.motion.x = 0;
	moveToPointService.request.motion.y = 0;
	moveToPointService.request.motion.z = DotMatrixNodeSettings::DRAW_FIELD_Z_HIGH;
	moveToPointService.request.motion.speed = DotMatrixNodeSettings::SPEED;

	point.x = 0;
	point.y = 0;
	point.z = DotMatrixNodeSettings::DRAW_FIELD_Z_HIGH;
	point.speed = DotMatrixNodeSettings::SPEED;

}

DotMatrixNode::~DotMatrixNode( ) {

}

/**
 * Draws a dot at coordinate x, y.
 * @param x X coordinate of the dotted pixel
 * @param y Y coordinate of the dotted pixel
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
 * Draws a dot at coordinate x, y.
 * @param x X coordinate of the dotted pixel
 * @param y Y coordinate of the dotted pixel
 **/
void DotMatrixNode::drawDotToPath(int x, int y) {
	point.x = x;
	point.y = y;
	movePathService.request.motion.push_back(point);
	point.z = DotMatrixNodeSettings::DRAW_FIELD_Z_LOW;
	movePathService.request.motion.push_back(point);
	point.z = DotMatrixNodeSettings::DRAW_FIELD_Z_HIGH;
	movePathService.request.motion.push_back(point);
}

/**
 * Transforms the image on the topic to the correct size and format and publishes to a new topic.
 *
 * @param msg The pointer to the message that contains the camera image.
 **/
void DotMatrixNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	bool pathDrawing = true;

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

	assert(cols <= DotMatrixNodeSettings::DRAW_FIELD_WIDTH * DotMatrixNodeSettings::DRAW_FIELD_DOTS_PER_MM);
	assert(rows <= DotMatrixNodeSettings::DRAW_FIELD_HEIGHT * DotMatrixNodeSettings::DRAW_FIELD_DOTS_PER_MM);

	// Calculate X,Y by converting X,Y in pixels to X,Y in mm
	// ===========
	// =+--------=  + = Startpoint of the drawing (0,0)
	// =--o------=	x = endpoint of the drawing
	// =----x----=  x = Startpoint of deltarobot (0,0)
	// =---------=	o = Offset starting point of the deltaRobot (drawX, drawY)
	// =---------=
	// ===========
	double drawX = -(DotMatrixNodeSettings::DRAW_FIELD_WIDTH / 2.0) + ((DotMatrixNodeSettings::DRAW_FIELD_WIDTH * DotMatrixNodeSettings::DRAW_FIELD_DOTS_PER_MM - cols) / 2.0);
	double drawY = (DotMatrixNodeSettings::DRAW_FIELD_HEIGHT / 2.0) - ((DotMatrixNodeSettings::DRAW_FIELD_HEIGHT * DotMatrixNodeSettings::DRAW_FIELD_DOTS_PER_MM - rows) / 2.0);

	// Clear the old path.
	if (pathDrawing) {
		movePathService.request.motion.clear();
	}

	unsigned int pixelPointer = 0;
	// moves left to right, right to left
	for (int j = 0; j < rows; j++) {
		if (j % 2 == 0) {
			for (; (pixelPointer + 1) % cols != 0; pixelPointer++) {
				if (cv_ptr->image.data[pixelPointer] == 0) {
					if (!pathDrawing) {
						drawDot(drawX, drawY); //point for point drawing
					} else {
						drawDotToPath(drawX, drawY); //path drawing
					}
				}
				drawX += DotMatrixNodeSettings::DRAW_FIELD_MM_PER_DOTS;
			}
		} else {
			for (; pixelPointer % cols != 0; pixelPointer--) {
				if (cv_ptr->image.data[pixelPointer] == 0) {
					if (!pathDrawing) {
						drawDot(drawX, drawY); //point for point drawing
					} else {
						drawDotToPath(drawX, drawY); //path drawing
					}
				}
				drawX -= DotMatrixNodeSettings::DRAW_FIELD_MM_PER_DOTS;
			}
		}
		pixelPointer += cols;
		drawY -= DotMatrixNodeSettings::DRAW_FIELD_MM_PER_DOTS;
	}

	if (pathDrawing) {
		Utilities::StopWatch stopwatch("PathTimer", true);
		deltaRobotPathClient.call(movePathService); //path drawing
		stopwatch.stopAndPrint(stdout);
	}
}

/**
 * Blocking function that contains the main loop.
 * Spins in ROS to receive frames. These will execute the callbacks.
 * This function ends when ros receives a ^c
 **/
void DotMatrixNode::run( ) {
	imageSubscriber = imageTransport.subscribe(ImageTransformationNodeTopics::TRANSFORMED_IMAGE, 1, &DotMatrixNode::imageCallback, this, image_transport::TransportHints("compressed"));

	while (ros::ok()) {
		ros::spinOnce();
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, NODE_NAME);

	DotMatrixNode dotMatrixNode;
	dotMatrixNode.run();

	return 0;
}

