/*
 * DotMatrix.cpp
 *
 *  Created on: Nov 6, 2012
 *      Author: kbraham
 */

#include "iostream"
#include "ros/ros.h"
#include <DotMatrixNode/DotMatrixNode.h>
#include <DotMatrixNode/DotMatrixNodeSettings.h>
#include <DotMatrixNode/Image.h>

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
 */
void DotMatrixNode::drawDot(int x, int y) {
	// Move to X, Y

	// Move to Zlow
	// TODO: find the Z for drawing

	// Move to Zhigh

	// Done dotting?

	std::cout << x << " " << y << std::endl;
}

void DotMatrixNode::run( ) {
	// Calculate X,Y by converting X,Y in pixels to X,Y in mm
	// ===========
	// =-+-------=  + = Startpoint of the drawing (0,0)
	// =---------=
	// =----x----=  x = Startpoint of deltarobot (0,0)
	// =---------=
	// =---------=
	// ===========
	double startX = -(DotMatrixNodeSettings::DRAW_FIELD_WIDTH / 2.0);
	double startY = -(DotMatrixNodeSettings::DRAW_FIELD_HEIGHT / 2.0);

	//* Converts the coordinates to the deltarobot coordinates (x,y,z)
	std::cout << startX << std::endl;
	for (int i = 0; i < 10; ++i) {
		drawDot((i % DotMatrixNodeSettings::DRAW_FIELD_WIDTH) + startX, (i / DotMatrixNodeSettings::DRAW_FIELD_WIDTH) + startY);
	}

	for (unsigned int i = 0; i < gimp_image.height * gimp_image.width * gimp_image.bytes_per_pixel; i += gimp_image.bytes_per_pixel) {
		if (i % 50 == 0) {
			std::cout << std::endl;
		}
		if (gimp_image.pixel_data[i] == 0 && gimp_image.pixel_data[i] == gimp_image.pixel_data[i + 1] && gimp_image.pixel_data[i + 1] == gimp_image.pixel_data[i + 2]) {
			std::cout << "#";
		} else {
			std::cout << " ";
		}
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, NODE_NAME);

	DotMatrixNode dotMatrixNode;
	dotMatrixNode.run();

	return 0;
}

