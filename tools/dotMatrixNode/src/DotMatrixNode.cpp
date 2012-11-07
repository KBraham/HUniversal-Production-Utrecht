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

#define NODE_NAME "GripperNode"

DotMatrixNode::DotMatrixNode( ) :
		imageTransport(node) {

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
	// Move to X, Y, Zhigh

	// Move to X, Y, Zlow
	// TODO: find the Z for drawing

	// Move to X, Y, Zhigh
}

void DotMatrixNode::run( ) {
	//TODO imageData;

	//TODO assert sizewise? fix later for total size?
	assert(image.width <= DotMatrixNodeSettings::DRAW_FIELD_WIDTH * DotMatrixNodeSettings::DRAW_FIELD_DOTS_PER_MM);
	assert(image.height <= DotMatrixNodeSettings::DRAW_FIELD_HEIGHT * DotMatrixNodeSettings::DRAW_FIELD_DOTS_PER_MM);

	// Calculate X,Y by converting X,Y in pixels to X,Y in mm
	double deltaRobotX = -(DotMatrixNodeSettings::DRAW_FIELD_HEIGHT / 2.0);
	double deltaRobotY = DotMatrixNodeSettings::DRAW_FIELD_WIDTH / 2.0;

	

	unsigned int pixelPointer = 0;
	for(unsigned int j = 0; j < height; j++){
		if(j % 2 == 0){
			for (; (pixelPointer + 1) % width != 0; pixelPointer++) {
				//TODO if(imageData[pixelPointer]){
					drawDot(deltaRobotX, deltaRobotY);
				//}
				deltaRobotX += DotMatrixNodeSettings::DRAW_FIELD_MM_PER_DOTS;
			}
		} else {
			for (; pixelPointer % width != 0; pixelPointer--) {
				//TODO if(imageData[pixelPointer]){
					drawDot(deltaRobotX, deltaRobotY);
				//}
				deltaRobotX -= DotMatrixNodeSettings::DRAW_FIELD_MM_PER_DOTS;
			}
		}
		pixelPointer += width;
		deltaRobotY += DotMatrixNodeSettings::DRAW_FIELD_MM_PER_DOTS;
	}

	//for (unsigned int i = 0; i < gimp_image.height * gimp_image.width * gimp_image.bytes_per_pixel; i += gimp_image.bytes_per_pixel) {
	//	if (i % 50 == 0) {
	//		std::cout << std::endl;
	//	}
	//	if (gimp_image.pixel_data[i] == 0 && gimp_image.pixel_data[i] == gimp_image.pixel_data[i + 1] && gimp_image.pixel_data[i + 1] == gimp_image.pixel_data[i + 2]) {
	//		std::cout << "#";
	//	} else {
	//		std::cout << " ";
	//	}
	//}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, NODE_NAME);

	DotMatrixNode dotMatrixNode;
	dotMatrixNode.run();

	return 0;
}

