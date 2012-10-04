//******************************************************************************
//
//                 REXOS
//
//******************************************************************************
// Project:        DeltaRobot
// File:           EffectorBoundaries.cpp
// Description:    represents the effector's moving volume
// Author:         1.0 Lukas Vermond & Kasper van Nieuwland
//				   1.1 Koen Braham		Daan Veltman
// Notes:          
//
// License:        newBSD
//
// Copyright © 2012, HU University of Applied Sciences Utrecht
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of the HU University of Applied Sciences Utrecht nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

#include <iostream>
#include <DeltaRobot/Measures.h>
#include <DeltaRobot/EffectorBoundaries.h>
#include <DeltaRobot/InverseKinematicsException.h>
#include <DeltaRobot/EffectorBoundariesException.h>
#include <stack>
#include <vector>
#include <set>
#include <cstring>

namespace DeltaRobot
{

	/**
	 * Function to generate the boundaries and returns a pointer to the object
	 * @param model used to calculate the boundaries
	 * @param motors used for the minimum and maximum angle of the motors
	 * @param voxelSize the size of the voxels in mm
	 * @return pointer to the object
	 **/
	EffectorBoundaries* EffectorBoundaries::generateEffectorBoundaries(const InverseKinematicsModel& model,  Motor::StepperMotor* (&motors)[3], double voxelSize)
	{
		EffectorBoundaries* boundaries = new EffectorBoundaries(model, motors, voxelSize);
		
		//create boundaries variables in voxel space by dividing real space variables with the voxel size
		boundaries->width = (Measures::MAX_X  - Measures::MIN_X) / voxelSize;
        boundaries->height = (Measures::MAX_Z - Measures::MIN_Z) / voxelSize;
        boundaries->depth = (Measures::MAX_Y  - Measures::MIN_Y) / voxelSize;
        //create bitmap with value false for all voxels
        boundaries->boundariesBitmap = new bool[boundaries->width * boundaries->height * boundaries->depth];
        for(int i = 0; i < boundaries->width * boundaries->height * boundaries->depth; i++)
        {
        	boundaries->boundariesBitmap[i] = false;
        }
        boundaries->generateBoundariesBitmap();
        return boundaries;
    }

	/**
	 * Checks if the path from the starting to the destination point is not going out of the
	 * robots boundaries
	 * @param from the starting point
	 * @param to the destination point
	 *
	 * @return true if a straight path from from to to is valid
	 **/
    bool EffectorBoundaries::checkPath(const DataTypes::Point3D<double> & from, const DataTypes::Point3D<double> & to) const
    {
    	double x_length = to.x - from.x;
    	double y_length = to.y - from.y;
    	double z_length = to.z - from.z;
    	double largest_length = (double)(fabs(x_length) > fabs(y_length) ?
    			(fabs(x_length) > fabs(z_length) ? fabs(x_length) : fabs(z_length)) :
    			(fabs(y_length) > fabs(z_length) ? fabs(y_length) : fabs(z_length)));

    	x_length = x_length / largest_length;
    	y_length = y_length / largest_length;
    	z_length = z_length / largest_length;
    	
		for(double i = 1; i <= largest_length; i++)
		{
			int x = (from.x + x_length * i);
			int y = (from.y + y_length * i);
			int z = (from.z + z_length * i);
			BitmapCoordinate temp = fromRealCoordinate(DataTypes::Point3D<double>(x, y, z));
			int index = temp.x + temp.y * width + temp.z * width * depth;

			if(temp.x < 0
				|| temp.x > width
				|| temp.y < 0
				|| temp.y > depth
				|| temp.z < 0
				|| temp.z > height
				|| !boundariesBitmap[index]){
				return false;
			}
		}
        return true;
    }

	/**
	 * private constructor
	 * also initializes the voxel array
	 * @param model used to calculate the boundaries
	 * @param motors used for the minimum and maximum angle of the motors
	 * @param voxelSize the size of the voxels
	 **/
    EffectorBoundaries::EffectorBoundaries(const InverseKinematicsModel& model,  Motor::StepperMotor* (&motors)[3], double voxelSize)
    	: kinematics(model), motors(motors), voxelSize(voxelSize) { }

    EffectorBoundaries::~EffectorBoundaries()
    {

    	delete[] boundariesBitmap;
    }

	/**
	 * Checks if one of the neighbouring voxels can't be reached by the effector. This includes voxels outside of the MIN/MAX_X/Y/Z box as defined in measures.
	 * @param p The point in the bitmap that has to be checked.
	 * @param pointValidityCache Pointer to the cache where already checked values are stored, and unchecked points are unknown. This as opposed to the bitmap, which is defaulted to false instead of unknown.
	 *
	 * @return True if p has unreachable neighbouring voxels.
	 **/
	bool EffectorBoundaries::hasInvalidNeighbours(const BitmapCoordinate & p, char* pointValidityCache) const {
		//TODO: change from has_invalid_neighbours to isOnTheEdgeOfValidArea due to functionality change

    	//check if the voxel is valid and on the edge of the box
    	if(isValid(BitmapCoordinate(p.x,p.y,p.z), pointValidityCache)){
    		//voxel is on the edge of the box, automatically boardering invalid territory
    		if(p.x == 0 
    			|| p.x == width 
    			|| p.y == 0 
    			|| p.y == depth 
    			|| p.z == 0 
    			|| p.z == height){
    			return true;
    		}

	        //check all voxels in the 3x3x3 box around voxel p
	        for(int y = p.y - 1; y <= p.y + 1; y++){
	            for(int x = p.x - 1; x <= p.x + 1; x++){
	                for(int z = p.z - 1; z <= p.z + 1; z++){
	                    //check if one of the neighbours is not valid
	                    if(!isValid(BitmapCoordinate(x, y, z), pointValidityCache)){
	                        return true;
	                    }
	                }
	            }
	        }
	    }
	    // voxel is invalid OR inside of a box of 3x3x3 valid voxels
        return false;
    }

	/**
	 * Checks if the point can be reached by the effector. Whether the point can be reached is determined by the kinematics, minimum and maximum angles of the motors and the MIN/MAX_X/Y/Z box determined in measures.
	 * @param p The point that is checked if it can be reached by the effector.
	 * @param pointValidityCache Pointer to the cache where already checked values are stored, and unchecked points are unknown. This as opposed to the bitmap, which is defaulted to false instead of unknown.
	 * 
	 * @return True if p is reachable by the effector.
	 **/
    bool EffectorBoundaries::isValid(const BitmapCoordinate& p, char* pointValidityCache) const {
    	char* fromCache;
    	char dummy = UNKNOWN;
    	if(pointValidityCache == NULL)
    	{
    		fromCache = &dummy;
    	}
    	else
    	{
    		fromCache = &pointValidityCache[p.x + p.y * width + p.z * width * depth];
    	}

    	if(*fromCache == UNKNOWN)
    	{
    		DataTypes::DeltaRobotRotation drr;
			try {
				kinematics.pointToMotion(fromBitmapCoordinate(p), drr);

			} catch(DeltaRobot::InverseKinematicsException & ex) {
				*fromCache = INVALID;
				return false;
			}


			// Check motor angles
			if(drr.rotations[0].angle <= motors[0]->getMinAngle() || drr.rotations[0].angle >= motors[0]->getMaxAngle() ||
			  drr.rotations[1].angle <= motors[1]->getMinAngle() || drr.rotations[1].angle >= motors[1]->getMaxAngle()  ||
			  drr.rotations[2].angle <= motors[2]->getMinAngle() || drr.rotations[2].angle >= motors[2]->getMaxAngle()  ){
			  	*fromCache = INVALID;
			  	return false;
			}


			*fromCache = VALID;
			return true;
    	}
    	else
    	{
    		return *fromCache == VALID;
    	}
    }

	/**
	 * Generates boundaries for the robot. All members should be initialized before calling this function.
	 **/
    void EffectorBoundaries::generateBoundariesBitmap()
    {
    	char * pointValidityCache = new char[width * depth * height];
    	memset(pointValidityCache, 0, width * depth * height * sizeof(char));
    	std::stack<BitmapCoordinate> cstack;

    	//determine the center of the box
    	DataTypes::Point3D<double> begin (0, 0, Measures::MIN_Z + (Measures::MAX_Z - Measures::MIN_Z) / 2);
    	//if begin pixel is not part of a valid voxel the box dimensions are incorrect
    	if(!isValid(fromRealCoordinate(begin), pointValidityCache)){
    		throw EffectorBoundariesException("starting point outside of valid area, please adjust MAX/MIN_X/Y/Z values to have a valid center");
    	}
    	
    	//scan towards the right
		for(; begin.x < Measures::MAX_X; begin.x += voxelSize)
		{
			/**
			 * If an invalid voxel is found:
			 * step back to the last valid voxel
			 * push the voxel on the empty stack
			 * set the voxel as true in the bitmap
			 * end the loop
			 */
			if(!isValid(fromRealCoordinate(begin), pointValidityCache)){
				begin.x -= voxelSize;
				BitmapCoordinate startingVoxel = fromRealCoordinate(begin);
				cstack.push(startingVoxel);
				boundariesBitmap[startingVoxel.x + startingVoxel.y * width + startingVoxel.z * width * depth] = true;
				break;
			}
		}
		/**
		 * If the right-most voxel is in reach and an invalid voxel is never found the position of begin.x will be outside of the box limits. Step back inside the box and add that voxel to the stack and set it as true in the bitmap.
		 */
		if(begin.x >= Measures::MAX_X){
			begin.x -= voxelSize;
			BitmapCoordinate startingVoxel = fromRealCoordinate(begin);
			cstack.push(startingVoxel);
			boundariesBitmap[startingVoxel.x 
				+ startingVoxel.y * width 
				+ startingVoxel.z * width * depth] = true;
		}

		/**
		 * Start with the last added voxel on the stack and add new voxels to the stack. Do this until the valid borders (all valid voxels bordering unvalid voxels or the MAX/MIN_X/Y/Z box) of the valid voxel area are known (stack = empty).
		 */
		while(!cstack.empty())
		{
			//get last added voxel from the stack and remove it from the stack
			BitmapCoordinate borderVoxel = cstack.top();
			cstack.pop();

			//check all neighbours of the voxel
			for(int y = borderVoxel.y-1; y <= borderVoxel.y+1; y++){
				for(int x = borderVoxel.x-1; x <= borderVoxel.x+1; x++){
					for(int z = borderVoxel.z-1; z <= borderVoxel.z+1; z++){
						//don't do anything with voxels outside of the MAX/MIN_X/Y/Z box
						if(z >= height || z < 0 || x >= width || x < 0 || y >= depth || y < 0){
							continue;
						} else {
							/**
							 * new valid voxels on the valid border are 
							 * added to the stack and set in the bitmap
							 */
							int index = x + y * width + z * width * depth;
							if(isValid(BitmapCoordinate(x, y, z), pointValidityCache)
									&& !boundariesBitmap[index]
								    && hasInvalidNeighbours(BitmapCoordinate(x, y, z), pointValidityCache)){
								BitmapCoordinate bitmapCoordinate = BitmapCoordinate(x, y, z);
								cstack.push(bitmapCoordinate);
								boundariesBitmap[index] = true;
							}
						}
					}
				}
			}
		}

		delete[] pointValidityCache;
		pointValidityCache = NULL;
		
		//adds all the points within the boundaries
		cstack.push(fromRealCoordinate(DataTypes::Point3D<double>(0, 0, Measures::MIN_Z + (Measures::MAX_Z - Measures::MIN_Z) / 2)));
		while(!cstack.empty())
		{
			BitmapCoordinate validVoxel = cstack.top();
			cstack.pop();
			if(validVoxel.x <= 0
				|| validVoxel.x >= width
				|| validVoxel.y <= 0
				|| validVoxel.y >= depth
				|| validVoxel.z <= 0
				|| validVoxel.z >= height){
				continue;
			}
			int index = validVoxel.x + 
						validVoxel.y * width + 
						validVoxel.z * width * depth;

			int indices[6] = {
				index - 1, 
				index + 1, 
				index - width, 
				index + width, 
				index - width * depth, 
				index + width * depth
			};

			for(unsigned int i = 0; i < ( sizeof(indices) / sizeof(indices[0]) ); i++)
			{
				if(indices[i] < ((width*height*depth))){
					if(boundariesBitmap[indices[i]] == false){
						boundariesBitmap[indices[i]] = true;
						cstack.push(BitmapCoordinate(indices[i] % width, (indices[i] % (width * depth)) / width, indices[i] / (width * depth)));
					}
				}
			}	
		}
	}
}