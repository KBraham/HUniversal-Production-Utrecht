/**
 * @file PickAndPlace.cpp
 * @brief Provide the services to perform pick and place actions.
 * @date Created: 2012-09-19
 *
 * @author Patrick de Wit
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

#include <PickAndPlace/PickAndPlace.h>

using namespace std;

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "DeltaRobotNode"
// @endcond

namespace PickAndPlaceNamespace {
	PickAndPlace::PickAndPlace() {
		
	}

	void PickAndPlace::pick() {
		
	}

	void PickAndPlace::place() {

	}

	void PickAndPlace::moveToPoint(string command, pair<char, int> coordinates[], int workspaceid, int parentid, int resourceid) {
		ros::NodeHandle nodeHandle;

		const double speed = 100.0;

		// Getting MoveToRelativePoint Services.
		ros::ServiceClient moveToRelativePointClient = nodeHandle.serviceClient<deltaRobotNode::MoveToRelativePoint>(DeltaRobotNodeServices::MOVE_TO_RELATIVE_POINT);
		deltaRobotNode::MoveToRelativePoint moveToRelativePointService;

		moveToRelativePointService.request.motion.x = coordinates[0].second;
		moveToRelativePointService.request.motion.y = coordinates[1].second;
		moveToRelativePointService.request.motion.z = coordinates[2].second;
		moveToRelativePointService.request.motion.speed = speed;
		moveToRelativePointClient.call(moveToRelativePointService);

		// Get item from environment database. Example query:
		//  SELECT * FROM Items  WHERE workspaceID = workspaceid AND resourceID = resourceid AND (SELECT id FROM Items) = parentid;
	}
}

int main(int argc, char **argv) {
	// Ros init.
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle;

	// Getting Calibrate Services.
	ros::ServiceClient calibrateClient = nodeHandle.serviceClient<deltaRobotNode::Calibrate>(DeltaRobotNodeServices::CALIBRATE);
	deltaRobotNode::Calibrate calibrateService;
	calibrateClient.call(calibrateService);

	PickAndPlaceNamespace::PickAndPlace pickandplace;

	pair<char, int> m[3] = {
		pair<char, int>('x', 1), 
		pair<char, int>('y', 2), 
		pair<char, int>('z', 3)
	};

	pickandplace.moveToPoint("moveTo", m, 1, 1, 2);

	return 0;
}
