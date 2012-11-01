/**
 * @file EnvironmentNode.cpp
 * @brief Environment Node
 * @date Created: 2012-10-29
 *
 * @author Dennis Koole
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

#include <environmentNode/EnvironmentNode.h>
#include <sstream>

/**
 * The constructor of the equipletNode
 * @param eq The id of the Equiplet this EnvironmentNode belongs to. If not provided, assumes equiplet id is 0
 **/
EnvironmentNode::EnvironmentNode(int eq): equipletId(eq) {
	// Create the topic name that receives resouce updates. It exists of the string resourceUpdate and the equipletId attached to it
	ostringstream os(ostringstream::out);
	os << "resourceUpdate" << equipletId;
	// Initialize resource update subscriber
	ros::NodeHandle nh;
	resourceUpdateSubscriber = nh.subscribe<environmentNode::ResourceUpdate>(os.str().c_str(), 1000, &EnvironmentNode::updateResource, &this);
}

/**
 * Update a resource in the available resources database
 **/
EnvironmentNode::updateResource(environmentNode::ResourceUpdate &update) {

}