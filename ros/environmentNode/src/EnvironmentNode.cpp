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

#include <sstream>
#include <environmentNode/EnvironmentNode.h>
#include <environmentNode/KeyValue.h>

/**
 * The service that gets specified resources from the available resources database
 * @param req The request object
 * @param res The response object
 **/
bool EnvironmentNode::getWorkspaceItems(environmentNode::GetWorkspaceItems::Request &req, environmentNode::GetWorkspaceItems::Response &res) {
	return true;
}

/**
 * Update an item in the available resources database
 **/
bool EnvironmentNode::updateWorkspaceItem(environmentNode::UpdateWorkspaceItem::Request &req, environmentNode::UpdateWorkspaceItem::Response &res) {
	std::ostringstream os(std::ostringstream::out);
	std::vector<environmentNode::KeyValue> properties(req.item.properties);
	/* Create the JSON string */
	os << "{";
	for(int i = 0; i < properties.size(); i++)
	{
		os << "'" << properties[i].key << "'" << ":" << properties[i].value;
		if(i < (properties.size() - 1)){
			os << ",";
		}
	}
	os << "}";
	/* Create a bson object from the json string */
	mongo::BSONObj p = mongo::fromjson(os.str().c_str());
	/* Update the database */
	EnvironmentNode::clientConnection.update("REXOSV2.Item", QUERY("resource_type" << 2), BSON("$set" << p), false, true);
	std::string error = clientConnection.getLastError();
	res.success = error.empty();
	return res.success;
}



/**
 * The constructor of the equipletNode
 * @param eq The id of the Equiplet this EnvironmentNode belongs to. If not provided, assumes equiplet id is 0
 **/
EnvironmentNode::EnvironmentNode(int eq): equipletId(eq) {
	// Create the topic name that receives resouce updates. It exists of the string resourceUpdate and the equipletId attached to it
	std::ostringstream os(std::ostringstream::out);
	os << "itemUpdate" << equipletId;
	
	// Initialize subscribers, publishers and services
	ros::NodeHandle nh;
	getWorkspaceItemsService = nh.advertiseService("getWorkspaceItems", &EnvironmentNode::getWorkspaceItems, this);
	updateWorkspaceItemService = nh.advertiseService("updateWorkspaceItem", &EnvironmentNode::updateWorkspaceItem, this);

	// Connect to the workspace database
	try{
		clientConnection.connect("localhost");
		std::cout << "Connected to database" << std::endl;
	} catch( const mongo::DBException &e ) {
		std::cout << "caught " << e.what() << std::endl;
	}
}




