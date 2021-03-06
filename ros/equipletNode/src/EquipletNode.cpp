/**
 * @file EquipletNode.cpp
 * @brief Symbolizes an entire EquipletNode.
 * @date Created: 2012-10-12
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

#include <EquipletNode/EquipletNode.h>
#include <sstream>
#include <cstdio>
#include <unistd.h>
#include <algorithm>

/**
 * Create a new EquipletNode
 * @param id The unique identifier of the Equiplet
 **/
EquipletNode::EquipletNode(int id): equipletId(id), moduleTable() {
	// Create the map with moduleType mapped to package name and node name
	modulePackageNodeMap = std::map< int, std::pair<std::string, std::string> >();
	modulePackageNodeMap[1] = std::pair< std::string, std::string > ("deltaRobotNode", "DeltaRobotNode");
	modulePackageNodeMap[2] = std::pair< std::string, std::string > ("gripperTestNode", "GripperTestNode");

	ros::NodeHandle nodeHandle;
	std::stringstream stringStream;
	stringStream << equipletId;
	std::string str = stringStream.str();
	moduleErrorService = nodeHandle.advertiseService("ModuleError_" + str, &EquipletNode::moduleError, this); 
	stateUpdateService = nodeHandle.advertiseService("StateUpdate_" + str, &EquipletNode::stateChanged, this);
}; 

/**
 * Callback function that is called when a message is received on the equiplet_statechanged topic
 * It updates the state of a hardware module.
 * 
 * @param request Contains the data required for a state transition
 * @param response Says if update was succesfull
 **/
bool EquipletNode::stateChanged(rosMast::StateUpdate::Request &request, rosMast::StateUpdate::Response &response) {
	ROS_INFO("State changed message received");
	if(updateModuleState(request.state.moduleID, rosMast::StateType(request.state.newState))) {
		//std::cout << "The state of module " << msg->moduleID << " has been changed to " << rosMast::state_txt[msg->state] << std::endl; 
		response.succeeded = true;
	} else{
		//std::cerr << "Cannot update the state of the module " << msg->moduleID << " run for your life!" << std::endl;
		response.succeeded = false;
	}	
	return true;
}

/**
 * Callback for when a error occurs in a module
 *
 * @param request Contains the errorCode and the ID of the module were the error occured
 * @param response Will contain the new state after error occured
 **/
bool EquipletNode::moduleError(rosMast::ErrorInModule::Request &request, rosMast::ErrorInModule::Response &response) {
	int moduleID = request.moduleError.moduleID;
	ROS_INFO("Error message received from module %d", moduleID);
	//int errorCode = msg->errorCode;

	// TODO: Lookup errorcode in the DB and decide accordingly
	
	// Lookup current state of the module
	rosMast::StateType currentModuleState = getModuleState(moduleID);

	// This will be changed to a proper way to decide what state should be entered on error
	response.state.moduleID = moduleID;
	response.state.newState = rosMast::StateType(currentModuleState - 3); 
	return true;
}

/**
 * Send a StateChange request to a specific module
 * 
 * @param moduleID the unique identifier for the module which state needs to change
 * @param newState the new state for the module
 **/
void EquipletNode::sendStateChangeRequest(int moduleID, rosMast::StateType newState) {
	rosMast::StateChange msg;	
	msg.request.desiredState = newState;

	ros::NodeHandle nodeHandle;
	std::stringstream stringStream;
	stringStream << equipletId + "_" << moduleID;
	std::string str = stringStream.str();
	ros::ServiceClient stateChangeRequestClient = nodeHandle.serviceClient<rosMast::StateChange>("RequestStateChange_" + str);
	
	stateChangeRequestClient.call(msg);
}

/**
 * Update the safetyState of the Equiplet
 **/
void EquipletNode::updateSafetyState() {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	rosMast::StateType newSafetyState = rosMast::safe;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++) {
		if((*it).actuator && (*it).currentState > newSafetyState) {
			newSafetyState = (*it).currentState;
		}
	}
	safetyState = newSafetyState;
}

/**
 * Update the operation state of the Equiplet
 **/
void EquipletNode::updateOperationState() {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	bool operationStateSet = false;
	// first set the operation state to the highest state possible 
	rosMast::StateType newOperationState = rosMast::normal;

	for(it = moduleTable.begin(); it < moduleTable.end(); it++) {
		 // Set the new operation state if the hardware module is an actor, required for
		 // the current service and if its state is lower than the new operation state as
		 // Initialized above.
		if((*it).actuator && (*it).needed) {
			newOperationState = std::min((*it).currentState, newOperationState);
			operationStateSet = true;
		}
	}
	// If the operation state is not set, it means that there are no actor modules suited
	// for the current service and so the operation state is equal to the lowest state possible
	// the safe state.
	if(!operationStateSet) {
		newOperationState = rosMast::safe;
	}
	operationState = newOperationState;
}

/**
 * Add a hardware module to the module table
 * 
 * @param module The hardware to add to the table
 *
 * @return true if the module has a unique id, otherwise false
 **/
bool EquipletNode::addHardwareModule(Mast::HardwareModuleProperties module) {
	// First check if the module already exists
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++) {
		if(module.id == (*it).id) {
			return false;
		}
	}

    // Create the string that is used to start another ROS node	 
	std::pair< std::string, std::string > packageNodeName = modulePackageNodeMap[module.type];
	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	ss << "rosrun " << packageNodeName.first << " " << packageNodeName.second << " " << equipletId << " " << module.id;
	std::cout << ss.str() << std::endl;
	int pid = -1;
	switch(pid = fork()) {
		case 0:
			fclose(stderr);
			fclose(stdout);
			fclose(stdin);
			execl("/bin/sh", "/bin/sh", "-c", ss.str().c_str(), NULL);
		case -1: 
			std::cerr << "Cannot start node for hardware module " << module.id << std::endl;
			return false;
		default:
			break; 
	}

	
	// Add the module to the table and update the safety state and operation state	
	moduleTable.push_back(module);
	updateSafetyState();
	updateOperationState(); 
	
	return true;
}

/**
 * Remove a hardware module from the module table
 *
 * @param id The identifier that is used to identify the hardware module that needs to be removed
 *
 * @return true if the hardware module is removed, false if the module could not be found in the table
 **/
bool EquipletNode::removeHardwareModule(int id) {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++) {
		if((*it).id == id) {
			moduleTable.erase(it);
			updateSafetyState();
			updateOperationState();
			return true;
		}
	}	
	return false;
}

/**
 * Print all hardware modules in the table
 **/
void EquipletNode::printHardwareModules() {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++) {
		std::cout << *it << std::endl;
	}
}

/**
 * Get the state of the module corresponding to the moduleID
 *
 * @param moduleID the unique identifier for a module
 *
 * @return the State of the module
 **/

rosMast::StateType EquipletNode::getModuleState(int moduleID) {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++) {
		if((*it).id == moduleID) {
			return (*it).currentState;
		}
	}
	return rosMast::nostate;
}

/**
 * Update the state of a module in the module table. Also automatically updates the operationState and
 * safe state of the Equiplet
 *
 * @param moduleID the id of the module
 * @param state the new state of the module 
 * 
 * @return true if the module is found and the state is updated, false if the module is not found in module table
 **/
bool EquipletNode::updateModuleState(int moduleID, rosMast::StateType state) {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++) {
		if((*it).id == moduleID) {
			(*it).currentState = state;
			updateSafetyState();
			updateOperationState();
			return true;
		}
	}
	return false;
}