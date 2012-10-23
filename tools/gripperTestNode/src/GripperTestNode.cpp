#include <iostream>
#include <Gripper.h>
#include <GripperTestNode.h>


#define NODE_NAME "GripperripperTestNode"

GripperTestNode::GripperTestNode(int equipletID, int moduleID): rosMast::StateMachine(equipletID, moduleID)
{
	gripper = new Gripper(&error);
	ros::NodeHandle nodeHandle;
	// Advertise the services
	gripService = nodeHandle.advertiseService(GripperTestNodeServices::GRIP, &GripperTestNode::grip, this);
	releaseService = nodeHandle.advertiseService(GripperTestNodeServices::RELEASE, &GripperTestNode::release, this);
}

GripperTestNode::~GripperTestNode()
{
	delete gripper;
}

void GripperTestNode::error(){
	std::cout << "Gripper handler warning!" << std::endl;
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
int GripperTestNode::transitionSetup() {
	ROS_INFO("Setup transition called");
	setState(rosMast::setup);
   
	return 0; 
}

/**
 * Transition from Standby to Safe state
 * Will turn power off the motor 
 * @return will be 0 if everything went ok else error
 **/
int GripperTestNode::transitionShutdown() {	
	ROS_INFO("Shutdown transition called");	
	gripper->release();
	setState(rosMast::shutdown);
	return 0;
}

/**
 * Transition from Standby to Normal state
 * @return will be 0 if everything went ok else error 
 **/
int GripperTestNode::transitionStart() {
	ROS_INFO("Start transition called");   

	// Set currentState to start
	setState(rosMast::start);	
	return 0;
}
/**
 * Transition from Normal to Standby state
 * @return will be 0 if everything went ok else error
 **/
int GripperTestNode::transitionStop() {
	ROS_INFO("Stop transition called");
	
	// Set currentState to stop
	setState(rosMast::stop);
	return 0;
}

bool GripperTestNode::grip(gripperTestNode::Grip::Request &req, gripperTestNode::Grip::Response &res)
{
	res.succeeded = false;
	if(getState() == rosMast::normal)
	{
		gripper->grab();
		res.succeeded = true;
	}
	else
	{
		res.succeeded = false;
	}
	return res.succeeded;

}	

bool GripperTestNode::release(gripperTestNode::Release::Request &req, gripperTestNode::Release::Response &res)
{
	res.succeeded = false;
	if(getState() == rosMast::normal)
	{
		gripper->release();
		res.succeeded = true;
	}
	else
	{
		res.succeeded = false;
	}
	return res.succeeded;

}



int main(int argc, char** argv){
	
	ros::init(argc, argv, NODE_NAME);

	int equipletID = atoi(argv[1]);
	int moduleID = atoi(argv[2]);

	ROS_INFO("Creating GripperTestNode"); 	

	GripperTestNode gripperTestNode(equipletID, moduleID);    

	ROS_INFO("GripperTestNode ready..."); 	
	gripperTestNode.StateEngine();


	return 0;
}