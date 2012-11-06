#include "ImageTransformationNode.h"

#define NODE_NAME "ImageTransformationNode"

/**
 * @var WINDOW_NAME
 * Name for the opencv image window.
 **/
static const char WINDOW_NAME[] = "Image window";

/**
 * On mouse click event. 
 *
 * @param event CV_EVENT (eg. CV_EVENT_LBUTTONDOWN).
 * @param x X coordinate of the click.
 * @param y Y coordinate of the click.
 * @param flags CV_EVENT_FLAG.
 * @param param Output image to publish on topic.
 **/
void on_mouse(int event, int x, int y, int flags, void* param) {
	if (event == CV_EVENT_LBUTTONDOWN) {
		
	}
}

/**
 * Constructor 
 **/
ImageTransformationNode::ImageTransformationNode(int equipletID, int moduleID): rosMast::StateMachine(equipletID, moduleID) {
	ros::NodeHandle nodeHandle;
	// Advertise the services
	pub = it.advertise(ImageTransformationNodeTopics::TRANSFORMED_IMAGE, 1);

	// OpenCV GUI
	cv::namedWindow(WINDOW_NAME);
	cv::createTrackbar( "blockSize:", WINDOW_NAME, &blockSize, maximum );
	cv::createTrackbar( "subtract :", WINDOW_NAME, &subtract, maximum );

	cvSetMouseCallback(WINDOW_NAME, &on_mouse, outputImage);
}

/**
 * Callback function for the calibration procedure.
 * Receives an image of the image topic and tries to detect all fiducials that are present.
 **/
void CrateLocatorNode::transformCallback(const sensor_msgs::ImageConstPtr& msg) {
	// Receive image
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

/**
 * Transforms the image on the topic to the correct size and format and publishes to a new topic.
 **/
void ImageTransformationNode::transformCallback(const sensor_msgs::ImageConstPtr& msg) {
	// Receive image
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//TODO get these values from DotMatrixNodeSettings
	double scale = std::min(cv_ptr->image.rows / 350/*pixel height*/, cv_ptr->image.cols / 220/*pixel width*/);
	cv::Size outputSize = cv::Size(cv_ptr->image.rows / scale, cv_ptr->image.cols / scale);
	cv::resize(cv_ptr->image, outputImage, outputSize);

	cv::Mat grayImage;
	cv::cvtColor(outputImage, grayImage, CV_BGR2GRAY);
	
	//Threshold the image, note that blocksize has to be a multiple of 3 and >= 3.
	cv::Mat thresholdedImage;
	cv::adaptiveThreshold(grayImage, thresholdedImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, blockSize < 3 ? 3 : blockSize | 1, subtract);

	cv::imshow(WINDOW_NAME, thresholdedImage);
	cv::waitKey(3);
}

/**
 * Destructor
 **/
ImageTransformationNode::~ImageTransformationNode() {
}

/**
 * Blocking function that contains the main loop.
 * Spins in ROS to receive frames. These will execute the callbacks.
 * This function ends when ros receives a ^c
 **/
void ImageTransformationNode::run( ) {
	cameraSubscriber = imageTransport.subscribe("camera/image", 1, &CrateLocatorNode::transformCallback, this/*, image_transport::TransportHints("compressed")*/);

	while(ros::ok()) {
		ros::spinOnce();
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, NODE_NAME);
	int equipletID = 0;
	int moduleID = 0;
	if(argc < 3 || !(Utilities::stringToInt(equipletID, argv[1]) == 0 && Utilities::stringToInt(moduleID, argv[2]) == 0))
	{ 	 	
    	std::cerr << "Cannot read equiplet id and/or moduleId from commandline please use correct values." << std::endl;
 		return -1;
  	} 

	ImageTransformationNode ImageTransformationNode(equipletID, moduleID);    

	ImageTransformationNode.startStateMachine();
	return 0;
}