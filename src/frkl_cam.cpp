#include <string>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <vector>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace {
const char* about = "Pose estimation of ArUco marker images";
const char* keys  =
	"{d        |11    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
    "DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
    "DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
    "DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
    "DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
    "{l        |0.05  | Actual marker length in meter }"
	"{ref      |0     | Ids of markers of reference, otherwise id '0'  }"
  /*"{v        |0     | Custom video source, otherwise '0' }"
	"{cd       |<none>| Camera data calibration file }"
	"{ch       |1080  | Camera capture resolution height }"
	"{cw  	   |1920  | Camera capture resolution width }"  <- If we use real camera*/
	"{h 	   |1080  | Screen resolution height }"
	"{w 	   |1920  | Screen resolution width }"
    ;
}

ros::Publisher markers_pub;
cv::Mat image

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try{
		image = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char** argv) {

//////////////////////////////////

	ros::init(argc, argv, "frkl_cam");

	markers_pub = n.advertise<std::vector<double>>("markers_pos", 1);

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("images", 1, imageCallback);

//////////////////////////////////

	cv::CommandLineParser parser(argc, argv, keys);
	parser.about(about);

	parser.printMessage();
	
	/*Dictionary of Aruco markers*/
	int dictionaryId = parser.get<int>("d");
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	/*Marker length*/
	float marker_length_m = parser.get<float>("l");
	if (marker_length_m <= 0) {
		std::cerr << "marker length must be a positive value in meter" << std::endl;
        return 1;
	}

	/*Ids of markers of reference*/
	std::vector<int> ids_ref;
	std::string str=parser.get<std::string>("ref");
	std::stringstream iss(str);
	int number;
	while ( iss >> number ){
		ids_ref.push_back(number);
	}
	
	std::vector<cv::Vec3d> tvec_inter_ref = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
	
	/* If we use real camera
	//Video source
	cv::String videoInput = "0";
	cv::VideoCapture in_video;
	if (parser.has("v")) {
        videoInput = parser.get<cv::String>("v");
        if (videoInput.empty()) {
			parser.printMessage();
			return 1;
        }
        char* end = nullptr;
       	int source = static_cast<int>(std::strtol(videoInput.c_str(), &end,10));
        if (!end || end == videoInput.c_str()) {
			in_video.open(videoInput); // url
        } 
		else {
			in_video.open(source); // id
        }
	}
	else {
        in_video.open(0);
   	}
	if (!in_video.isOpened()) {
        std::cerr << "failed to open video input: " << videoInput << std::endl;
        return 1;
    }	

	//Camera data calibration file
   	cv::Mat camera_matrix, dist_coeffs;
	cv::FileStorage fs(parser.get<cv::String>("cd"), cv::FileStorage::READ);
	fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

	//Camera capture resolution
	in_video.set(cv::CAP_PROP_FRAME_WIDTH,parser.get<int>("cw"));
	in_video.set(cv::CAP_PROP_FRAME_HEIGHT,parser.get<int>("ch"));	


	if (!parser.check()) {
        parser.printErrors();
        return 1;
    }*/

	
    cv::Mat image_resized;
	std::ostringstream vector_to_marker;


	ros::Rate loop_rate(60); //60FPS

	while(1){
		ros::spinOnce();
		if(image != NULL){
			// Capture of the video and detection of markers
        	/*in_video.retrieve(image); <- If we use real camera*/
	
        	std::vector<int> ids;
        	std::vector<std::vector<cv::Point2f> > corners;
        	cv::aruco::detectMarkers(image, dictionary, corners, ids);

        	// if at least one marker detected
        	if (ids.size() > 0)
        	{
           		cv::aruco::drawDetectedMarkers(image, corners, ids);
            	std::vector<cv::Vec3d> rvecs, tvecs, rvecs_ref, tvecs_ref;
				std::vector<int> ids_ref_detected;
            	cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,camera_matrix, dist_coeffs, rvecs, tvecs);

				bool ref_detected = false;

	    		for(int i=0; i < ids.size(); i++){
					
					cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);

	    			if(std::find(ids_ref.begin(), ids_ref.end(), ids[i]) != ids_ref.end()){

						tvecs_ref.push_back(tvecs[i]);
						ids_ref_detected.push_back(ids[i]);

						ref_detected = true;
					}
	    		}
            
               	if(ref_detected){
					std::vector<double> ref = {0.,0.,0.};
				
					for(int i=0; i < ids_ref_detected.size(); i++){
						std::vector<int>::iterator it = std::find(ids_ref.begin(), ids_ref.end(), ids_ref_detected[i]);
						ref[0] += tvecs_ref[i](0) - tvec_inter_ref[it[0]](0);
						ref[1] += tvecs_ref[i](1) - tvec_inter_ref[it[0]](1);
						ref[2] += tvecs_ref[i](2) - tvec_inter_ref[it[0]](2);
					}
					ref[0]/ids_ref_detected.size();
					ref[1]/ids_ref_detected.size();
					ref[2]/ids_ref_detected.size();
				
					int tab = 10;
            		for(int i=0; i < ids.size(); i++){
						if(std::find(ids_ref.begin(), ids_ref.end(), ids[i]) == ids_ref.end()){

							//Display of coordinates of each markers
							cv::putText(image, std::to_string(ids[i]), cv::Point(tab, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 252, 124), 1, CV_8S);				

							vector_to_marker.str(std::string());
		        			vector_to_marker << std::setprecision(4) << "X: " << std::setw(8) << tvecs[i](1) - ref[1]; 
							cv::putText(image, vector_to_marker.str(), cv::Point(tab, 60), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 1, CV_8S);

               				vector_to_marker.str(std::string());
                			vector_to_marker << std::setprecision(4) << "Y: " << std::setw(8) << tvecs[i](0) - ref[0];
                			cv::putText(image, vector_to_marker.str(), cv::Point(tab, 90), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 1, CV_8S);

                			vector_to_marker.str(std::string());
                			vector_to_marker << std::setprecision(4) << "Z: " << std::setw(8) << tvecs[i](2) - ref[2];
                			cv::putText(image, vector_to_marker.str(), cv::Point(tab, 120), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 0, 0), 1, CV_8S);

							tab += 300;

							//Sending coordinates on a topic
							std::vector<double> marker_pos = {ids[i], tvecs[i](1) - ref[1], tvecs[i](0) - ref[0], tvecs[i](2) - ref[2]};
							markers_pub.publish(marker_pos);
						}
					}
            	}
				else{
					cv::putText(image, "Pas de reference detectee", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 252, 124), 1, CV_8S);
				}
        	}
			else{
				cv::putText(image, "Pas de marqueur detecte", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 252, 124), 1, CV_8S);
			}
			//Resise of the image
			cv::Size size(parser.get<int>("w"),parser.get<int>("h")); 
			cv::resize(image,image_resized,size);
			//Display image
        	imshow("Pose estimation", image_resized);
		}
		else{
			std::cout << "No image" << std::endl;
		}
		//Quit on "Echap"
        char key = (char)cv::waitKey(10);
        if (key == 27){
			break;
		}
		loop_rate.sleep();
	}

	return 0;
}
