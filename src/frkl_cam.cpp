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

cv::Mat rtvecToRtmat(cv::Mat rmat, cv::Vec3d tvec){
	cv::Mat tmat(3,1,CV_64FC1,tvec);
	cv::Mat trmat;
	double fllist[4] = {0.0, 0.0, 0.0, 1.0};
	cv::Mat flmat(1, 4, CV_64FC1, fllist);
	tmat.at<double>(0,0) = tvec[0];
	tmat.at<double>(1,0) = tvec[1];
	tmat.at<double>(2,0) = tvec[2];
	cv::hconcat(rmat, tmat, trmat);
	cv::vconcat(trmat, flmat, trmat);
	return trmat;
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
	image_transport::Subscriber sub = it.subscribe("~/camera/link/camera/image", 1, imageCallback);

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
       	 	if (ids.size() > 0){
           		cv::aruco::drawDetectedMarkers(image, corners, ids);
				cv::Mat rmat;
            	std::vector<cv::Vec3d> rvecs, tvecs;
				std::vector<cv::Mat> rmats, rtmats;
				std::vector<int> ids_ref_detected;
            	cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,camera_matrix, dist_coeffs, rvecs, tvecs);

				bool ref_detected = false;

	    		for(int i=0; i < ids.size(); i++){
					cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
					cv::Rodrigues(rvecs[i], rmat);
					rmats.push_back(rmat);
					rtmats.push_back(rtvecToRtmat(rmat,tvecs[i]));
	    			if(std::find(ids_ref.begin(), ids_ref.end(), ids[i]) != ids_ref.end()){
						ids_ref_detected.push_back(i);
						ref_detected = true;
					}
	    		}
            
            	if(ref_detected){				
					int tab = 10;
            		for(int i=0; i < ids.size(); i++){
						if(std::find(ids_ref.begin(), ids_ref.end(), ids[i]) == ids_ref.end()){

							//Position of the robot in the coordinate system of the mobile marker
							cv::Mat pose_mob (4,1,CV_64FC1);
							std::vector<double> pose_robot = {0.0,0.0,0.0};
							pose_mob.at<double>(0,0) = pose_robot[0];
							pose_mob.at<double>(1,0) = pose_robot[1];
							pose_mob.at<double>(2,0) = pose_robot[2];
							pose_mob.at<double>(3,0) = 1.0;
	
							//Calcul the position of the robot in the coordinate system of the marker of reference
							cv::Mat pose_ref = rtmats[ids_ref_detected[0]].inv() * rtmats[i] * pose_mob;
							double tx = pose_ref.at<double>(0,0);
							double ty = pose_ref.at<double>(1,0);
							double tz = pose_ref.at<double>(2,0);

							//Rotation of the robot in the coordinate system of the mobile marker
							cv::Mat rot_mob (3,1,CV_64FC1);
							std::vector<double> rot_robot = {0.0,0.0,0.0};
							rot_mob.at<double>(0,0) = rot_robot[0];
							rot_mob.at<double>(1,0) = rot_robot[1];
							rot_mob.at<double>(2,0) = rot_robot[2];

							//Calcul the rotation of the robot in the coordinate system of the marker of reference
							cv::Mat rot_ref = rmats[ids_ref_detected[0]].inv() * rmats[i] * rot_mob;
							double rx = pose_ref.at<double>(0,0);
							double ry = pose_ref.at<double>(1,0);
							double rz = pose_ref.at<double>(2,0);

							//Display translation and rotation on screen
							cv::putText(image, "Translation ID "+std::to_string(ids[i]), cv::Point(tab, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 252, 124), 1, CV_8S);
							cv::putText(image, "X = " + std::to_string(tx), cv::Point(tab, 60), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 1, CV_8S);
                			cv::putText(image, "Y = " + std::to_string(ty), cv::Point(tab, 90), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 1, CV_8S);
                			cv::putText(image, "Z = " + std::to_string(tz), cv::Point(tab, 120), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 0, 0), 1, CV_8S);
							cv::putText(image, "Rotation", cv::Point(tab, 150), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 252, 124), 1, CV_8S);
							cv::putText(image, "X = " + std::to_string(rx), cv::Point(tab, 180), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 1, CV_8S);
                			cv::putText(image, "Y = " + std::to_string(ry), cv::Point(tab, 210), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 1, CV_8S);
                			cv::putText(image, "Z = " + std::to_string(rz), cv::Point(tab, 240), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 0, 0), 1, CV_8S);

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
