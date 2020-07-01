#include <string>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <franklin/Camera.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

cv::Mat rtvecToRtmat(cv::Vec3d rvec, cv::Vec3d tvec){
	cv::Mat tmat(3,1,CV_64FC1,tvec);
	cv::Mat rmat, trmat;
	double fllist[4] = {0.0, 0.0, 0.0, 1.0};
	cv::Mat flmat(1, 4, CV_64FC1, fllist);
	tmat.at<double>(0,0) = tvec[0];
	tmat.at<double>(1,0) = tvec[1];
	tmat.at<double>(2,0) = tvec[2];
	cv::Rodrigues(rvec, rmat);
	cv::hconcat(rmat, tmat, trmat);
	cv::vconcat(trmat, flmat, trmat);
	return trmat;
}

ros::Publisher markers_pub;
cv::Mat image;

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

int main(int argc, char **argv){

	ros::init(argc, argv, "cam_relatif");
	ros::NodeHandle n("~");
	markers_pub = n.advertise<franklin::Camera>("markers_pos_relatif", 1);
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);
	
	/*Dictionary of Aruco markers*/
	int dictionaryId = 11;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	/*Marker length*/
	float marker_length_m= 0.05;

	/*Camera distortion*/
	int numberOfPixelInHeight,numberOfPixelInWidth;
 	double heightOfSensor, widthOfSensor;
   	double focalLength=0.5;
   	double mx, my, U0, V0;
   	numberOfPixelInHeight=600;
    numberOfPixelInWidth=600;
 
    heightOfSensor=10;
    widthOfSensor=10;
 
   	my=(numberOfPixelInHeight)/heightOfSensor ;
    U0=(numberOfPixelInHeight)/2 ;
 
    mx=(numberOfPixelInWidth)/widthOfSensor;
    V0=(numberOfPixelInWidth)/2;

	cv::Mat camera_matrix = (cv::Mat_<double>(3,3)<<focalLength*mx, 0, V0,0,focalLength*my,U0,0,0,1);
	cv::Mat dist_coeffs = (cv::Mat_<double>(5,1) <<0, 0, 0, 0, 0);

	////////////////////

	ros::Rate loop_rate(60); //60FPS
	cv::Mat image_resized;

    while (ros::ok()){
		ros::spinOnce();
		if(image.empty()==false){
        	std::vector<int> ids;
        	std::vector<std::vector<cv::Point2f> > corners;
        	cv::aruco::detectMarkers(image, dictionary, corners, ids);
        	// if at least one marker detected
        	if (ids.size() > 0){
           		cv::aruco::drawDetectedMarkers(image, corners, ids);
				cv::Mat rmat;
            	std::vector<cv::Vec3d> rvecs, tvecs;
				std::vector<cv::Mat> rmats, rtmats;
            	cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,camera_matrix, dist_coeffs, rvecs, tvecs);

	    		for(int i=0; i < ids.size(); i++){
					cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
					rtmats.push_back(rtvecToRtmat(rvecs[i],tvecs[i]));
	    		}
				int tab = 10;
            	for(int i=0; i < ids.size(); i++){
					//Position of the robot in the coordinate system of the mobile marker
					cv::Mat pose_mob (4,1,CV_64FC1);
					pose_mob.at<double>(0,0) = 0.0;
					pose_mob.at<double>(1,0) = 0.0;
					pose_mob.at<double>(2,0) = 0.0;
					pose_mob.at<double>(3,0) = 1.0;
	
					//Calcul the position of the robot in the coordinate system of the marker of reference
					double tx = 0.0;
					double ty = 0.0;
					double tz = 0.0;
					cv::Mat pose_ref = rtmats[i] * pose_mob;
					tx = pose_ref.at<double>(0,0);
					ty = pose_ref.at<double>(1,0);
					tz = pose_ref.at<double>(2,0);

					//Conversion double to string

					std::ostringstream osX;
					std::ostringstream osY;
					std::ostringstream osZ;
					osX << tx;
					osY << ty;
					osZ << tz;
					std::string texteX = osX.str();
					std::string texteY = osY.str();
					std::string texteZ = osZ.str();
					texteX = texteX.substr(0, 7);
					texteY = texteY.substr(0, 7);
					texteZ = texteZ.substr(0, 7);

					//Display translation and rotation on screen

					cv::putText(image, "ID" + ids[i] , cv::Point(tab, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 252, 124), 1, CV_8S);
					cv::putText(image, "Translation", cv::Point(tab, 60), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 252, 124), 1, CV_8S);
					cv::putText(image, texteX + " m", cv::Point(tab, 90), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 1, CV_8S);
                	cv::putText(image, texteY + " m", cv::Point(tab, 120), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 1, CV_8S);
                	cv::putText(image, texteZ + " m", cv::Point(tab, 150), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 0, 0), 1, CV_8S);

					tab += 300;

					//Sending coordinates on a topic
					franklin::Camera marker_pos;
					marker_pos.y=tx;
					marker_pos.x=ty;
					marker_pos.z=tz;
					markers_pub.publish(marker_pos);
				}

        	}
			else{
				cv::putText(image, "Pas de marqueur detecte", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 252, 124), 1, CV_8S);
			}
			cv::Size size(1920,1080);
			cv::resize(image,image_resized,size);
        	imshow("Pose estimation", image_resized);
		}
		else{
			std::cout << "No image" << std::endl;
		}

		char key = (char)cv::waitKey(10);
    	if (key == 27){
			break;
		}
		loop_rate.sleep();
    }
	return 0;
}
