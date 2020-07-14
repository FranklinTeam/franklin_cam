#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

namespace {
const char* about = "Pose estimation of ArUco marker images";
const char* keys  =
	"{d        |11    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
	"DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
	"DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
	"DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
	"DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
	"{l        |0.05  | Actual marker length in meter }"
	"{v        |0     | Custom video source, otherwise '0' }"
	"{ref      |0	  | Ids of markers of reference, otherwise id '0'  }"
	"{orig     |      | Position of the origin to the different marker of reference  }"
	"{cd       |./out_camera_data.xml| Camera data calibration file }"
	"{ch       |2160  | Camera capture resolution height }"
	"{cw  	   |3840  | Camera capture resolution width }"
	"{h 	   |1080  | Screen resolution height }"
	"{w 	   |1920  | Screen resolution width }"
        ;
}

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

int main(int argc, char **argv){

	cv::CommandLineParser parser(argc, argv, keys);
	parser.about(about);
	
	/*Dictionary of Aruco markers*/
	int dictionaryId = parser.get<int>("d");
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	/*Marker length*/
	float marker_length_m = parser.get<float>("l");
	if (marker_length_m <= 0) {
		std::cerr << "marker length must be a positive value in meter" << std::endl;
        return 1;
	}
	
	/*Video source*/
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


	/*Ids of markers of reference*/
	std::vector<int> ids_ref;
	std::string str=parser.get<std::string>("ref");
	std::stringstream iss(str);
	int number;
	while ( iss >> number ){
		ids_ref.push_back(number);
	}

	/*Position of the origin to the different marker of reference*/
	std::vector<cv::Vec3d> pos_ref_origin;
	cv::String orig;
	if(parser.has("orig")){
		orig = parser.get<cv::String>("orig");
	}
	else{
		orig = "{{0.0,0.0,0.0}}";
	}
	std::vector<cv::String> orig_list; 
    boost::split(orig_list, orig, boost::is_any_of(",")); 
    for (int i = 0; i < orig_list.size(); i+=3){
        boost::erase_all(orig_list[i], "{");
		boost::erase_all(orig_list[i], "}");
		double px = ::atof(orig_list[i+0].c_str());
		double py = ::atof(orig_list[i+1].c_str());
		double pz = ::atof(orig_list[i+2].c_str());
		cv::Vec3d point = {px,py,pz};
		pos_ref_origin.push_back(point);
	} 
	
	/*Camera data calibration file*/
   	cv::Mat camera_matrix, dist_coeffs;
	cv::FileStorage fs(parser.get<cv::String>("cd"), cv::FileStorage::READ);
	fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

	/*Camera capture resolution*/
	in_video.set(cv::CAP_PROP_FRAME_WIDTH,parser.get<int>("cw"));
	in_video.set(cv::CAP_PROP_FRAME_HEIGHT,parser.get<int>("ch"));	


	if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

	
    cv::Mat image, image_resized;
	std::ostringstream vector_to_marker;

    while (in_video.grab()){
		// capture of the video and detection of markers
        in_video.retrieve(image);
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

	    	for(int i=0; i < ids.size(); i++){
				cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
				rtmats.push_back(rtvecToRtmat(rvecs[i],tvecs[i]));
	    		if(std::find(ids_ref.begin(), ids_ref.end(), ids[i]) != ids_ref.end()){
					ids_ref_detected.push_back(i);
				}
	    	}
            
            if(ids_ref_detected.size() > 0){				
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
						double tx = 0.0;
						double ty = 0.0;
						double tz = 0.0;
						for(int j=0; j < ids_ref_detected.size(); j++){
							cv::Mat pose_ref = rtmats[ids_ref_detected[j]].inv() * rtmats[i] * pose_mob;
							std::vector<int>::iterator it = std::find(ids_ref.begin(), ids_ref.end(), ids[ids_ref_detected[j]]);
							int index = std::distance(ids_ref.begin(), it);
							tx = tx + pose_ref.at<double>(0,0) + pos_ref_origin[index](0);
							ty = ty + pose_ref.at<double>(1,0) + pos_ref_origin[index](1);
							tz = tz + pose_ref.at<double>(2,0) + pos_ref_origin[index](2);
						}
						tx = tx/ids_ref_detected.size();
						ty = ty/ids_ref_detected.size();
						tz = tz/ids_ref_detected.size();

						//Display translation and rotation on screen
						cv::putText(image, "ID "+std::to_string(ids[i]), cv::Point(tab, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 252, 124), 1, CV_8S);
						cv::putText(image, "Translation", cv::Point(tab, 60), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 252, 124), 1, CV_8S);
						cv::putText(image, "X = " + std::to_string(tx).substr(0, 7) + " m", cv::Point(tab, 90), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 1, CV_8S);
                		cv::putText(image, "Y = " + std::to_string(ty).substr(0, 7) + " m", cv::Point(tab, 120), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 1, CV_8S);
                		cv::putText(image, "Z = " + std::to_string(tz).substr(0, 7) + " m", cv::Point(tab, 150), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 0, 0), 1, CV_8S);

						tab += 300;
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
		cv::Size size(parser.get<int>("w"),parser.get<int>("h"));
		cv::resize(image,image_resized,size);
        imshow("Pose estimation", image_resized);
        char key = (char)cv::waitKey(10);
        if (key == 27){
			break;
		}
    }

	in_video.release();
	return 0;
}
