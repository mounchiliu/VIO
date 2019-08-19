 
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "../../dataset/";
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;

void PubImuData()  //Get IMU data
{
	//string sImu_data_file = sData_path + "imu_pose.txt";
	string sImu_data_file = sData_path + "imu_pose_noise.txt";
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
	//pose
	Vector4d q;
	Vector3d t;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec >> q.w() >> q.x() >> q.y() >> q.z() >>
                t.x() >> t.y() >> t.x() >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec /*/ 1e9*/, vGyr, vAcc); //Modified
		usleep(5000*nDelayTimes);
	}
	fsImu.close();
}

void PubImageData() //Get Image data
{
	string sImage_file = sData_path + "keyframe/";
	string sTimeStep_file = sImage_file + "CamTimeStep.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsTimeStep; //modified
	fsTimeStep.open(sTimeStep_file.c_str());//Added

	if (!fsTimeStep.is_open())
	{
		cerr << "Failed to open TimeStep file! " << sTimeStep_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	double dStampNSec_pre = 0;
	std::vector<Vector2d> feature_positions_pre;
	string sImgFileName;
	double fx = 460.0; double fy = 460.0; double cx = 320.0; double cy = 320.0;
	
	
	// cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
	while (std::getline(fsTimeStep, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;//Cam timestep for each keyframe & 
							//filename of saving the feature points in that KF
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + sImgFileName;
		
		//-----------------------------------------------------------------
		//Get feature points information
		ifstream fsFeatures;
 		fsFeatures.open(imagePath.c_str());
		
		if(!fsFeatures.is_open())
		{
		cerr << "Failed to open TimeStep file! " << sTimeStep_file << endl;
		return;
		}

		std::string sFeature_line;

		//Get all the feature points information in this KF
		//std::vector<Vector3d> landmarks;
		std::vector<Vector2d> feature_positions_un,feature_positions; 
		//For HW2, it gets the position on the normalized plane
		std::vector<Vector2d> feature_velocities;
		std::vector<int> ids;
		int count = 0;
		double dt = dStampNSec - dStampNSec_pre;
		
		while (std::getline(fsFeatures, sFeature_line) && !sFeature_line.empty())
		{
			Vector4d landmark;
			Vector2d feature_position;
			Vector2d feature_velocity;

			std::istringstream ssFeatureData(sFeature_line);
			ssFeatureData >> landmark.x() >> landmark.y() >> landmark.z() >> landmark.w() >> feature_position.x() >> feature_position.y();




			//Record
			//landmarks.push_back(landmark); //Pixel coordinate
			feature_positions_un.push_back(feature_position);
			feature_position.x() = feature_position.x()*fx + cx;
			feature_position.y() = feature_position.y()*fy + cy;
			feature_positions.push_back(feature_position);

			//This dataset has aligned the feature points
			//Calculate the velocity //cur - pre; //in pixel coordinate
			if(dt==0){//initialization
				feature_velocity.x() = 0;
				feature_velocity.y() = 0;	
			}
			else{
			feature_velocity.x() = (feature_position.x() - feature_positions_pre[count].x())/(dt); 
			feature_velocity.y() = (feature_position.y() - feature_positions_pre[count].y())/(dt);
			}
			feature_velocities.push_back(feature_velocity);
			
			ids.push_back(count);	

			count++;
	

		}
		
		feature_positions_pre = feature_positions;
		dStampNSec_pre = dStampNSec;

		fsFeatures.close();

		
		pSystem->PubFeatureData(dStampNSec /* 1e9*/, ids, /*landmarks,*/ 
feature_positions_un, feature_positions, feature_velocities);

		// cv::imshow("SOURCE IMAGE", img);
		// cv::waitKey(0);
		usleep(50000*nDelayTimes);
	}
	fsTimeStep.close();
}

int main(int argc, char **argv)
{
//	if(argc != 3)
//	{
//		cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n" 
//			<< "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/"<< endl;
//		return -1;
//	}

//	sData_path = argv[1];
//	sConfig_path = argv[2];

	pSystem.reset(new System(sConfig_path));

	
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);
		
	sleep(5);
	std::thread thd_PubImuData(PubImuData);

	std::thread thd_PubImageData(PubImageData);
	
	std::thread thd_Draw(&System::Draw, pSystem);
	
	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
