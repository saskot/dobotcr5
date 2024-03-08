#include <iostream>
#include <cstring>
#include "sensor3d.h"
#include "sensor3ddefines.h"
#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

void cleanupAndDisconnect(void* &sensorHandle, POINT_CLOUD &scanBuffer) {
    delete[] scanBuffer.point;
    delete[] scanBuffer.intensity;
    Sensor3D_Disconnect(sensorHandle);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "wenglor_driver");
    ros::NodeHandle nh;


	std::string ip = "192.168.100.1";
	std::string port = "32001";
	char readBuffer[0xFFFF];
	int nrScans = 20;

	// pcl::PointCloud<pcl::PointXYZ> cloud; 

  	// cloud.width    = 5;
  	// cloud.height   = 1;
  	// cloud.is_dense = false;
  	// cloud.resize (cloud.width * cloud.height);

	// std::cout<<cloud.size()<<std::endl;

	// for (auto& point: cloud)
  	// {
   	//  	point.x = 1024 * rand () / (RAND_MAX + 1.0f); // point.x = wenglor_cloud.point[0].x
    // 	point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    // 	point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  	// }

	//If Ip is passed as argument
	if (argc == 2) {
		ip = std::string(argv[1]);
	}
	
	//Connect to Sensor
	std::cout << "Trying to connect to Sensor: " << ip << std::endl;
	void* sensorHandle = Sensor3D_Connect(ip.c_str(), 5000);

	//Check if connection was successfull
	if (sensorHandle)
	{
		int sensorHandlestatus = 0;
		std::string command;
		//Read out sensor status
		if (Sensor3D_GetSensorStatus(sensorHandle, &sensorHandlestatus) == SENSOR3D_OK)
		{
			if (sensorHandlestatus & SENSOR_CONNECTED)
			{
				std::cout << "Connection established!"<< std::endl;

				int camera_width;
				int camera_height;
				POINT_CLOUD scanBuffer;

                //Read Dll Version Number
                int result = Sensor3D_GetVersion(readBuffer, sizeof(readBuffer));
                std::string version(readBuffer);
                std::cout<<"version: "<<readBuffer<<std::endl;
				
				//Read out number of pixels
                result = Sensor3D_ReadData(sensorHandle, "GetPixelXMax", readBuffer, sizeof(readBuffer), 1000);
				if (result != SENSOR3D_OK)
				{
					std::cerr << "Error, could not read GetPixelXMax, result = " << result << std::endl;
					cleanupAndDisconnect(sensorHandle, scanBuffer);
					return -1;
				}
				else
				{
					camera_width = std::stoi(std::string(readBuffer));
				}
				memset(readBuffer, 0x00, sizeof(readBuffer));
				result = Sensor3D_ReadData(sensorHandle, "GetPixelYMax", readBuffer, sizeof(readBuffer), 1000);
				if (result != SENSOR3D_OK)
				{
					std::cerr << "Error, could not read GetPixelYMax, result = " << result << std::endl;
					cleanupAndDisconnect(sensorHandle, scanBuffer);
					return -1;
				}
				else
				{
					camera_height = std::stoi(std::string(readBuffer));
				}

				//Initialize Buffers
				int nrPixels = camera_width * camera_height;
				scanBuffer.point = new POINT3D[nrPixels];
				scanBuffer.intensity = new unsigned short[nrPixels];
				int pc_size = (sizeof(POINT3D) + sizeof(unsigned short)) * nrPixels;


				//Set SensorMode to Pointcloud 
				command = "SetSensorMode=4\r";
				result = Sensor3D_WriteData(sensorHandle, command.data());
				if (result != SENSOR3D_OK)
				{
					std::cerr << "Error, could not write " << command << ", result = " << result << std::endl;
					cleanupAndDisconnect(sensorHandle, scanBuffer);
					return -1;
				}

				//Set TriggerSource To Software Trigger
				command = "SetTriggerSource=0\r";
				result = Sensor3D_WriteData(sensorHandle, command.data());
				if (result != SENSOR3D_OK)
				{
					std::cerr << "Error, could not write " << command << ", result = " << result << std::endl;
					cleanupAndDisconnect(sensorHandle, scanBuffer);
					return -1;
				}

				//Set Pattern to 28Patterns (highest Precision)
				command = "SetLEDPattern=28\r";
				result = Sensor3D_WriteData(sensorHandle, command.data());
				if (result != SENSOR3D_OK)
				{
					std::cerr << "Error, could not write " << command << ", result = " << result << std::endl;
					cleanupAndDisconnect(sensorHandle, scanBuffer);
					return -1;
				}


				//Start Acquisition
				command = "SetAcquisitionStart\r";
				result = Sensor3D_WriteData(sensorHandle, command.data());
				if (result != SENSOR3D_OK)
				{
					std::cerr << "Error, could not write " << command << ", result = " << result << std::endl;
					cleanupAndDisconnect(sensorHandle, scanBuffer);
					return -1;
				}
				



				//toto som si setol gain
				command = "SetGain=50\r";
				result = Sensor3D_WriteData(sensorHandle, command.data());
				if (result != SENSOR3D_OK)
				{
					std::cerr << "Error, could not write " << command << ", result = " << result << std::endl;
					cleanupAndDisconnect(sensorHandle, scanBuffer);
					return -1;
				}


				//toto som si setol led power
				command = "SetGain=60\r";
				result = Sensor3D_WriteData(sensorHandle, command.data());
				if (result != SENSOR3D_OK)
				{
					std::cerr << "Error, could not write " << command << ", result = " << result << std::endl;
					cleanupAndDisconnect(sensorHandle, scanBuffer);
					return -1;
				}





				ROI roi;
				int number_of_points = 0;
				//Acquire Pointclouds
				nrScans = 1;
				for (int i = 0; i < nrScans; i++) {
					
					result = Sensor3D_GetPointCloud(sensorHandle, &scanBuffer, pc_size, &number_of_points, &roi, 3000);
					if (result != SENSOR3D_OK)
					{
						std::cerr << "Error, could not read Pointcloud, result = " << result << std::endl;
					}
					else
					{
						std::cout <<"Point cloud read, amount of returned points: "<< number_of_points <<std::endl;
                        
						pcl::PointCloud<pcl::PointXYZ> cloud;
						
						// cloud.width    = 2048;
  						// cloud.height   = 2448;
  						// cloud.is_dense = false;
  						// cloud.resize (cloud.width * cloud.height);                        
						
						int nrPixels = camera_width * camera_height;
   						cloud.width = camera_width;
    					cloud.height = camera_height;
    					cloud.is_dense = false;
    					cloud.resize(nrPixels);
						
						
						//for (int i = 0; i <= number_of_points; i++){
							
						// int pointIndex = 0;
    					// for (int i = 0; i < camera_height; ++i) {
        				// 	for (int j = 0; j < camera_width; ++j) {
            			// 	// Assuming scanBuffer contains the point cloud data
            			// 	cloud.points[pointIndex].x = scanBuffer.point[pointIndex].x;
            			// 	cloud.points[pointIndex].y = scanBuffer.point[pointIndex].y;
						// 	std::cout <<"2 "<< number_of_points <<std::endl;
            			// 	cloud.points[pointIndex].z = scanBuffer.point[pointIndex].z;
						// 	std::cout <<"1 "<< number_of_points <<std::endl;
            			// 	++pointIndex;
        				// 		}
   						// 	}
							
						int pointIndex = 0;
						for(int i = 0; i < camera_height * camera_width; i++){
            				// Assuming scanBuffer contains the point cloud data
            				cloud.points[i].x = scanBuffer.point[i].x;
            				cloud.points[i].y = scanBuffer.point[i].y;
							//std::cout <<"2 "<<std::endl;
            				cloud.points[i].z = scanBuffer.point[i].z;
							//std::cout <<"1 "<<std::endl;
            				
        						
   							}
														
							
							
							
							
							//if (scanBuffer.point[i].x > 0)
							//{
							//float cloud;

							//cloud[i].x = scanBuffer.point[i].x;	 
							//cloud[i].y = scanBuffer.point[i].y;
							//cloud[i].z = scanBuffer.point[i].z;


								//std::cout << scanBuffer.point[i].x << std::endl;
								//break;
								// for (auto& point: cloud){
								// point.x = scanBuffer.point[i].x;
								// std::cout <<"tusi "<<std::endl;
								// point.y = scanBuffer.point[i].y;
								// std::cout <<"tusi2 "<<std::endl;
								// point.z = scanBuffer.point[i].z;
								// std::cout <<"tusi3 "<<std::endl;
								// //cloud.push_back(point);
							//}
						std::cout <<"3 "<< number_of_points <<std::endl;	
						pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
						std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;
						//for (const auto& point: cloud)
						//std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
                            //}
						std::cout << nrScans << " Scans Acquired!" << std::endl;
						cleanupAndDisconnect(sensorHandle, scanBuffer);


					
					}
				}
				
			}
		}
	}
	return 0;
}
