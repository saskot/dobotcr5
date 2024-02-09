#include <iostream>
#include <cstring>
#include "sensor3d.h"
#include "sensor3ddefines.h"
#include "ros/ros.h"


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

				ROI roi;
				int number_of_points = 0;
				//Acquire Pointclouds
				for (int i = 0; i < nrScans; i++) {
					
					result = Sensor3D_GetPointCloud(sensorHandle, &scanBuffer, pc_size, &number_of_points, &roi, 3000);
					if (result != SENSOR3D_OK)
					{
						std::cerr << "Error, could not read Pointcloud, result = " << result << std::endl;
					}
					else
					{
						std::cout <<"Point cloud read, amount of returned points: "<< number_of_points <<std::endl;
					}
				}
				std::cout << nrScans << " Scans Acquired!" << std::endl;
				cleanupAndDisconnect(sensorHandle, scanBuffer);

			}
		}
	}
	return 0;
}