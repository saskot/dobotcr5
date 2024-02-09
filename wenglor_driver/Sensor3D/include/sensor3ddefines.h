#ifndef SENSOR3DDEFINES_H
#define SENSOR3DDEFINES_H

#include <string>

// Return codes

#define SENSOR3D_OK  0

#define SENSOR3D_INVALIDSENSORHANDLE -1
#define SENSOR3D_SENSORNOTCONNECTED -2
#define SENSOR3D_TIMEOUT -3
#define SENSOR3D_CONFIGURATIONERROR -4
#define SENSOR3D_ARGUMENTNULLPOINTER -101
#define SENSOR3D_ARGUMENTOUTOFRANGE -102
#define SENSOR3D_RETURNBUFFERTOOSMALL -103
#define SENSOR3D_COMMANDNOTFOUND -104
#define SENSOR3D_SOCKETCOMMUNICATIONERROR -106
#define SENSOR3D_BUSY -201
#define SENSOR3D_DATASTREAMERROR -301


// Sensor status
#define SENSOR_CONNECTED  0x1
#define SENSOR_ACQUISITION_STARTED  0x4
#define SENSOR_OVERHEATED  0x8

// Point structs
typedef struct {
	double x;
	double y;
	double z;
} struct3DPoint, POINT3D;

typedef struct {
	uint32_t offset_x;
	uint32_t offset_y;
	uint32_t width;
	uint32_t height;
} structROI, ROI;

struct POINT_CLOUD {
	POINT3D* point;
	uint16_t* intensity;
	uint16_t* confidence;
	POINT_CLOUD() {
		point = nullptr;
		intensity = nullptr;
		confidence = nullptr;
	}
};


#endif