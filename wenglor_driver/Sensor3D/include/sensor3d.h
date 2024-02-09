#ifndef SENSOR3D_H
#define SENSOR3D_H
#if defined(_MSC_VER)
    #define SCANNER3D_DLL_EXPORT __declspec(dllexport)        
#else
    #define SCANNER3D_DLL_EXPORT __attribute__ ((visibility ("default")))
    #define _stdcall
#endif



extern "C" SCANNER3D_DLL_EXPORT void* Sensor3D_Connect(const char* ip, int timeout);
extern "C" SCANNER3D_DLL_EXPORT int  Sensor3D_Disconnect(void* ethernet_scanner);
extern "C" SCANNER3D_DLL_EXPORT int  Sensor3D_GetSensorStatus(void* ethernet_scanner, int* status);
extern "C" SCANNER3D_DLL_EXPORT int  Sensor3D_GetVersion(char* buffer, int size);
extern "C" SCANNER3D_DLL_EXPORT int  Sensor3D_ReadData(void* ethernet_scanner, const char* feature_name, char* buffer, int size, int reserved);
extern "C" SCANNER3D_DLL_EXPORT int  Sensor3D_WriteData(void* ethernet_scanner, const char* buffer);
extern "C" SCANNER3D_DLL_EXPORT int  Sensor3D_GetPointCloud(void* ethernet_scanner, void* buffer, int buffer_size, int* number_of_points, void* roi, int timeout);
extern "C" SCANNER3D_DLL_EXPORT int  Sensor3D_GetCameraImage(void* ethernet_scanner, void* buffer, int buffer_size, void* roi, int timeout);
extern "C" SCANNER3D_DLL_EXPORT int  Sensor3D_ListDevices(char* buffer, int size);
extern "C" SCANNER3D_DLL_EXPORT int  Sensor3D_RegisterPclCallback(void* ethernet_scanner, void (*pcl_callback_func)(void* context),void* context);
extern "C" SCANNER3D_DLL_EXPORT int  Sensor3D_UnregisterPclCallback(void* ethernet_scanner);

#endif // SENSOR3D_H
