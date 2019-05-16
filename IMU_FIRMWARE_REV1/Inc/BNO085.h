#ifndef __BNO085
#define __BNO085

#include "stdint.h"
#include "i2c.h"
#include <math.h>

#define BNO085_BUFFER_LENGTH 50

#define CHANNEL_COMMAND	0
#define CHANNEL_EXECUTABLE	1
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3
#define CHANNEL_WAKE_REPORTS 4
#define CHANNEL_GYRO 5

#define ID_ACCELEROMETER 0x01
#define ID_GYROSCOPE 0x02
#define ID_MAGNETOMETER 0x03
#define ID_LINEAR_ACCELERATION 0x04
#define ID_ROTATION_VECTOR 0x05
#define ID_GRAVITY 0x06
#define ID_GAME_ROTATION_VECTOR 0x08
#define ID_GEO_ROTATION_VECTOR 0x09
#define ID_RAW_ACCELEROMETER	0x14
#define ID_RAW_GYROSCOPE 0x15
#define ID_RAW_MAGNETOMETER 0x16

#define SHTP_COMMAND_RESPONSE 0xF1
#define SHTP_COMMAND_REQUEST 0xF2
#define SHTP_FRS_READ_RESPONSE 0xF3
#define SHTP_FRS_READ_REQUEST 0xF4
#define SHTP_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_PRODUCT_ID_REQUEST 0xF9
#define SHTP_BASE_TIMESTAMP 0xFB
#define SHTP_SET_FEATURE_COMMAND 0xFD

/*Quaternion conversion defines*/
#define THRESHOLD 0.499f
#define M_PI 3.14159265358979323846f


/*Angle typedefs*/
typedef struct
{
	float qw; /*Real part*/
	float qi;	/*Imaginary*/
	float qj;
	float qk;
}quaternionType;

typedef struct
{
	float roll;
	float pitch;
	float yaw;
}eulerType;

/*Sensor readings typedefs*/
typedef struct
{
	float X;
	float Y;
	float Z;
	uint8_t status;
	uint8_t sequenceNumber;
}motionDataType;

typedef struct
{	
	eulerType orientation;
	uint8_t status;
	uint8_t sequenceNumber;
	float accuracy;
}orientationDataType;

typedef struct
{
	uint16_t X;
	uint16_t Y;
	uint16_t Z;
	uint8_t status;
	uint8_t sequenceNumber;
}rawSensorsDataType;

/*Full sensor reading type*/
typedef struct
{
	motionDataType acceleration;
	motionDataType angular;
	motionDataType magneticField;
	
	motionDataType linearAcceleration;
	orientationDataType absoluteOrientation;
	motionDataType gravity;
	orientationDataType relativeOrientation;
	orientationDataType geoOrientation;
	
	rawSensorsDataType rawAccelerometer;
	rawSensorsDataType rawGyroscope;
	rawSensorsDataType rawMagnetometer;	
}fullSensorReadingType;

/*IMU Typedef*/
typedef struct
{
	I2C_HandleTypeDef *hi2cx;																	/*i2c handle*/
	uint8_t address;																					/*i2c device address*/
	GPIO_TypeDef *reset_GPIOx;																/*gpio handle for RESET pin*/
	uint16_t reset_Pin;																				/*RESET pin name*/
	GPIO_TypeDef *boot_GPIOx;																	/*gpio handle for nBOOT pin, if not present initialize it as NULL*/
	uint16_t nBOOT_Pin;																				/*nBOOT pin name*/
	uint8_t BNO085_Receive_Buffer [BNO085_BUFFER_LENGTH];			/*Receive buffer*/
	uint8_t BNO085_Send_Buffer [BNO085_BUFFER_LENGTH];				/*Send buffer*/
	uint8_t sequenceNumber[6];																/*Sequence numbers array*/
	uint8_t commandSequenceNumber;														/*Command sequence number*/
	fullSensorReadingType sensor_readings;	
	uint8_t sensorsEnabled[23];																/*sensorsEnabled[ID_SENSOR]= 0 -> not enabled, 1 -> enabled*/
}BNO085;

/*This funcition creates and initializes a new BNO085 object. It returns a pointer to the object. If boot is not used, pass NULL to the GPIO_TypeDef argument*/
BNO085 BNO085_CreateIMU (I2C_HandleTypeDef *hi2cx, uint8_t address, GPIO_TypeDef *reset_GPIOx, uint16_t reset_Pin, GPIO_TypeDef *boot_GPIOx, uint16_t nBOOT_Pin);

/*BNO085 SHTP comunication functions*/
uint8_t BNO085_IsAlive(BNO085 *myIMU);
void BNO085_HardReset(BNO085 *myIMU);
void BNO085_SoftReset(BNO085 *myIMU);
void BNO085_FlushI2C(BNO085 *myIMU);
uint8_t BNO085_SetFeatureCommand(BNO085 *myIMU, uint8_t sensorID, uint16_t timeBetweenReports, uint32_t configWord);
uint16_t BNO085_DataAvailable(BNO085 *myIMU);
void BNO085_ReceiveData(BNO085 *myIMU, uint16_t length);

/*BNO085 Enable calibrated sensor reports functions*/
void BNO085_EnableAccelerometer(BNO085 *myIMU, uint16_t timeBetweenReports);
void BNO085_EnableGyroscope(BNO085 *myIMU, uint16_t timeBetweenReports);
void BNO085_EnableMagnetometer(BNO085 *myIMU, uint16_t timeBetweenReports);

void BNO085_EnableLinearAcc(BNO085 *myIMU, uint16_t timeBetweenReports);
void BNO085_EnableAbsoluteRotationVector(BNO085 *myIMU, uint16_t timeBetweenReports);
void BNO085_EnableGravity(BNO085 *myIMU, uint16_t timeBetweenReports);
void BNO085_EnableRelativeRotationVector(BNO085 *myIMU, uint16_t timeBetweenReports);
void BNO085_EnableGeoRotation(BNO085 *myIMU, uint16_t timeBetweenReports);

void BNO085_EnableRawAccelerometer(BNO085 *myIMU, uint16_t timeBetweenReports);
void BNO085_EnableRawGyroscope(BNO085 *myIMU, uint16_t timeBetweenReports);
void BNO085_EnableRawMagnetometer(BNO085 *myIMU, uint16_t timeBetweenReports);

/*Update sensor reading*/
void BNO085_UpdateSensorReading(BNO085 *myIMU);
	
/*Get separated sensor readings*/
motionDataType BNO085_GetAcceleration(BNO085 *myIMU);
motionDataType BNO085_GetAngularRate(BNO085 *myIMU);
motionDataType BNO085_GetMagnetometer(BNO085 *myIMU);

motionDataType BNO085_GetLinearAcceleration(BNO085 *myIMU);
orientationDataType BNO085_GetAbsoluteOrientation(BNO085 *myIMU);
motionDataType BNO085_GetGravity(BNO085 *myIMU);
orientationDataType BNO085_GetRelativeOrientation(BNO085 *myIMU);
orientationDataType BNO085_GetGeoOrientation(BNO085 *myIMU);

rawSensorsDataType BNO085_GetRawAccelerometer(BNO085 *myIMU);
rawSensorsDataType BNO085_GetRawGyroscope(BNO085 *myIMU);
rawSensorsDataType BNO085_GetRawMagnetometer(BNO085 *myIMU);

/*Math functions*/
eulerType quaternionToEuler (quaternionType quaternion);
float radToDeg(float angle);
quaternionType populateQuaternion (int16_t real, int16_t i, int16_t j, int16_t k);
float qToFloat(int16_t fixedPointValue, uint8_t qPoint);

#endif
