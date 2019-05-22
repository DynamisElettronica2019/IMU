#ifndef __BNO085
#define __BNO085

#include "stdint.h"
#include "i2c.h"
#include <math.h>

#define BNO085_BUFFER_LENGTH 50
#define BNO085_DEBUG_BUFFER_LENGTH 30
#define BNO085_FRS_BUFFER_LENGTH 100

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
#define SHTP_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_PRODUCT_ID_REQUEST 0xF9
#define SHTP_BASE_TIMESTAMP 0xFB
#define SHTP_SET_FEATURE_RESPONSE 0xFC
#define SHTP_SET_FEATURE_COMMAND 0xFD

#define SHTP_COMMAND_ID_ERRORS 0x01
#define SHTP_COMMAND_ID_COUNTER 0x02
#define SHTP_COMMAND_ID_TARE 0x03
#define SHTP_COMMAND_ID_INITIALIZE 0x04
#define SHTP_COMMAND_ID_RESERVED 0x05
#define SHTP_COMMAND_ID_SAVE_DCD 0x06
#define SHTP_COMMAND_ID_ME_CALIBRATION 0x07
#define SHTP_COMMAND_ID_CONFIGURE_DCD 0x09
#define SHTP_COMMAND_ID_OSCILLATOR 0x0A
#define SHTP_COMMAND_ID_CLEAR_DCD 0x0B

#define SHTP_FRS_WRITE_REQUEST_ID 0xF7
#define SHTP_FRS_WRITE_DATA_REQUEST_ID 0xF6
#define SHTP_FRS_WRITE_RESPONSE_ID 0xF5
#define SHTP_FRS_READ_REQUEST_ID 0xF4
#define SHTP_FRS_READ_RESPONSE_ID 0xF3

/*Quaternion conversion defines*/
#define THRESHOLD 0.499999f
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
	fullSensorReadingType sensor_readings;										/*Stores the readings of the sensors. Updated every time the update sensor reading function is called*/
	uint8_t sensorsEnabled[23];																/*sensorsEnabled[ID_SENSOR]= 0 -> not enabled, 1 -> enabled*/
	uint8_t BNO085_Product_ID_Buffer[BNO085_DEBUG_BUFFER_LENGTH];
	uint8_t BNO085_Command_Buffer[BNO085_DEBUG_BUFFER_LENGTH];/*Buffer used for Command Responses*/
	
	uint32_t BNO085_FRS_Read_Buffer[BNO085_FRS_BUFFER_LENGTH];		/*Buffer used for FRS reads, FRS write responses*/
	uint32_t BNO085_FRS_Write_Buffer[BNO085_FRS_BUFFER_LENGTH];		/*Buffer used for FRS reads, FRS write responses*/
	
}BNO085;

/*Math functions*/
eulerType quaternionToEuler (quaternionType quaternion);
float radToDeg(float angle);
quaternionType populateQuaternion (int16_t real, int16_t i, int16_t j, int16_t k);
float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
int16_t floatToQ (double realToConvert, uint8_t qPoint);

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

/*Product ID request*/
void BNO085_Product_ID_Request (BNO085 *myIMU);

void BNO085_GetProductID(BNO085 *myIMU, uint16_t length);

/*Command functions*/
void BNO085_Command_RequestErrorReport(BNO085 *myIMU, uint8_t severity);

void BNO085_Command_GetCounts(BNO085 *myIMU, uint8_t sensorID);
void BNO085_Command_ClearCounts(BNO085 *myIMU, uint8_t sensorID);

void BNO085_Command_TareNow(BNO085 *myIMU, uint8_t axesToTare, uint8_t vectorToTare);
void BNO085_Command_PersistTare(BNO085 *myIMU);
void BNO085_Command_SetReorientation(BNO085 *myIMU, double quatX, double quatY, double quatZ, double quatW);

void BNO085_Command_Initialize(BNO085 *myIMU);

void BNO085_Command_SaveDCD(BNO085 *myIMU);

void BNO085_Command_EnableFullCalibration(BNO085 *myIMU);
void BNO085_Command_DisableFullCalibration(BNO085 *myIMU);
void BNO085_Command_ConfigureCalibration(BNO085 *myIMU, uint8_t accelerometer, uint8_t gyroscope, uint8_t magnetometer, uint8_t planar);
void BNO085_Command_GetCalibrationStatus(BNO085 *myIMU);

void BNO085_Command_EnableAutoSaveDCD(BNO085 *myIMU);
void BNO085_Command_DisableAutoSaveDCD(BNO085 *myIMU);

void BNO085_Command_GetOscType(BNO085 *myIMU);

void BNO085_Command_ClearResetDCD(BNO085 *myIMU);

/*Command parsing*/
void BNO085_GetCommandResponse(BNO085 *myIMU, uint16_t length);

/*FRS basic communication*/
uint8_t BNO085_FRS_PerformWriteOperation (BNO085 *myIMU, uint16_t wordsToWrite, uint16_t FRSType, uint16_t initialOffset);
void BNO085_FRS_InitializeWriteRequest(BNO085 *myIMU, uint16_t length, uint16_t FRSType);
void BNO085_FRS_WriteData_Request (BNO085 *myIMU, uint16_t offset, uint32_t data0, uint32_t data1);

uint8_t BNO085_FRS_ReadRequest (BNO085 *myIMU, uint16_t offset, uint16_t FRSType, uint16_t numberOfWords);
void BNO085_FRS_GetReadResponse (BNO085 *myIMU, uint32_t *data0, uint32_t *data1);

/*FRS requests*/
uint8_t BNO085_FRS_RequestOrientation(BNO085* myIMU);

#endif
