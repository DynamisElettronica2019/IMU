/*HEADER FILE BNO085 LIBRARY*/
#include "stdint.h"

/*THRESOLD for handling singularities in quaternion conversion*/
#define THRESHOLD 0.499
#define M_PI 3.14159265358979323846
#define IMU_ADDRESS (0x28 << 1) /*Alternative 0x29 (IMU PIN 17 HIGH), one left bit shift required for hal function*/ 

/*Quaternion type. Used for conversion*/
typedef struct
{
	float qw;
	float qi;
	float qj;
	float qk;
}quaternionType;

/*Euler angle type. Units: Degrees*/
typedef struct
{
	float roll;
	float pitch;
	float yaw;
}eulerType;

/*Struct used to store raw data from the sensor*/
typedef struct
{
	/*Accelerometer uncompensated data. Units m/s^2. Q point = 8*/
	int16_t accel_X;
	int16_t accel_Y;
	int16_t accel_Z;
	
	/*Accelerometer data compensated against gravity. Units m/s^2. Q point = 8*/
	int16_t linear_X;
	int16_t linear_Y;
	int16_t linear_Z;
	
	/*Gyroscope angular rate data. Units rad/s. Q point = 9*/
	int16_t angular_X;
	int16_t angular_Y;
	int16_t angular_Z;
	
	/*Quaternion rotation data. Order: i - j - k - real. Q point = 12 + accuracy, Q point 12 (units??).*/
	int16_t quat_i;
	int16_t quat_j;
	int16_t quat_k;
	int16_t quat_r;
	int16_t accuracy;
	
}sensorRawReadType;

/*Functions prototypes*/

eulerType quaternionToEuler (quaternionType quaternion);
float radToDeg(float angle);
