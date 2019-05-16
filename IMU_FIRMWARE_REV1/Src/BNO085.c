#include "BNO085.h"

/*MATH FUNCTIONS*/

/*Conversion from quaternions to Euler angles*/
eulerType quaternionToEuler (quaternionType quaternion)
{
	eulerType output;
	float sqi,sqj,sqk;
	float test;

	test = quaternion.qi*quaternion.qj + quaternion.qk*quaternion.qw;

	if (test > THRESHOLD) 
	{
		output.yaw = 2 * atan2(quaternion.qi,quaternion.qw);
		output.pitch = M_PI/2;
		output.roll = 0;
		output.yaw = radToDeg(output.yaw);
		output.pitch = radToDeg(output.pitch);
		output.roll = radToDeg(output.roll);
		return output;
	}

	if (test < -THRESHOLD)
	{
		output.yaw = -2 * atan2(quaternion.qi,quaternion.qw);
		output.pitch = - M_PI/2;
		output.roll = 0;
		output.yaw = radToDeg(output.yaw);
		output.pitch = radToDeg(output.pitch);
		output.roll = radToDeg(output.roll);
		return output;
	}

  sqi = quaternion.qi*quaternion.qi;
  sqj = quaternion.qj*quaternion.qj;
  sqk = quaternion.qk*quaternion.qk;

  output.yaw = atan2(2*quaternion.qj*quaternion.qw-2*quaternion.qi*quaternion.qk , 1 - 2*sqj - 2*sqk);
	output.pitch = asin(2*test);
	output.roll = atan2(2*quaternion.qi*quaternion.qw-2*quaternion.qj*quaternion.qk , 1 - 2*sqi - 2*sqk);
	output.yaw = radToDeg(output.yaw);
	output.pitch = radToDeg(output.pitch);
	output.roll = radToDeg(output.roll);
	

	return output;
}

/*Conversion from radians to degrees*/
float radToDeg(float angle)
{
	return ((angle/M_PI)*180.00f);
}

/*This functions takes values from the reading and puts them into a quaternion type*/
quaternionType populateQuaternion (int16_t real, int16_t i, int16_t j, int16_t k)
{
	quaternionType outputQuaternion;
	outputQuaternion.qw = real;
	outputQuaternion.qi = i;
	outputQuaternion.qj = j;
	outputQuaternion.qk = k;
	
	return outputQuaternion;
}

/*Conversion from q (fixed point) numbers to float*/
float qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	double point;
	float qFloat = fixedPointValue;
	point = qPoint * (-1);
	qFloat = qFloat * pow(2, point);
	return (qFloat);
}

/*BNO085 Functions*/

/*Initialization functions*/
BNO085 BNO085_CreateIMU (I2C_HandleTypeDef *hi2cx, uint8_t address, GPIO_TypeDef *reset_GPIOx, uint16_t reset_Pin, GPIO_TypeDef *boot_GPIOx, uint16_t nBOOT_Pin)
{
	BNO085 myIMU;
	int i=0;
	
	myIMU.hi2cx=hi2cx;
	myIMU.address=address;
	myIMU.reset_GPIOx=reset_GPIOx;
	myIMU.reset_Pin=reset_Pin;
	myIMU.boot_GPIOx=boot_GPIOx;
	myIMU.nBOOT_Pin=nBOOT_Pin;
	
	while (i<6)
	{
		myIMU.sequenceNumber[i]=0;
		i++;
	}
	i=0;
	while (i<10)
	{
		myIMU.sensorsEnabled[i]=0;
		i++;
	}
	myIMU.commandSequenceNumber=0;
	
	return myIMU;
}

/*SHTP Basic Operations Functions*/

/*Complete reset of the sensor*/
void BNO085_HardReset(BNO085 *myIMU)
{
	HAL_GPIO_WritePin(myIMU->reset_GPIOx, myIMU->reset_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(myIMU->boot_GPIOx, myIMU->nBOOT_Pin, GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(myIMU->reset_GPIOx, myIMU->reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(myIMU->reset_GPIOx, myIMU->reset_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	BNO085_SoftReset(myIMU);
	return;
}

void BNO085_SoftReset(BNO085 *myIMU)
{
	uint8_t i=0;
	
	while (i<23)
	{
		myIMU->sensorsEnabled[i]=0;
		i++;
	}
	
	myIMU->BNO085_Send_Buffer[0]=5;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_EXECUTABLE;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_EXECUTABLE];
	myIMU->sequenceNumber[CHANNEL_EXECUTABLE]++;
	myIMU->BNO085_Send_Buffer[4]=1;
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer, 5, 100);
	HAL_Delay(50);
	BNO085_FlushI2C(myIMU);
	HAL_Delay(50);
	BNO085_FlushI2C(myIMU);
	return;
}

void BNO085_FlushI2C(BNO085 *myIMU)
{
	uint16_t packetLength;
	packetLength = BNO085_DataAvailable(myIMU);
	while(packetLength!=0)
	{
		BNO085_ReceiveData(myIMU, packetLength);
		packetLength = BNO085_DataAvailable(myIMU);
	}
	return;
}

uint16_t BNO085_DataAvailable(BNO085 *myIMU)
{
	uint16_t dataLength;
	HAL_I2C_Master_Receive(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Receive_Buffer, 4, 1000);
	dataLength = ((uint16_t)myIMU->BNO085_Receive_Buffer[1] << 8 | myIMU->BNO085_Receive_Buffer[0]);
	dataLength &= ~(1 << 15);
	return dataLength;
}

void BNO085_ReceiveData(BNO085 *myIMU, uint16_t length)
{
	if (length>BNO085_BUFFER_LENGTH)
	{
		length=BNO085_BUFFER_LENGTH;
	}
	HAL_I2C_Master_Receive(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Receive_Buffer, length, 1000);
	return;
}

uint8_t BNO085_IsAlive(BNO085 *myIMU)
{
	uint8_t status;
	status = HAL_I2C_IsDeviceReady(myIMU->hi2cx,myIMU->address,10,1000);
	return status;
}

uint8_t BNO085_SetFeatureCommand(BNO085 *myIMU, uint8_t sensorID, uint16_t timeBetweenReports, uint32_t configWord)
{
	long microsBetweenReports = (long)timeBetweenReports * 1000L;
	uint16_t dataLength;
	myIMU->BNO085_Send_Buffer[0]=21;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
	
	myIMU->BNO085_Send_Buffer[4]=SHTP_SET_FEATURE_COMMAND;
	myIMU->BNO085_Send_Buffer[5]=sensorID;
	myIMU->BNO085_Send_Buffer[6]=0;
	myIMU->BNO085_Send_Buffer[7]=0;
	
	myIMU->BNO085_Send_Buffer[8]=0;
	myIMU->BNO085_Send_Buffer[9]=(microsBetweenReports >> 0) & 0xFF;
	myIMU->BNO085_Send_Buffer[10]=(microsBetweenReports >> 8) & 0xFF;
	myIMU->BNO085_Send_Buffer[11]=(microsBetweenReports >> 16) & 0xFF;
	
	myIMU->BNO085_Send_Buffer[12]=(microsBetweenReports >> 24) & 0xFF;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	myIMU->BNO085_Send_Buffer[16]=0;
	myIMU->BNO085_Send_Buffer[17]=(configWord >> 0) & 0xFF;
	myIMU->BNO085_Send_Buffer[18]=(configWord >> 8) & 0xFF;
	myIMU->BNO085_Send_Buffer[19]=(configWord >> 16) & 0xFF;
	
	myIMU->BNO085_Send_Buffer[20]=(configWord >> 24) & 0xFF;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,21, 1000);
	
	/*Receive sensor activation report. ReceiveBuffer[4] = 0xFC, ReceiveBuffer[5]=SENSOR_ID*/
	while (myIMU->sensorsEnabled[sensorID]==0)
	{
	dataLength = BNO085_DataAvailable(myIMU);
	while (dataLength==0)
	{
		dataLength = BNO085_DataAvailable(myIMU);
	}
	BNO085_ReceiveData(myIMU, dataLength);
	if (myIMU->BNO085_Receive_Buffer[4]==0xFC)
	{
		myIMU->sensorsEnabled[myIMU->BNO085_Receive_Buffer[5]]=1;
	}
	}
	return 1;
}

/*Enable sensor reports functions*/

void BNO085_EnableAccelerometer(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_ACCELEROMETER, timeBetweenReports, 0);
	return;
}

void BNO085_EnableGyroscope(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_GYROSCOPE, timeBetweenReports, 0);
	return;
}

void BNO085_EnableMagnetometer(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_MAGNETOMETER, timeBetweenReports, 0);
	return;
}

void BNO085_EnableLinearAcc(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_LINEAR_ACCELERATION, timeBetweenReports, 0);
	return;
}

void BNO085_EnableAbsoluteRotationVector(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_ROTATION_VECTOR, timeBetweenReports, 0);
	return;
}

void BNO085_EnableGravity(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_GRAVITY, timeBetweenReports, 0);
	return;
}

void BNO085_EnableRelativeRotationVector(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_GAME_ROTATION_VECTOR, timeBetweenReports, 0);
	return;
}

void BNO085_EnableGeoRotation(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_GEO_ROTATION_VECTOR, timeBetweenReports, 0);
	return;
}

void BNO085_EnableRawAccelerometer(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_RAW_ACCELEROMETER, timeBetweenReports, 0);
	return;
}
void BNO085_EnableRawGyroscope(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_RAW_GYROSCOPE, timeBetweenReports, 0);
	return;
}
void BNO085_EnableRawMagnetometer(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_RAW_MAGNETOMETER, timeBetweenReports, 0);
	return;
}

/*Sensor reading functions*/

void BNO085_UpdateSensorReading(BNO085 *myIMU)
{
	uint16_t length;
	length = BNO085_DataAvailable(myIMU);
	
	while (length==0)
	{
			length = BNO085_DataAvailable(myIMU);
	}
	BNO085_ReceiveData(myIMU, length);
	if (myIMU->BNO085_Receive_Buffer[2]==CHANNEL_REPORTS && myIMU->BNO085_Receive_Buffer[4]==SHTP_BASE_TIMESTAMP)
	{
		switch (myIMU->BNO085_Receive_Buffer[9])
		{
			case ID_ACCELEROMETER:
				myIMU->sensor_readings.acceleration = BNO085_GetAcceleration(myIMU);
			break;
			
			case ID_GYROSCOPE:
				myIMU->sensor_readings.angular = BNO085_GetAngularRate(myIMU);
			break;
			
			case ID_MAGNETOMETER:
				myIMU->sensor_readings.magneticField = BNO085_GetMagnetometer(myIMU);
			break;
			
			case ID_LINEAR_ACCELERATION:
				myIMU->sensor_readings.linearAcceleration = BNO085_GetLinearAcceleration(myIMU);
			break;
			
			case ID_ROTATION_VECTOR:
				myIMU->sensor_readings.absoluteOrientation = BNO085_GetAbsoluteOrientation(myIMU);
			break;
			
			case ID_GRAVITY:
				myIMU->sensor_readings.gravity = BNO085_GetGravity(myIMU);
			break;
			
			case ID_GAME_ROTATION_VECTOR:
				myIMU->sensor_readings.relativeOrientation = BNO085_GetRelativeOrientation(myIMU);
			break;
			
			case ID_GEO_ROTATION_VECTOR:
				myIMU->sensor_readings.geoOrientation = BNO085_GetGeoOrientation(myIMU);
			break;

			case ID_RAW_ACCELEROMETER:
				myIMU->sensor_readings.rawAccelerometer = BNO085_GetRawAccelerometer(myIMU);
			break;

			case ID_RAW_GYROSCOPE:
				myIMU->sensor_readings.rawGyroscope = BNO085_GetRawGyroscope(myIMU);
			break;
									
			case ID_RAW_MAGNETOMETER:
				myIMU->sensor_readings.rawMagnetometer = BNO085_GetRawMagnetometer(myIMU);
			break;
	
			default:
				/*Unhandled ID*/
			break;
		}
	}
	return;
}

/*Sensor report parsing functions*/

motionDataType BNO085_GetAcceleration(BNO085 *myIMU)
{
	motionDataType accelerationData;
	uint8_t status;
	uint16_t dataRawX;
	uint16_t dataRawY;
	uint16_t dataRawZ;
	
	accelerationData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	status=(myIMU->BNO085_Receive_Buffer[11])&0x03;
	accelerationData.status=status;
	
	dataRawX = ((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8) | myIMU->BNO085_Receive_Buffer[13]);
	accelerationData.X =qToFloat(dataRawX,8);
	
	dataRawY = ((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8) | myIMU->BNO085_Receive_Buffer[15]);
	accelerationData.Y =qToFloat(dataRawY,8);
	
	dataRawZ = ((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8) | myIMU->BNO085_Receive_Buffer[17]);
	accelerationData.Z =qToFloat(dataRawZ,8);
	
	return accelerationData;
}

motionDataType BNO085_GetAngularRate(BNO085 *myIMU)
{
	motionDataType angularRateData;
	
	uint8_t status;
	uint16_t dataRawX;
	uint16_t dataRawY;
	uint16_t dataRawZ;
	
	angularRateData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	status=(myIMU->BNO085_Receive_Buffer[11])&0x03;
	angularRateData.status=status;
	
	dataRawX = ((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8) | myIMU->BNO085_Receive_Buffer[13]);
	angularRateData.X =qToFloat(dataRawX,9);
	
	dataRawY = ((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8) | myIMU->BNO085_Receive_Buffer[15]);
	angularRateData.Y =qToFloat(dataRawY,9);
	
	dataRawZ = ((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8) | myIMU->BNO085_Receive_Buffer[17]);
	angularRateData.Z =qToFloat(dataRawZ,9);

	return angularRateData;
}

motionDataType BNO085_GetMagnetometer(BNO085 *myIMU)
{
	motionDataType magneticFieldData;
	
	uint8_t status;
	uint16_t dataRawX;
	uint16_t dataRawY;
	uint16_t dataRawZ;
	
	magneticFieldData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	status=(myIMU->BNO085_Receive_Buffer[11])&0x03;
	magneticFieldData.status=status;
	
	dataRawX = ((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8) | myIMU->BNO085_Receive_Buffer[13]);
	magneticFieldData.X =qToFloat(dataRawX,4);
	
	dataRawY = ((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8) | myIMU->BNO085_Receive_Buffer[15]);
	magneticFieldData.Y =qToFloat(dataRawY,4);
	
	dataRawZ = ((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8) | myIMU->BNO085_Receive_Buffer[17]);
	magneticFieldData.Z =qToFloat(dataRawZ,4);

	return magneticFieldData;
}

motionDataType BNO085_GetLinearAcceleration(BNO085 *myIMU)
{
	motionDataType linearAccData;
	uint8_t status;
	uint16_t dataRawX;
	uint16_t dataRawY;
	uint16_t dataRawZ;
	
	linearAccData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	status=(myIMU->BNO085_Receive_Buffer[11])&0x03;
	linearAccData.status=status;
	
	dataRawX = ((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8) | myIMU->BNO085_Receive_Buffer[13]);
	linearAccData.X =qToFloat(dataRawX,8);
	
	dataRawY = ((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8) | myIMU->BNO085_Receive_Buffer[15]);
	linearAccData.Y =qToFloat(dataRawY,8);
	
	dataRawZ = ((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8) | myIMU->BNO085_Receive_Buffer[17]);
	linearAccData.Z =qToFloat(dataRawZ,8);
	return linearAccData;
}

orientationDataType BNO085_GetAbsoluteOrientation(BNO085 *myIMU)
{
	orientationDataType absoluteOrientation;
	quaternionType absoluteQuaternion;

	uint16_t quaternionI;
	uint16_t quaternionJ;
	uint16_t quaternionK;
	uint16_t quaternionR;
	uint16_t accuracy;
	
	uint8_t status;
	
	quaternionI=((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8)| myIMU->BNO085_Receive_Buffer[13]);
	quaternionJ=((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8)| myIMU->BNO085_Receive_Buffer[15]);
	quaternionK=((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8)| myIMU->BNO085_Receive_Buffer[17]);
	quaternionR=((((uint16_t)myIMU->BNO085_Receive_Buffer[20]) << 8)| myIMU->BNO085_Receive_Buffer[19]);
	
	absoluteQuaternion.qi=qToFloat(quaternionI,14);
	absoluteQuaternion.qj=qToFloat(quaternionJ,14);
	absoluteQuaternion.qk=qToFloat(quaternionK,14);
	absoluteQuaternion.qw=qToFloat(quaternionR,14);
	
	absoluteOrientation.orientation=quaternionToEuler(absoluteQuaternion);
	
	status = myIMU->BNO085_Receive_Buffer[11]&0x03;
	absoluteOrientation.status = status;

	absoluteOrientation.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	accuracy = ((((uint16_t)myIMU->BNO085_Receive_Buffer[22]) << 8)| myIMU->BNO085_Receive_Buffer[21]);
	accuracy=qToFloat(accuracy,12);
	absoluteOrientation.accuracy=radToDeg(accuracy);
	
	return absoluteOrientation;
}

motionDataType BNO085_GetGravity(BNO085 *myIMU)
{
	motionDataType gravityData;
	uint8_t status;
	uint16_t dataRawX;
	uint16_t dataRawY;
	uint16_t dataRawZ;
	
	gravityData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	status=(myIMU->BNO085_Receive_Buffer[11])&0x03;
	gravityData.status=status;
	
	dataRawX = ((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8) | myIMU->BNO085_Receive_Buffer[13]);
	gravityData.X =qToFloat(dataRawX,8);
	
	dataRawY = ((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8) | myIMU->BNO085_Receive_Buffer[15]);
	gravityData.Y =qToFloat(dataRawY,8);
	
	dataRawZ = ((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8) | myIMU->BNO085_Receive_Buffer[17]);
	gravityData.Z =qToFloat(dataRawZ,8);
	return gravityData;
}

orientationDataType BNO085_GetRelativeOrientation(BNO085 *myIMU)
{
	orientationDataType relativeOrientation;
	quaternionType relativeQuaternion;

	uint16_t quaternionI;
	uint16_t quaternionJ;
	uint16_t quaternionK;
	uint16_t quaternionR;
	
	uint8_t status;
	
	quaternionI=((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8)| myIMU->BNO085_Receive_Buffer[13]);
	quaternionJ=((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8)| myIMU->BNO085_Receive_Buffer[15]);
	quaternionK=((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8)| myIMU->BNO085_Receive_Buffer[17]);
	quaternionR=((((uint16_t)myIMU->BNO085_Receive_Buffer[20]) << 8)| myIMU->BNO085_Receive_Buffer[19]);
	
	relativeQuaternion.qi=qToFloat(quaternionI,14);
	relativeQuaternion.qj=qToFloat(quaternionJ,14);
	relativeQuaternion.qk=qToFloat(quaternionK,14);
	relativeQuaternion.qw=qToFloat(quaternionR,14);
	
	relativeOrientation.orientation=quaternionToEuler(relativeQuaternion);
	
	status = myIMU->BNO085_Receive_Buffer[11]&0x03;
	relativeOrientation.status = status;

	relativeOrientation.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	return relativeOrientation;
}

orientationDataType BNO085_GetGeoOrientation(BNO085 *myIMU)
{
	orientationDataType geoOrientation;
	quaternionType geoQuaternion;

	uint16_t quaternionI;
	uint16_t quaternionJ;
	uint16_t quaternionK;
	uint16_t quaternionR;
	uint16_t accuracy;
	
	uint8_t status;
	
	quaternionI=((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8)| myIMU->BNO085_Receive_Buffer[13]);
	quaternionJ=((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8)| myIMU->BNO085_Receive_Buffer[15]);
	quaternionK=((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8)| myIMU->BNO085_Receive_Buffer[17]);
	quaternionR=((((uint16_t)myIMU->BNO085_Receive_Buffer[20]) << 8)| myIMU->BNO085_Receive_Buffer[19]);
	
	geoQuaternion.qi=qToFloat(quaternionI,14);
	geoQuaternion.qj=qToFloat(quaternionJ,14);
	geoQuaternion.qk=qToFloat(quaternionK,14);
	geoQuaternion.qw=qToFloat(quaternionR,14);
	
	geoOrientation.orientation=quaternionToEuler(geoQuaternion);
	
	status = myIMU->BNO085_Receive_Buffer[11]&0x03;
	geoOrientation.status = status;

	geoOrientation.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	accuracy = ((((uint16_t)myIMU->BNO085_Receive_Buffer[22]) << 8)| myIMU->BNO085_Receive_Buffer[21]);
	accuracy=qToFloat(accuracy,12);
	geoOrientation.accuracy=radToDeg(accuracy);
	
	return geoOrientation;
}

rawSensorsDataType BNO085_GetRawAccelerometer(BNO085 *myIMU)
{
	rawSensorsDataType rawAccelerometerData;
	uint8_t status;
	uint16_t dataRawX;
	uint16_t dataRawY;
	uint16_t dataRawZ;
	
	rawAccelerometerData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	status=(myIMU->BNO085_Receive_Buffer[11])&0x03;
	rawAccelerometerData.status=status;
	
	dataRawX = ((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8) | myIMU->BNO085_Receive_Buffer[13]);
	rawAccelerometerData.X = dataRawX;
	
	dataRawY = ((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8) | myIMU->BNO085_Receive_Buffer[15]);
	rawAccelerometerData.Y = dataRawY;
	
	dataRawZ = ((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8) | myIMU->BNO085_Receive_Buffer[17]);
	rawAccelerometerData.Z = dataRawZ;
	return rawAccelerometerData;
}

rawSensorsDataType BNO085_GetRawGyroscope(BNO085 *myIMU)
{
	rawSensorsDataType rawGyroscopeData;
	uint8_t status;
	uint16_t dataRawX;
	uint16_t dataRawY;
	uint16_t dataRawZ;
	
	rawGyroscopeData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	status=(myIMU->BNO085_Receive_Buffer[11])&0x03;
	rawGyroscopeData.status=status;
	
	dataRawX = ((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8) | myIMU->BNO085_Receive_Buffer[13]);
	rawGyroscopeData.X = dataRawX;
	
	dataRawY = ((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8) | myIMU->BNO085_Receive_Buffer[15]);
	rawGyroscopeData.Y = dataRawY;
	
	dataRawZ = ((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8) | myIMU->BNO085_Receive_Buffer[17]);
	rawGyroscopeData.Z = dataRawZ;
	return rawGyroscopeData;
}

rawSensorsDataType BNO085_GetRawMagnetometer(BNO085 *myIMU)
{
	rawSensorsDataType rawMagnetometerData;
	uint8_t status;
	uint16_t dataRawX;
	uint16_t dataRawY;
	uint16_t dataRawZ;
	
	rawMagnetometerData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	
	status=(myIMU->BNO085_Receive_Buffer[11])&0x03;
	rawMagnetometerData.status=status;
	
	dataRawX = ((((uint16_t)myIMU->BNO085_Receive_Buffer[14]) << 8) | myIMU->BNO085_Receive_Buffer[13]);
	rawMagnetometerData.X = dataRawX;
	
	dataRawY = ((((uint16_t)myIMU->BNO085_Receive_Buffer[16]) << 8) | myIMU->BNO085_Receive_Buffer[15]);
	rawMagnetometerData.Y = dataRawY;
	
	dataRawZ = ((((uint16_t)myIMU->BNO085_Receive_Buffer[18]) << 8) | myIMU->BNO085_Receive_Buffer[17]);
	rawMagnetometerData.Z = dataRawZ;
	return rawMagnetometerData;
}
