#include "BNO085.h"

/*MATH FUNCTIONS*/

/*Conversion from quaternions to Euler angles*/
eulerType quaternionToEuler (quaternionType quaternion)
{
	eulerType output;
	float sqi,sqj,sqk;
	float test;

	test = quaternion.qk*quaternion.qj + quaternion.qi*quaternion.qw;

	if (test > THRESHOLD) 
	{
		output.roll = 2 * atan2(quaternion.qk,quaternion.qw);
		output.pitch = M_PI/2;
		output.yaw = 0;
		output.roll = radToDeg(output.roll);
		output.pitch = radToDeg(output.pitch);
		output.yaw = radToDeg(output.yaw);
		return output;
	}

	if (test < -THRESHOLD)
	{
		output.roll = -2 * atan2(quaternion.qk,quaternion.qw);
		output.pitch = - M_PI/2;
		output.yaw = 0;
		output.roll = radToDeg(output.roll);
		output.pitch = radToDeg(output.pitch);
		output.yaw = radToDeg(output.yaw);
		return output;
	}

  sqi = quaternion.qi*quaternion.qi;
  sqj = quaternion.qj*quaternion.qj;
  sqk = quaternion.qk*quaternion.qk;

  output.roll = atan2(2*quaternion.qj*quaternion.qw-2*quaternion.qk*quaternion.qi , 1 - 2*sqj - 2*sqi);
	output.pitch = asin(2*test);
	output.yaw = atan2(2*quaternion.qk*quaternion.qw-2*quaternion.qj*quaternion.qi , 1 - 2*sqk - 2*sqi);
	output.roll = radToDeg(output.roll);
	output.pitch = radToDeg(output.pitch);
	output.yaw = radToDeg(output.yaw);
	
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

int32_t floatToQ (double realToConvert, uint8_t qPoint)
{
	int32_t qFormatNumber;
	realToConvert=realToConvert*pow(2,qPoint);
	qFormatNumber=(int32_t)realToConvert;
	return qFormatNumber;
}

/*BNO085 Functions*/

/*Initialization functions*/
BNO085 BNO085_CreateIMU (I2C_HandleTypeDef *hi2cx, uint8_t address, GPIO_TypeDef *reset_GPIOx, uint16_t reset_Pin, GPIO_TypeDef *boot_GPIOx, uint16_t nBOOT_Pin)
{
	BNO085 myIMU;
	int i;
	
	myIMU.hi2cx=hi2cx;
	myIMU.address=address;
	myIMU.reset_GPIOx=reset_GPIOx;
	myIMU.reset_Pin=reset_Pin;
	myIMU.boot_GPIOx=boot_GPIOx;
	myIMU.nBOOT_Pin=nBOOT_Pin;
	myIMU.commandSequenceNumber=0;
	
	i=0;
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
	
	i=0;
	while (i<BNO085_DEBUG_BUFFER_LENGTH)
	{
		myIMU.BNO085_Command_Buffer[i]=0;
		myIMU.BNO085_Product_ID_Buffer[i]=0;
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
	if (myIMU->BNO085_Receive_Buffer[4]== SHTP_SET_FEATURE_RESPONSE)
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

	/*Handle sensor reports*/
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
		return;
	}
	
	/*Handle product ID response*/
	if (myIMU->BNO085_Receive_Buffer[2]==CHANNEL_CONTROL && myIMU->BNO085_Receive_Buffer[4]==SHTP_PRODUCT_ID_RESPONSE)
	{
		BNO085_GetProductID(myIMU, 20);
		return;
	}
	
	/*Handle command responses*/
	if (myIMU->BNO085_Receive_Buffer[2]==CHANNEL_CONTROL && myIMU->BNO085_Receive_Buffer[4]==SHTP_COMMAND_RESPONSE)
	{
		BNO085_GetCommandResponse(myIMU, length);
		return;
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

void BNO085_Product_ID_Request (BNO085 *myIMU)
{
	myIMU->BNO085_Send_Buffer[0]=6;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
	
	myIMU->BNO085_Send_Buffer[4]= SHTP_PRODUCT_ID_REQUEST;
	myIMU->BNO085_Send_Buffer[5]= 0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,6, 1000);
	return;
}

void BNO085_GetProductID(BNO085 *myIMU, uint16_t length)
{
	int count=0;
	
	while (count<length)
	{
		myIMU->BNO085_Product_ID_Buffer[count]=myIMU->BNO085_Receive_Buffer[count];
		count++;
	}
	return;
}

void BNO085_Command_RequestErrorReport(BNO085 *myIMU, uint8_t severity)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_ERRORS;
	myIMU->BNO085_Send_Buffer[7]=severity;
		
	myIMU->BNO085_Send_Buffer[8]=0;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}
	
void BNO085_Command_GetCounts(BNO085 *myIMU, uint8_t sensorID)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_COUNTER;
	myIMU->BNO085_Send_Buffer[7]=0x00;
		
	myIMU->BNO085_Send_Buffer[8]=sensorID;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}

void BNO085_Command_ClearCounts(BNO085 *myIMU, uint8_t sensorID)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_COUNTER;
	myIMU->BNO085_Send_Buffer[7]=0x01;
		
	myIMU->BNO085_Send_Buffer[8]=sensorID;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}

/*Axes to tare: 0x07 -> tare around all exes; 0x04 -> tare around z*/
/*Vector to tare: 0 -> rotation vector; 1-> game vector; 2 -> geo vectot*/

void BNO085_Command_TareNow(BNO085 *myIMU, uint8_t axesToTare, uint8_t vectorToTare)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_TARE;
	myIMU->BNO085_Send_Buffer[7]=0x00;
		
	myIMU->BNO085_Send_Buffer[8]=axesToTare;
	myIMU->BNO085_Send_Buffer[9]=vectorToTare;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}

/*Writes to FRS the current tare*/
void BNO085_Command_PersistTare(BNO085 *myIMU)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_TARE;
	myIMU->BNO085_Send_Buffer[7]=0x01;
		
	myIMU->BNO085_Send_Buffer[8]=0;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}


void BNO085_Command_SetReorientation(BNO085 *myIMU, double quatX, double quatY, double quatZ, double quatW)
{
	int16_t quatMSB, quatLSB;
	int32_t convertedQuaternion;
	
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_TARE;
	myIMU->BNO085_Send_Buffer[7]=0x02;
	
	convertedQuaternion= floatToQ(quatX, 14);
	quatLSB = ((int16_t) convertedQuaternion) & 0xFF;
	quatMSB = ((int16_t) convertedQuaternion >> 8)&0xFF;
	myIMU->BNO085_Send_Buffer[8]=(int8_t)quatLSB;
	myIMU->BNO085_Send_Buffer[9]=(int8_t)quatMSB;

	convertedQuaternion= floatToQ(quatY, 14);
	quatLSB = ((int16_t) convertedQuaternion) & 0xFF;
	quatMSB = ((int16_t) convertedQuaternion >> 8)&0xFF;
	myIMU->BNO085_Send_Buffer[10]=(int8_t)quatLSB;
	myIMU->BNO085_Send_Buffer[11]=(int8_t)quatMSB;

	convertedQuaternion= floatToQ(quatZ, 14);
	quatLSB = ((int16_t) convertedQuaternion) & 0xFF;
	quatMSB = ((int16_t) convertedQuaternion >> 8)&0xFF;
	myIMU->BNO085_Send_Buffer[12]=(int8_t)quatLSB;
	myIMU->BNO085_Send_Buffer[13]=(int8_t)quatMSB;
	
	convertedQuaternion= floatToQ(quatW, 14);
	quatLSB = ((int16_t) convertedQuaternion) & 0xFF;
	quatMSB = ((int16_t) convertedQuaternion >> 8)&0xFF;
	myIMU->BNO085_Send_Buffer[14]=(int8_t)quatLSB;
	myIMU->BNO085_Send_Buffer[15]=(int8_t)quatMSB;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}

void BNO085_Command_Initialize(BNO085 *myIMU)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_INITIALIZE;
	myIMU->BNO085_Send_Buffer[7]=0x01;
		
	myIMU->BNO085_Send_Buffer[8]=0;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}

void BNO085_Command_SaveDCD(BNO085 *myIMU)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_SAVE_DCD;
	myIMU->BNO085_Send_Buffer[7]=0;
		
	myIMU->BNO085_Send_Buffer[8]=0;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}

void BNO085_Command_ConfigureCalibration(BNO085 *myIMU, uint8_t accelerometer, uint8_t gyroscope, uint8_t magnetometer, uint8_t planar)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_ME_CALIBRATION;
	myIMU->BNO085_Send_Buffer[7]=accelerometer;
		
	myIMU->BNO085_Send_Buffer[8]=gyroscope;
	myIMU->BNO085_Send_Buffer[9]=magnetometer;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=planar;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}

void BNO085_Command_GetCalibrationStatus(BNO085 *myIMU)
{
		myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_ME_CALIBRATION;
	myIMU->BNO085_Send_Buffer[7]=0;
		
	myIMU->BNO085_Send_Buffer[8]=0;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=1;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}
void BNO085_Command_EnableFullCalibration(BNO085 *myIMU)
{
	BNO085_Command_ConfigureCalibration(myIMU,1,1,1,1);
	return;
}
void BNO085_Command_DisableFullCalibration(BNO085 *myIMU)
{
	BNO085_Command_ConfigureCalibration(myIMU,0,0,0,0);
	return;
}

void BNO085_Command_EnableAutoSaveDCD(BNO085 *myIMU)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_CONFIGURE_DCD;
	myIMU->BNO085_Send_Buffer[7]=0x01;
		
	myIMU->BNO085_Send_Buffer[8]=0;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}	
void BNO085_Command_DisableAutoSaveDCD(BNO085 *myIMU)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_CONFIGURE_DCD;
	myIMU->BNO085_Send_Buffer[7]=0x00;
		
	myIMU->BNO085_Send_Buffer[8]=0;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}	

void BNO085_Command_GetOscType(BNO085 *myIMU)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_OSCILLATOR;
	myIMU->BNO085_Send_Buffer[7]=0;
		
	myIMU->BNO085_Send_Buffer[8]=0;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}	

void BNO085_Command_ClearResetDCD(BNO085 *myIMU)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
		
	myIMU->BNO085_Send_Buffer[4]=SHTP_COMMAND_REQUEST;
	myIMU->BNO085_Send_Buffer[5]=myIMU->commandSequenceNumber;
	myIMU->commandSequenceNumber++;
	myIMU->BNO085_Send_Buffer[6]=SHTP_COMMAND_ID_CLEAR_DCD;
	myIMU->BNO085_Send_Buffer[7]=0;
		
	myIMU->BNO085_Send_Buffer[8]=0;
	myIMU->BNO085_Send_Buffer[9]=0;
	myIMU->BNO085_Send_Buffer[10]=0;
	myIMU->BNO085_Send_Buffer[11]=0;

	myIMU->BNO085_Send_Buffer[12]=0;
	myIMU->BNO085_Send_Buffer[13]=0;
	myIMU->BNO085_Send_Buffer[14]=0;
	myIMU->BNO085_Send_Buffer[15]=0;
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}	

void BNO085_GetCommandResponse(BNO085 *myIMU, uint16_t length)
{
	int count=0;
	
	if (length>BNO085_DEBUG_BUFFER_LENGTH)
	{
		length=BNO085_BUFFER_LENGTH;
	}
	
	while (count<length)
	{
		myIMU->BNO085_Command_Buffer[count]=myIMU->BNO085_Receive_Buffer[count];
		count++;
	}
	
	return;
}

/*FRS functions*/

uint8_t BNO085_FRS_PerformWriteOperation (BNO085 *myIMU, uint16_t wordsToWrite, uint16_t FRSType, uint16_t initialOffset)
{
	uint16_t currentOffset, dataLength;
	uint8_t initializeResponse=255, currentWriteResponse=255, writeCount=0;
	
	BNO085_FRS_InitializeWriteRequest(myIMU, wordsToWrite, FRSType);

	while (initializeResponse!=0)
	{
		dataLength = BNO085_DataAvailable(myIMU);
		while (dataLength==0)
		{
			dataLength = BNO085_DataAvailable(myIMU);
		}
		
		BNO085_ReceiveData(myIMU, dataLength);
		if (myIMU->BNO085_Receive_Buffer[4] == SHTP_FRS_WRITE_RESPONSE_ID)
		{
			initializeResponse=0;
		}
	}
	if (myIMU->BNO085_Receive_Buffer[5]!=4)
	{
		return 1;
	}
	currentOffset=initialOffset;
	
	while ((wordsToWrite-currentOffset)>0)
	{
		BNO085_FRS_WriteData_Request (myIMU, currentOffset, myIMU->BNO085_FRS_Write_Buffer[writeCount], myIMU->BNO085_FRS_Write_Buffer[writeCount+1]);
		writeCount=writeCount+2;
		currentOffset = currentOffset+2;
		currentWriteResponse=255;
		while (currentWriteResponse!=0)
		{
			dataLength = BNO085_DataAvailable(myIMU);
			while (dataLength==0)
			{
				dataLength = BNO085_DataAvailable(myIMU);
			}
			
			BNO085_ReceiveData(myIMU, dataLength);
			if (myIMU->BNO085_Receive_Buffer[4] == SHTP_FRS_WRITE_RESPONSE_ID)
			{
				currentWriteResponse=0;
			}
		}
		if (myIMU->BNO085_Receive_Buffer[5]!=0 && myIMU->BNO085_Receive_Buffer[5]!=3)
		{
			return 1;
		}
	}
	if (myIMU->BNO085_Receive_Buffer[5]==3)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void BNO085_FRS_InitializeWriteRequest(BNO085 *myIMU, uint16_t length, uint16_t FRSType)
{
	myIMU->BNO085_Send_Buffer[0]=10;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
	
	myIMU->BNO085_Send_Buffer[4]=SHTP_FRS_WRITE_REQUEST_ID;
	myIMU->BNO085_Send_Buffer[5]=0;
	myIMU->BNO085_Send_Buffer[6]=((uint8_t)(length & 0xFF));
	myIMU->BNO085_Send_Buffer[7]=((uint8_t)((length>>8) & 0xFF));
	
	myIMU->BNO085_Send_Buffer[8]=((uint8_t)(FRSType & 0xFF));
	myIMU->BNO085_Send_Buffer[9]=((uint8_t)((FRSType>>8) & 0xFF));
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,10, 1000);
}

void BNO085_FRS_WriteData_Request (BNO085 *myIMU, uint16_t offset, uint32_t data0, uint32_t data1)
{
	myIMU->BNO085_Send_Buffer[0]=16;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
	
	myIMU->BNO085_Send_Buffer[4]=SHTP_FRS_WRITE_DATA_REQUEST_ID;
	myIMU->BNO085_Send_Buffer[5]=0;
	myIMU->BNO085_Send_Buffer[6]=((uint8_t)(offset & 0xFF));
	myIMU->BNO085_Send_Buffer[7]=((uint8_t)((offset>>8) & 0xFF));
	
	myIMU->BNO085_Send_Buffer[8]=((uint8_t)(data0 & 0xFF));
	myIMU->BNO085_Send_Buffer[9]=((uint8_t)((data0>>8) & 0xFF));
	myIMU->BNO085_Send_Buffer[10]=((uint8_t)((data0>>16) & 0xFF));
	myIMU->BNO085_Send_Buffer[11]=((uint8_t)((data0>>24) & 0xFF));

	myIMU->BNO085_Send_Buffer[12]=((uint8_t)(data1 & 0xFF));
	myIMU->BNO085_Send_Buffer[13]=((uint8_t)((data1>>8) & 0xFF));
	myIMU->BNO085_Send_Buffer[14]=((uint8_t)((data1>>16) & 0xFF));
	myIMU->BNO085_Send_Buffer[15]=((uint8_t)((data1>>24) & 0xFF));
		
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,16, 1000);
}

uint8_t BNO085_FRS_ReadRequest (BNO085 *myIMU, uint16_t offset, uint16_t FRSType, uint16_t numberOfWords)
{
	uint16_t counter, dataLength;
	uint8_t status;
	
	myIMU->BNO085_Send_Buffer[0]=12;
	myIMU->BNO085_Send_Buffer[1]=0;
	myIMU->BNO085_Send_Buffer[2]=CHANNEL_CONTROL;
	myIMU->BNO085_Send_Buffer[3]=myIMU->sequenceNumber[CHANNEL_CONTROL];
	myIMU->sequenceNumber[CHANNEL_CONTROL]++;
	
	myIMU->BNO085_Send_Buffer[4]=SHTP_FRS_READ_REQUEST_ID;
	myIMU->BNO085_Send_Buffer[5]=0;
	myIMU->BNO085_Send_Buffer[6]=((uint8_t)(offset & 0xFF));
	myIMU->BNO085_Send_Buffer[7]=((uint8_t)((offset>>8) & 0xFF));
	
	myIMU->BNO085_Send_Buffer[8]=((uint8_t)(FRSType & 0xFF));
	myIMU->BNO085_Send_Buffer[9]=((uint8_t)((FRSType>>8) & 0xFF));
	myIMU->BNO085_Send_Buffer[10]=((uint8_t)(numberOfWords & 0xFF));
	myIMU->BNO085_Send_Buffer[11]=((uint8_t)((numberOfWords>>8) & 0xFF));
	
	HAL_I2C_Master_Transmit(myIMU->hi2cx, myIMU->address, myIMU->BNO085_Send_Buffer,12, 1000);
	
	counter = 0;
	while (counter<numberOfWords)
	{
		dataLength = BNO085_DataAvailable(myIMU);
		while (dataLength==0)
		{
			dataLength = BNO085_DataAvailable(myIMU);
		}
		
		BNO085_ReceiveData(myIMU, dataLength);
		if (myIMU->BNO085_Receive_Buffer[4] == SHTP_FRS_READ_RESPONSE_ID)
		{
			status = myIMU->BNO085_Receive_Buffer[5] & 0x07;
			if ((status!=0 && status != 3)&&(status!=6 && status!=7))
			{
				return 1;
			}
			else 
			{
				BNO085_FRS_GetReadResponse(myIMU, &(myIMU->BNO085_FRS_Read_Buffer[counter]),&(myIMU->BNO085_FRS_Read_Buffer[counter+1]));
				if (counter+1 >= numberOfWords)
				{
					myIMU->BNO085_FRS_Read_Buffer[counter+1]=0;
				}
			}
			counter = counter+2;
		}
	}
	return 0;
}

void BNO085_FRS_GetReadResponse (BNO085 *myIMU, uint32_t *data0, uint32_t *data1)
{
	uint8_t length;
	length = myIMU->BNO085_Receive_Buffer[5]>>3;
	*data0=0;
	*data1=1;
	if (length>0)
	{
		*data0 = (myIMU->BNO085_Receive_Buffer[10]<<24)|(myIMU->BNO085_Receive_Buffer[11]<<16)|(myIMU->BNO085_Receive_Buffer[9]<<8)|(myIMU->BNO085_Receive_Buffer[8]);
	}
	if (length>1)
	{
		*data1 = (myIMU->BNO085_Receive_Buffer[15]<<24)|(myIMU->BNO085_Receive_Buffer[14]<<16)|(myIMU->BNO085_Receive_Buffer[13]<<8)|(myIMU->BNO085_Receive_Buffer[12]);		
	}
	return;
}

uint8_t BNO085_FRS_RequestOrientation(BNO085* myIMU)
{
	uint8_t result;
	result = BNO085_FRS_ReadRequest(myIMU, 0, 0x2D3E, 4);
	return result;
}
