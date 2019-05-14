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
	return ((angle/M_PI)*180.00);
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
	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}

/*BNO085 Functions*/

/*Initialization functions*/
BNO085 *BNO085_CreateIMU (I2C_HandleTypeDef *hi2cx, uint8_t address, GPIO_TypeDef *reset_GPIOx, uint16_t reset_Pin, GPIO_TypeDef *boot_GPIOx, uint16_t nBOOT_Pin)
{
	BNO085 *myIMU;
	int i=0;
	myIMU=malloc(sizeof(BNO085));
	
	myIMU->hi2cx=hi2cx;
	myIMU->address=address;
	myIMU->reset_GPIOx=reset_GPIOx;
	myIMU->reset_Pin=reset_Pin;
	myIMU->boot_GPIOx=boot_GPIOx;
	myIMU->nBOOT_Pin=nBOOT_Pin;
	
	while (i<6)
	{
		myIMU->sequenceNumber[i]=0;
		i++;
	}
	myIMU->commandSequenceNumber=0;
	
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
	return 1;
}

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

void BNO085_EnableAbsoluteRotationVector(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_ROTATION_VECTOR, timeBetweenReports, 0);
	return;
}

void BNO085_EnableRelativeRotationVector(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_GAME_ROTATION_VECTOR, timeBetweenReports, 0);
	return;
}

void BNO085_EnableLinearAcc(BNO085 *myIMU, uint16_t timeBetweenReports)
{
	BNO085_SetFeatureCommand(myIMU, ID_LINEAR_ACCELERATION, timeBetweenReports, 0);
	return;
}

void BNO085_UpdateSensorReading(BNO085 *myIMU)
{
	uint16_t length;
	length = BNO085_DataAvailable(myIMU);
	
	if (length==0)
	{
		return;
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
			
			case ID_ROTATION_VECTOR:
				myIMU->sensor_readings.absoluteOrientation = BNO085_GetAbsoluteOrientation(myIMU);
			break;
			
			case ID_GAME_ROTATION_VECTOR:
				myIMU->sensor_readings.relativeOrientation = BNO085_GetRelativeOrientation(myIMU);
			break;
			
			case ID_LINEAR_ACCELERATION:
				myIMU->sensor_readings.linearAcceleration = BNO085_GetLinearAcceleration(myIMU);
			break;
			
			default:
				/*Sensor report IDs unhandled*/
			break;
		}
	}
	return;
}

motionDataType BNO085_GetAcceleration(BNO085 *myIMU)
{
	motionDataType accelerationData;
	accelerationData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	accelerationData.status=myIMU->BNO085_Receive_Buffer[11]&0x03;
	accelerationData.X =qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[14] << 8 | myIMU->BNO085_Receive_Buffer[13],8);
	accelerationData.Y =qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[16] << 8 | myIMU->BNO085_Receive_Buffer[15],8);
	accelerationData.Z =qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[18] << 8 | myIMU->BNO085_Receive_Buffer[17],8);
	return accelerationData;
}

motionDataType BNO085_GetLinearAcceleration(BNO085 *myIMU)
{
	motionDataType linearAccData;
	linearAccData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	linearAccData.status=myIMU->BNO085_Receive_Buffer[11]&0x03;
	linearAccData.X =qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[14] << 8 | myIMU->BNO085_Receive_Buffer[13],8);
	linearAccData.Y =qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[16] << 8 | myIMU->BNO085_Receive_Buffer[15],8);
	linearAccData.Z =qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[18] << 8 | myIMU->BNO085_Receive_Buffer[17],8);
	return linearAccData;
}

motionDataType BNO085_GetAngularRate(BNO085 *myIMU)
{
	motionDataType angularRateData;
	angularRateData.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	angularRateData.status=myIMU->BNO085_Receive_Buffer[11]&0x03;
	angularRateData.X =qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[14] << 8 | myIMU->BNO085_Receive_Buffer[13],9);
	angularRateData.Y =qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[16] << 8 | myIMU->BNO085_Receive_Buffer[15],9);
	angularRateData.Z =qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[18] << 8 | myIMU->BNO085_Receive_Buffer[17],9);
	return angularRateData;
}

orientationDataType BNO085_GetAbsoluteOrientation(BNO085 *myIMU)
{
	orientationDataType absoluteOrientation;
	quaternionType myQuaternion;
	myQuaternion.qi=qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[14] << 8 | myIMU->BNO085_Receive_Buffer[13],14);
	myQuaternion.qj=qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[16] << 8 | myIMU->BNO085_Receive_Buffer[15],14);
	myQuaternion.qk=qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[18] << 8 | myIMU->BNO085_Receive_Buffer[17],14);
	myQuaternion.qw=qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[20] << 8 | myIMU->BNO085_Receive_Buffer[19],14);
	absoluteOrientation.orientation=quaternionToEuler(myQuaternion);
	absoluteOrientation.status=myIMU->BNO085_Receive_Buffer[11]&0x03;
	absoluteOrientation.sequenceNumber=myIMU->BNO085_Receive_Buffer[10];
	absoluteOrientation.accuracy=radToDeg(qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[22] << 8 | myIMU->BNO085_Receive_Buffer[21],12));
	return absoluteOrientation;
}

orientationDataType BNO085_GetRelativeOrientation(BNO085 *myIMU)
{
	orientationDataType relativeOrientation;
	quaternionType myQuaternion;
	myQuaternion.qi=qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[14] << 8 | myIMU->BNO085_Receive_Buffer[13],14);
	myQuaternion.qj=qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[16] << 8 | myIMU->BNO085_Receive_Buffer[15],14);
	myQuaternion.qk=qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[18] << 8 | myIMU->BNO085_Receive_Buffer[17],14);
	myQuaternion.qw=qToFloat((uint16_t)myIMU->BNO085_Receive_Buffer[20] << 8 | myIMU->BNO085_Receive_Buffer[19],14);
	relativeOrientation.orientation=quaternionToEuler(myQuaternion);
	relativeOrientation.status=myIMU->BNO085_Receive_Buffer[11]&0x03;
	relativeOrientation.sequenceNumber=0;
	relativeOrientation.accuracy=0;
	return relativeOrientation;
}
