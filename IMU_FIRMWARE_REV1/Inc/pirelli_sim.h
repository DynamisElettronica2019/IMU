#ifndef __PIRELLI_SIM_H
#define __PIRELLI_SIM_H

#include "main.h"

/*Typedefs*/

typedef enum
{
	IN_RANGE, OUT_OF_RANGE
}statusTypedef;

typedef enum
{
	TYPE_UNDEFINED, RIM, TYRE_LEFT, TYRE_MIDDLE, TYRE_RIGHT
}typeTypedef;

typedef enum
{
	LOCATION_UNDEFINED, LEFT, RIGHT
}locationTypedef;

typedef enum
{
	AXLE_UNDEFINED, FRONT, REAR
}axleTypedef;

typedef enum
{
	PSN_MUTE, PSN_ADVERTISING
}muteTypedef;

typedef struct												//Range accettabili:
{
	uint32_t UID;												//Da 0 a 0xFFFFFFFF
	uint16_t Pressure;									//Da 0 a 2047
	statusTypedef Measurement_Status;		//Vedi typedef status
	int16_t Temperature;								//Da -78 a 177
	uint8_t Service_Counter;						//Da 0 a 255 ciclico
}advertiseMeasTypedef;

typedef struct												//Range accettabili:
{
	uint32_t UID;												//Da 0 a 0xFFFFFFFF
	locationTypedef Location;						//Vedi typedef Location
	axleTypedef Axle;										//Vedi typedef axle
	typeTypedef Type;										//Vedi typedef type
	int8_t RSSI;												//Da -127 a 0
	muteTypedef Mute;										//Vedi typedef mute
	uint16_t Battery_Voltage;						//Da 100 a 355		
	uint8_t Service_Counter;						//Da 0 a 255 ciclico
}advertiseSensorStatusTypedef;

/*Function prototypes*/

uint8_t* AdvertiseMeasToArray(advertiseMeasTypedef myAdvertiseMeas, uint8_t* myArray);
uint8_t* AdvertiseSensorStatusToArray(advertiseSensorStatusTypedef myAdvertiseSensorStatus, uint8_t* myArray);

#endif
