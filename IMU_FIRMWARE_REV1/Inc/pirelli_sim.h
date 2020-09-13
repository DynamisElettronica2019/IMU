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

typedef struct
{
	uint32_t UID;
	uint16_t Pressure;					//Usati solo 11 bit
	statusTypedef Measurement_Status;
	int8_t Temperature; 				//Inviata con offset di 50°C
	uint8_t Service_Counter;
}advertiseMeasTypedef;

typedef struct
{
	uint32_t UID;
	locationTypedef Location;
	axleTypedef Axle;
	typeTypedef Type;
	int8_t RSSI;
	muteTypedef Mute;
	uint8_t Battery_Voltage;		//Inviata con offset di 100 (centesimi di volt?)
	uint8_t Service_Counter;
}advertiseSensorStatusTypedef;

/*Function prototypes*/

uint8_t* AdvertiseMeasToArray(advertiseMeasTypedef myAdvertiseMeas, uint8_t* myArray);
uint8_t* AdvertiseSensorStatusToArray(advertiseSensorStatusTypedef myAdvertiseSensorStatus, uint8_t* myArray);

#endif
