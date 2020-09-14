#include "main.h"
#include "pirelli_sim.h"

/*Functions*/

uint8_t* AdvertiseMeasToArray(advertiseMeasTypedef myAdvertiseMeas, uint8_t* myArray)
{
	myArray[0] = (myAdvertiseMeas.UID>>0) & 0xFF;
	myArray[1] = (myAdvertiseMeas.UID>>8) & 0xFF;
	myArray[2] = (myAdvertiseMeas.UID>>16) & 0xFF;
	myArray[3] = (myAdvertiseMeas.UID>>24) & 0xFF;

	myArray[4] = myAdvertiseMeas.Pressure & 0xFF;
	myArray[5] = ((myAdvertiseMeas.Pressure>>8) & 0x07) | ((myAdvertiseMeas.Measurement_Status&0x01)<<7);
	myArray[6] = (myAdvertiseMeas.Temperature-50)&0xFF;
	myArray[7] = myAdvertiseMeas.Service_Counter;

	return myArray;
}

uint8_t* AdvertiseSensorStatusToArray(advertiseSensorStatusTypedef myAdvertiseSensorStatus, uint8_t* myArray)
{
	myArray[0] = (myAdvertiseSensorStatus.UID>>0) & 0xFF;
	myArray[1] = (myAdvertiseSensorStatus.UID>>8) & 0xFF;
	myArray[2] = (myAdvertiseSensorStatus.UID>>16) & 0xFF;
	myArray[3] = (myAdvertiseSensorStatus.UID>>24) & 0xFF;

	myArray[4] = (myAdvertiseSensorStatus.Location & 0x03) | ((myAdvertiseSensorStatus.Axle & 0x07)<<2) | ((myAdvertiseSensorStatus.Type & 0x07)<<5);
	myArray[5] = ((myAdvertiseSensorStatus.RSSI+63) & 0x7F) | ((myAdvertiseSensorStatus.Mute & 0x01)<<7);
	myArray[6] = (myAdvertiseSensorStatus.Battery_Voltage-100)&0xFF;
	myArray[7] = myAdvertiseSensorStatus.Service_Counter;

	return myArray;
}


