#include "stdio.h"
#include "string.h"
#include "stdint.h"

typedef struct{
	uint8_t hourTime;
	uint8_t minTime;
	uint8_t secTime;
	uint16_t microsecTime;

	uint16_t latitude1;
	uint32_t latitude2;

	char northOrSouth;

	uint32_t longitude1;
	uint32_t longitude2;

	char eastOrWest;

	uint8_t fixQuality;

	uint8_t satellitesNumber;

	uint8_t HDOP1;
	uint8_t HDOP2;

	uint8_t vel1;
	uint8_t vel2;

	uint8_t info;
}GPS;

void gpsData(GPS *device, const char *bufferRx){
	char *dataString;
	dataString = strstr(bufferRx, "$GPGGA");
	sscanf(dataString, "$GPGGA, %2hd%2hd%2hd.%3d, %4d.%5ld, %1c, %5ld.%5ld, %c, %hd, %hd, %1hd.%hd, ", &device->hourTime, &device->minTime, &device->secTime, &device->microsecTime, &device->latitude1, &device->latitude2, &device->northOrSouth, &device->longitude1, &device->longitude2, &device->eastOrWest, &device->fixQuality, &device->satellitesNumber, &device->HDOP1, &device->HDOP2/*,&device.trash*/);

	char *velString;
	velString = strstr(bufferRx, "$GPRMC");
	velString+=39/*caratteri*/+6/*spazi*/;		//salto caratteri inutili
	sscanf(velString, " %3hd.%1hd, ", &device->vel1, device->vel2);

	/*AUX VARIABLES*/
	uint8_t eOw;
	uint8_t nOs;

	if(device->eastOrWest=='W')
		eOw=0;
	else
		eOw=1;

	if(device->northOrSouth=='S')
		nOs=0;
	else
		nOs=1;

	if(device->satellitesNumber>15)
		device->satellitesNumber=15;

	device->info=(device->satellitesNumber&0x0F)|((device->fixQuality&0x03)<<4)|(eOw<<6)|(nOs<<7);
}
