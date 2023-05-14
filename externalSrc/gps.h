#include "stdio.h"
#include "string.h"
#include "stdint.h"

#include "block_4Hz.h"

/*Function to read one single value from a GPS string*/
char* readGPSValue(char *inputString, uint8_t length, void* dataPointer, uint8_t dataType, uint8_t* error){

	uint8_t index = 0;
	char tempString[5];
	for(int k=0; k<5; k++){
		tempString[k]='A';
	}
	/*if length is not 0 only read up to length characters*/
	if(length > 0){
		while(*inputString != '.' && *inputString != ',' && inputString != '\0' && index<length){
			tempString[index]= *inputString;
			index++;
			inputString++;
		}
		if(index == length){
			if(dataType == 8){
				uint8_t* data8 = dataPointer;
				*data8 = atoi(tempString);
			}
			else if(dataType == 16){
				uint16_t* data16 = dataPointer;
				*data16 = atoi(tempString);
			}
			else if(dataType == 32){
				uint32_t* data32 = dataPointer;
				*data32 = atoi(tempString);
			}
			/*Char type*/
			else if(dataType == 0){
				char* data0 = dataPointer;
				*data0 = tempString[0];
			}
		}
		else
			error = -1;
	}
	/*unknown variable length*/
	else{
		while(*inputString != '.' && *inputString != ',' && inputString != '\0'){
			tempString[index]= *inputString;
			index++;
			inputString++;
		}
		if(index > 0){
			if(dataType == 8){
				uint8_t* data8 = dataPointer;
				*data8 = atoi(tempString);
			}
			else if(dataType == 16){
				uint16_t* data16 = dataPointer;
				*data16 = atoi(tempString);
			}
			else if(dataType == 32){
				uint32_t* data32 = dataPointer;
				*data32 = atoi(tempString);
			}
			/*char type*/
			else if(dataType == 0){
				char* data0 = dataPointer;
				*data0 = tempString[0];
			}
		}
		else
			error = -1;
	}
	return inputString;
}

/*Funtion to compile Block from string*/
int8_t gpsData(block_4Hz *device, char *bufferRx){

	int8_t error = 0;

	uint8_t NorthorSouth;
	uint8_t EastorWest;
	uint8_t Fix;
	uint8_t NSatellites;


	/*SAVE GNGGA DATA*/

	/*Skip "$GNGGA"*/
	char *dataString = strstr(bufferRx, "$GNGGA");
	if(dataString != NULL){

		/*Skip unused data*/
		for(int i = 0; i<7; i++){
			if(dataString != '\0')
				dataString++;
		}
		/*read every value if the string is not ended*/
		if(dataString != '\0'){
			dataString = readGPSValue(dataString, 2, &device->hour, 8, &error);
			dataString = readGPSValue(dataString, 2, &device->minute, 8, &error);
			dataString = readGPSValue(dataString, 2, &device->second, 8, &error);
			if(*dataString == '.')
				dataString++;
			dataString = readGPSValue(dataString, 0, &device->microsecond, 8, &error);
			dataString++;
			dataString = readGPSValue(dataString, 0, &device->Latitude1, 16, &error);
			if(*dataString == '.')
				dataString++;
			dataString = readGPSValue(dataString, 0, &device->Latitude2, 32, &error);
			dataString++;
			dataString = readGPSValue(dataString, 0, &NorthorSouth, 0, &error);
			dataString++;
			dataString = readGPSValue(dataString, 0, &device->Longitude1, 32, &error);
			if(*dataString == '.')
				dataString++;
			dataString = readGPSValue(dataString, 0, &device->Longitude2, 32, &error);
			dataString++;
			dataString = readGPSValue(dataString, 0, &EastorWest, 0, &error);
			dataString++;
			dataString = readGPSValue(dataString, 0, &Fix, 8, &error);
			dataString++;
			dataString = readGPSValue(dataString, 0, &NSatellites, 8, &error);
			dataString++;
			dataString = readGPSValue(dataString, 0, &device->HDOP1, 8, &error);
			if(*dataString == '.')
					dataString++;
			dataString = readGPSValue(dataString, 0, &device->HDOP2, 8, &error);
		}
		else
			error = -1;
	}
	else
		error = -1;
	/*GNGGA string structure*/
	//"$GNGGA, %2c%2c%2c.%3hu, %4hu.%5ld, %1c, %5ld.%5ld, %1c, %c, %c, %1c.%c, ", &device->hour, &device->minute, &device->second, &device->microsecond, &device->Latitude1, &device->Latitude2, &NorthorSouth, &device->Longitude1, &device->Longitude2, &EastorWest, &Fix, &NSatellites, &device->HDOP1, &device->HDOP2


	/*SAVE GPRMC DATA*/

	/*Skip "$GNRMC"*/
	dataString = strstr(bufferRx, "$GNRMC");

	if(dataString != NULL){

		//Skip unused data
		for(int i = 0; i<7; i++){
			dataString = strstr(dataString,",");
			if(dataString != NULL)
				dataString++;
		}

		/*read every value if the string is not ended*/
		if(dataString != '\0'){
			dataString = readGPSValue(dataString, 0, &device->Vel1, 8, &error);
			if(*dataString == '.')
				dataString++;
			dataString = readGPSValue(dataString, 0, &device->Vel2, 8, &error);
		}
		else
			error = -1;
	}
	else
		error = -1;


	/*AUX VARIABLES*/

	/*Decode char variables to bits*/
	if(NorthorSouth=='N')
		NorthorSouth = 1;
	else
		NorthorSouth = 0;

	if(EastorWest == 'E')
		EastorWest = 1;
	else
		EastorWest = 0;

	if(NSatellites>15)
		NSatellites=15;

	device->info = (NSatellites)|(Fix<<4)|(EastorWest<<6)|(NorthorSouth<<7);

	return error;
}
