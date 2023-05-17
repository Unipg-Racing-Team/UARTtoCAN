typedef struct{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint16_t microsecond;

	/*BIT A BIT DIVISION
	 * bit 0-3 : number of satellites
	 * bit 4-5 : Fix quality
	 * bit 6 : East=1, West=0
	 * bit 7 : North=1, South=0
	 */
	uint8_t info;

	/*
	 * HDOP REBUILD FORMULE: HDOP1->HDOP2
	 */
	uint8_t HDOP1;
	uint8_t HDOP2;

	/*
	 * LATITUDE REBUILD FORMULA: TO-DO
	 */
	uint16_t Latitude1;
	uint32_t Latitude2;

	/*
	 * LONGITUDE REBUILD FORMULA: TO-DO
	 */
	uint32_t Longitude1;
	uint32_t Longitude2;

	/*
	 * VEL REBUILD FORMULA: Vel1->Vel2
	 */
	uint8_t Vel1;
	uint8_t Vel2;

	uint8_t decoded_data[24];
	uint8_t coded_data[35];

	// Must be between 0 and 63
	uint8_t headerIndex;
}block_4Hz;

void codeData4Hz(block_4Hz *block){
	block->coded_data[0] = 0x02;
	block->coded_data[1]=block->headerIndex;
	for (uint16_t i=0; i<sizeof(block->decoded_data)/3; i++){
		block->coded_data[4*i+2] = block->decoded_data[3*i]>>2 | 0x40;
		block->coded_data[4*i+3] = (block->decoded_data[3*i]&0x03)<<4 | block->decoded_data[3*i+1]>>4 | 0x40;
		block->coded_data[4*i+4] = (block->decoded_data[3*i+1]&0x0F)<<2 | block->decoded_data[3*i+2]>>6 | 0x40;
		block->coded_data[4*i+5] = (block->decoded_data[3*i+2] & 0x3F) | 0x40;
	}
	block->coded_data[34] = 0x03;
}

void setDecodedData4Hz(block_4Hz *block){
	block->headerIndex=0x04;
	block->decoded_data[0] = block->hour;
	block->decoded_data[1] = block->minute;
	block->decoded_data[2] = block->second;
	block->decoded_data[3] = block->microsecond>>8;
	block->decoded_data[4] = block->microsecond&0xFF;
	block->decoded_data[5] = block->info;
	block->decoded_data[6] = block->HDOP1;
	block->decoded_data[7] = block->HDOP2;
	block->decoded_data[8] = block->Latitude1>>8;
	block->decoded_data[9] = block->Latitude1&0xFF;
	block->decoded_data[10] = block->Latitude2>>24;
	block->decoded_data[11] = (block->Latitude2>>16)&0xFF;
	block->decoded_data[12] = (block->Latitude2>>8)&0xFF;
	block->decoded_data[13] = block->Latitude2&0xFF;
	block->decoded_data[14] = block->Longitude1>>24;
	block->decoded_data[15] = (block->Longitude1>>16)&0xFF;
	block->decoded_data[16] = (block->Longitude1>>8)&0xFF;
	block->decoded_data[17] = block->Longitude1&0xFF;
	block->decoded_data[18] = block->Longitude2>>24;
	block->decoded_data[19] = (block->Longitude2>>16)&0xFF;
	block->decoded_data[20] = (block->Longitude2>>8)&0xFF;
	block->decoded_data[21] = block->Longitude2&0xFF;
	block->decoded_data[22] = block->Vel1;
	block->decoded_data[23] = block->Vel2;
}
