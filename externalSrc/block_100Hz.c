typedef struct{
	uint16_t rpm;
	uint16_t tps;

	uint16_t accelX;
	uint16_t accelY;
	uint16_t accelZ;

	uint16_t gyroX;
	uint16_t gyroY;
	uint16_t gyroZ;

	uint8_t	potFSx;
	uint8_t potFDx;
	/*BIT A BIT DIVISION
	 * bit 0-3 : overflow pot_Front_Sx
	 * bit 4-7 : overflow pot_Front_Dx
	 */
	uint8_t potFOver;
	uint8_t potFAccuracy;

	uint8_t	potRSx;
	uint8_t potRDx;
	/*BIT A BIT DIVISION
	 * bit 0-3 : overflow pot_Rear_Sx
	 * bit 4-7 : overflow pot_Rear_Dx
	 */
	uint8_t potROver;
	uint8_t potRAccuracy;

	uint8_t countFSx;
	uint8_t countFDx;
	/*BIT A BIT DIVISION
	 * bit 0-3 : overflow count_Front_Sx
	 * bit 4-7 : overflow count_Front_Dx
	 */
	uint8_t countFOver;
	uint8_t dtF;
	uint8_t steering_encoder;

	uint8_t countRSx;
	uint8_t countRDx;
	/*BIT A BIT DIVISION
	 * bit 0-3 : overflow count_Rear_Sx
	 * bit 4-7 : overflow count_Rear_Dx
	 */
	uint8_t countROver;
	uint8_t dtR;
	uint8_t gear;


	uint8_t decoded_data[36];
	uint8_t coded_data[51];

	// Must be between 0 and 63
	uint8_t headerIndex;
}block_100Hz;

void codeData100Hz(block_100Hz *block){
	block->coded_data[0]=0x02;
	block->coded_data[1]=block->headerIndex;
	for (uint16_t i=0; i<sizeof(block->decoded_data)/3; i++){
		block->coded_data[4*i+2] = block->decoded_data[3*i]>>2 | 0x40;
		block->coded_data[4*i+3] = (block->decoded_data[3*i]&0x03)<<4 | block->decoded_data[3*i+1]>>4 | 0x40;
		block->coded_data[4*i+4] = (block->decoded_data[3*i+1]&0x0F)<<2 | block->decoded_data[3*i+2]>>6 | 0x40;
		block->coded_data[4*i+5] = (block->decoded_data[3*i+2] & 0x3F) | 0x40;
	}
	block->coded_data[50]=0x03;
}

void setDecodedData100Hz(block_100Hz *block){
	block->headerIndex=0x3F;
	block->decoded_data[0] = block->rpm>>8;
	block->decoded_data[1] = block->rpm&0xFF;
	block->decoded_data[2] = block->tps>>8;
	block->decoded_data[3] = block->tps&0xFF;
	block->decoded_data[4] = block->accelX>>8;
	block->decoded_data[5] = block->accelX&0xFF;
	block->decoded_data[6] = block->accelY>>8;
	block->decoded_data[7] = block->accelY&0xFF;
	block->decoded_data[8] = block->accelZ>>8;
	block->decoded_data[9] = block->accelZ&0xFF;
	block->decoded_data[10] = block->gyroX>>8;
	block->decoded_data[11] = block->gyroX&0xFF;
	block->decoded_data[12] = block->gyroY>>8;
	block->decoded_data[13] = block->gyroY&0xFF;
	block->decoded_data[14] = block->gyroZ>>8;
	block->decoded_data[15] = block->gyroZ&0xFF;
	block->decoded_data[16] = block->potFSx;
	block->decoded_data[17] = block->potFDx;
	block->decoded_data[18] = block->potFOver;
	block->decoded_data[19] = block->potFAccuracy;
	block->decoded_data[20] = block->potRSx;
	block->decoded_data[21] = block->potRDx;
	block->decoded_data[22] = block->potROver;
	block->decoded_data[23] = block->potRAccuracy;
	block->decoded_data[24] = block->countFSx;
	block->decoded_data[25] = block->countFDx;
	block->decoded_data[26] = block->countFOver;
	block->decoded_data[27] = block->dtF;
	block->decoded_data[28] = block->steering_encoder;
	block->decoded_data[29] = block->countRSx;
	block->decoded_data[30] = block->countRDx;
	block->decoded_data[31] = block->countROver;
	block->decoded_data[32] = block->dtR;
	block->decoded_data[33] = block->gear;

	/*Aggiungere eventuali byte di controllo, avanzano*/
	block->decoded_data[34]=0;
	block->decoded_data[35]=0;
}
