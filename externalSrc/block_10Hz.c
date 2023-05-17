typedef struct{
	uint8_t t_h2o;
	uint8_t t_air;
	uint8_t t_oil;

	uint8_t vbb;

	uint8_t lambda1_avg;
	uint8_t lambda1_raw;
	uint16_t klambda1;

	uint16_t injLow;
	uint16_t injHigh;

	uint8_t decoded_data[12];
	uint8_t coded_data[19];

	// Must be between 0 and 63
	uint8_t headerIndex;
}block_10Hz;

void codeData10Hz(block_10Hz *block){
	block->coded_data[0]=0x02;
	block->coded_data[1]=block->headerIndex;
	for (uint16_t i=0; i<sizeof(block->decoded_data)/3; i++){
		block->coded_data[4*i+2] = block->decoded_data[3*i]>>2 | 0x40;
		block->coded_data[4*i+3] = (block->decoded_data[3*i]&0x03)<<4 | block->decoded_data[3*i+1]>>4 | 0x40;
		block->coded_data[4*i+4] = (block->decoded_data[3*i+1]&0x0F)<<2 | block->decoded_data[3*i+2]>>6 | 0x40;
		block->coded_data[4*i+5] = (block->decoded_data[3*i+2] & 0x3F) | 0x40;
	}
	block->coded_data[18]=0x03;
}

void setDecodedData10Hz(block_10Hz *block){
	block->headerIndex=0x0A;
	block->decoded_data[0] = block->t_h2o;
	block->decoded_data[1] = block->t_air;
	block->decoded_data[2] = block->t_oil;
	block->decoded_data[3] = block->vbb;
	block->decoded_data[4] = block->lambda1_avg;
	block->decoded_data[5] = block->lambda1_raw;
	block->decoded_data[6] = block->klambda1>>8;
	block->decoded_data[7] = block->klambda1&0xFF;
	block->decoded_data[8] = block->injLow>>8;
	block->decoded_data[9] = block->injLow&0xFF;
	block->decoded_data[10] = block->injHigh>>8;
	block->decoded_data[11] = block->injHigh&0xFF;
}
