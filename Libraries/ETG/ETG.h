;typedef struct {
    byte nodeId;
    byte group;
    char msg[RF12_EEPROM_SIZE-4];
    word crc;
} RF12Config;

typedef struct {
	bool instrType; //0 if dimmer, 1 if memory
	bool elOn;
	byte pwmLevels[16];
	byte memIndex;
	unsigned int fadeTime; // time in deciseconds
} ETGPacket;


typedef struct {
	unsigned instrType : 1; //0 if dimmer, 1 if memory
	unsigned elOn : 1;
	unsigned pwmLevel_00 : 3;
	unsigned pwmLevel_01 : 3;
	unsigned pwmLevel_02 : 3;
	unsigned pwmLevel_03 : 3;
	unsigned pwmLevel_04 : 3;
	unsigned pwmLevel_05 : 3;
	unsigned pwmLevel_06 : 3;
	unsigned pwmLevel_07 : 3;
	unsigned pwmLevel_08 : 3;
	unsigned pwmLevel_09 : 3;
	unsigned pwmLevel_10 : 3;
	unsigned pwmLevel_11 : 3;
	unsigned pwmLevel_12 : 3;
	unsigned pwmLevel_13 : 3;
	unsigned pwmLevel_14 : 3;
	unsigned pwmLevel_15 : 3;
	unsigned fadeTime : 8; // time in deciseconds
} ETGPackedPacket;

int transmitToMilli(byte transmitTime) {
	int milliTime = ((int)transmitTime) * 100;
	return milliTime;
}

byte milliToTransmit(int milliTime) {
	byte transmitTime;
	milliTime /= 100;
	
	if(milliTime > 255) 
		transmitTime = 255;
	else
		transmitTime = milliTime;
	
	return transmitTime;
}

byte byte_to_threebit(byte source){
	return source >> 5;
}

byte threebit_to_byte(byte source){
	return (source << 5) | (source << 2) | (source >> 1);
}

void etg_pack(const ETGPacket& source, ETGPackedPacket& dest){
	dest.instrType = source.instrType ? 1 : 0;
	dest.elOn =      source.elOn      ? 1 : 0;
	dest.pwmLevel_00 = byte_to_threebit( source.pwmLevels[ 0]);
	dest.pwmLevel_01 = byte_to_threebit( source.pwmLevels[ 1]  );
	dest.pwmLevel_02 = byte_to_threebit( source.pwmLevels[ 2]  );
	dest.pwmLevel_03 = byte_to_threebit( source.pwmLevels[ 3]  );
	dest.pwmLevel_04 = byte_to_threebit( source.pwmLevels[ 4]  );
	dest.pwmLevel_05 = byte_to_threebit( source.pwmLevels[ 5]  );
	dest.pwmLevel_06 = byte_to_threebit( source.pwmLevels[ 6]  );
	dest.pwmLevel_07 = byte_to_threebit( source.pwmLevels[ 7]  );
	dest.pwmLevel_08 = byte_to_threebit( source.pwmLevels[ 8]  );
	dest.pwmLevel_09 = byte_to_threebit( source.pwmLevels[ 9]  );
	dest.pwmLevel_10 = byte_to_threebit( source.pwmLevels[10]  );
	dest.pwmLevel_11 = byte_to_threebit( source.pwmLevels[11]  );
	dest.pwmLevel_12 = byte_to_threebit( source.pwmLevels[12]  );
	dest.pwmLevel_13 = byte_to_threebit( source.pwmLevels[13]  );
	dest.pwmLevel_14 = byte_to_threebit( source.pwmLevels[14]  );
	dest.pwmLevel_15 = byte_to_threebit( source.pwmLevels[15]  );
	dest.fadeTime = milliToTransmit(source.fadeTime);
}

void etg_unpack(const ETGPackedPacket& source, ETGPacket& dest){
	dest.instrType = source.instrType;
	dest.elOn = source.elOn;
	dest.pwmLevels[ 0] = threebit_to_byte( source.pwmLevel_00 );
	dest.pwmLevels[ 1] = threebit_to_byte( source.pwmLevel_01 );
	dest.pwmLevels[ 2] = threebit_to_byte( source.pwmLevel_02 );
	dest.pwmLevels[ 3] = threebit_to_byte( source.pwmLevel_03 );		
	dest.pwmLevels[ 4] = threebit_to_byte( source.pwmLevel_04 );
	dest.pwmLevels[ 5] = threebit_to_byte( source.pwmLevel_05 );
	dest.pwmLevels[ 6] = threebit_to_byte( source.pwmLevel_06 );
	dest.pwmLevels[ 7] = threebit_to_byte( source.pwmLevel_07 );		
	dest.pwmLevels[ 8] = threebit_to_byte( source.pwmLevel_08 );
	dest.pwmLevels[ 9] = threebit_to_byte( source.pwmLevel_09 );
	dest.pwmLevels[10] = threebit_to_byte( source.pwmLevel_10 );
	dest.pwmLevels[11] = threebit_to_byte( source.pwmLevel_11 );		
	dest.pwmLevels[12] = threebit_to_byte( source.pwmLevel_12 );
	dest.pwmLevels[13] = threebit_to_byte( source.pwmLevel_13 );
	dest.pwmLevels[14] = threebit_to_byte( source.pwmLevel_14 );
	dest.pwmLevels[15] = threebit_to_byte( source.pwmLevel_15 );		
	dest.fadeTime = transmitToMilli(source.fadeTime);
}

/*
// RF12_DATA_RATE_3 and RF12_DATA_RATE_2.
enum rf12DataRates {
    RF12_DATA_RATE_CMD = 0xC600,
    RF12_DATA_RATE_9 = RF12_DATA_RATE_CMD | 0x02, // Approx 115200 bps
    RF12_DATA_RATE_8 = RF12_DATA_RATE_CMD | 0x05, // Approx 57600 bps
    RF12_DATA_RATE_7 = RF12_DATA_RATE_CMD | 0x06, // Approx 49200 bps
    RF12_DATA_RATE_6 = RF12_DATA_RATE_CMD | 0x08, // Approx 38400 bps
    RF12_DATA_RATE_5 = RF12_DATA_RATE_CMD | 0x11, // Approx 19200 bps
    RF12_DATA_RATE_4 = RF12_DATA_RATE_CMD | 0x23, // Approx 9600 bps
    RF12_DATA_RATE_3 = RF12_DATA_RATE_CMD | 0x47, // Approx 4800 bps
    RF12_DATA_RATE_2 = RF12_DATA_RATE_CMD | 0x91, // Approx 2400 bps
    RF12_DATA_RATE_1 = RF12_DATA_RATE_CMD | 0x9E, // Approx 1200 bps
    RF12_DATA_RATE_DEFAULT = RF12_DATA_RATE_7,
};
*/

void etg_rf12_setup(){
  // rf12_control(RF12_DATA_RATE_5);
  // Serial.println(	"Speed set to Data Rate 5");
  
  // This wasn't working, no idea why
}

void PrintByteWithLeadingZeroes(byte val){
    if(val<100){
	Serial.print(" ");
    }
    if(val<10){
	Serial.print(" ");
    }
    Serial.print((int) val);
}

static char BACKSPACE = 8;