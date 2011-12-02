#include <RF12.h>
#include <Ports.h>
#include <ETG.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define SET_ALL_LEDS(level) dimmer.setMulti(dimmer.PWM0, level, level, level, level, level, level, level, level, level, level, level, level, level, level, level, level, -1);
#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

PortI2C dimmerPort (3);
DimmerPlug dimmer (dimmerPort, 0x40);
RF12Config config;
ETGPacket packet;
//Port elWire (2);
byte fadeTime;
byte value, stack[RF12_MAXDATA], top, sendLen, dest, quiet;
byte startLevels[16];
byte currentLevels[16];
byte targetLevels[16];
int deltas[16];
int fadeTimeMillis;
unsigned long fadeStartMillis;
MilliTimer milliTimer;
boolean fadeRunning;

char helpText1[] PROGMEM = 
    "\n"
    "Available commands:" "\n"
    "  <nn> i     - set node ID (standard node ids are 1..26)" "\n"
    "               (or enter an uppercase 'A'..'Z' to set id)" "\n"
    "  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)" "\n"
    "  <nnn> g    - set network group (RFM12 only allows 212, 0 = any)" "\n"
    "  <n> c      - set collect mode (advanced, normally 0)" "\n"
    "  t          - broadcast max-size test packet, with ack" "\n"
    "  ...,<nn> a - send data packet to node <nn>, with ack" "\n"
    "  ...,<nn> s - send data packet to node <nn>, no ack" "\n"
    "  <n> l      - turn activity LED on PB1 on or off" "\n"
    "  <n> q      - set quiet mode (1 = don't report bad packets)" "\n"
    "Remote control commands:" "\n"
    "  <hchi>,<hclo>,<addr>,<cmd> f     - FS20 command (868 MHz)" "\n"
    "  <addr>,<dev>,<on> k              - KAKU command (433 MHz)" "\n"
;

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            Serial.print('\r');
        Serial.print(c);
    }
}

static void saveConfig() {
	// set up a nice config string to be shown on startup
	memset(config.msg, 0, sizeof config.msg);
    strcpy(config.msg, " ");
    
    byte id = config.nodeId & 0x1F;
    addCh(config.msg, '@' + id);
    strcat(config.msg, " i");
    addInt(config.msg, id);
    if (config.nodeId & COLLECT)
        addCh(config.msg, '*');
    
    strcat(config.msg, " g");
    addInt(config.msg, config.group);
    
    strcat(config.msg, " @ ");
    static word bands[4] = { 315, 433, 868, 915 };
    word band = config.nodeId >> 6;
    addInt(config.msg, bands[band]);
    strcat(config.msg, " MHz ");
    
    config.crc = ~0;
    for (byte i = 0; i < sizeof config - 2; ++i)
        config.crc = _crc16_update(config.crc, ((byte*) &config)[i]);

    // save to EEPROM
    for (byte i = 0; i < sizeof config; ++i) {
        byte b = ((byte*) &config)[i];
        eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
    }
    
    if (!rf12_config())
        Serial.println("config save failed");
}

static void showHelp () {

}

static void addCh (char* msg, char c) {
    byte n = strlen(msg);
    msg[n] = c;
}

static void addInt (char* msg, word v) {
    if (v >= 10)
        addInt(msg, v / 10);
    addCh(msg, '0' + v % 10);
}

/*static void df_initialize () {
    // assumes SPI has already been initialized for the RFM12B
    df_disable(); //Ian: Does this disable the radio so that it can be configured? Or something like that...
    pinMode(DF_ENABLE_PIN, OUTPUT);
    df_command(0x9F); // Read Manufacturer and Device ID
    word info = df_xfer(0) << 8;
    info |= df_xfer(0);
    df_deselect();

    if (info == DF_DEVICE_ID) {
        df_writeCmd(0x01);  // Write Status Register ...
        df_xfer(0);         // ... Global Unprotect
        df_deselect();

        scanForLastSave();
        
        Serial.print("DF I ");
        Serial.print(dfLastPage);
        Serial.print(' ');
        Serial.println(dfBuf.seqnum);
    
        // df_wipe();
        df_saveBuf(); //XXX
    }
}*/

static void handleInput (char c) { //this is fine, but needs to be extended to allow for actual instructions rather than just config
	if ('0' <= c && c <= '9')
        value = 10 * value + c - '0'; //this takes the ASCII digits of a decimal number and reads them from left to right, at the end 'value' holds the integer equal to the number represented in decimal.
		
	else if (c == ',') {
        if (top < sizeof stack)
            stack[top++] = value;
        value = 0;
	
	} else if ('a' <= c && c <='z') {
        Serial.print("> ");
        Serial.print((int) value);
        Serial.println(c);
        switch (c) {
			default:
				showHelp();
				break;
			case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
                value = value == 8 ? RF12_868MHZ :
                        value == 9 ? RF12_915MHZ : RF12_433MHZ;
                config.nodeId = (value << 6) + (config.nodeId & 0x3F);
                saveConfig();
                break;
			case 'g': // set network group
                config.group = value;
				value = 0;
				memset(stack, 0, sizeof stack);
                saveConfig();
                break;
			case 'i': //set node id
				config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
                saveConfig();
                break;
			
			/*case 'a':
			case 's':
				cmd = c;
				sendLen = top; //top is a pointer to the top of the stack, so it gives the number of values stored
				dest = value;
				memcpy(testbuf, stack, top);
				break;*/
		} 
		value = 0;
		top = 0;
		memset(stack, 0, sizeof stack); //sets the stack to 0 as a letter has been reached
	} else if (c >= 'A')
		showHelp();
}

void FadeToTarget(const ETGPacket packet){
    milliTimer.set(10);
    fadeTimeMillis = packet.fadeTime; //take the transmitted time (in deciseconds) and convert it to a useful time (in milliseconds)
                    
    fadeRunning = true;
    fadeStartMillis = millis();
    for(int i=0; i<16; i++){
	startLevels[i] = currentLevels[i];
	targetLevels[i] = packet.pwmLevels[i];
	deltas[i] = ((int) targetLevels[i] ) - currentLevels[i];
    }
}


void setup() { //this is complete
	Serial.begin(57600);
	Serial.print("\nWelcome to the ETG 2011 Wireless Control Interface (receiver node)\n");
	dimmer.begin();
    dimmer.setReg(dimmer.MODE2,0x14);
	
	//Check to see if a config exists in the EEPROM. If not, use a default config then save it to EEPROM
	if (rf12_config()) {
		etg_rf12_setup();
    } else {
		Serial.println("Warning - RF12 Not Set Up");
	}
	int startLevel = 0;
	Serial.println("About to set LEDs");
	SET_ALL_LEDS(startLevel);
	Serial.println("SETTING CURRENT LEVELS");
	for(int i = 0; i < 16; i++) {
		currentLevels[i] = startLevel;
	}
	//elWire.mode(OUTPUT);
	showHelp();
}

int patch(int val){
    return val;
}

//MilliTimer timer;

void loop() { //this is a work in progress
//	if(timer.poll(2000)){
//		Serial.println("Ping");
//	}
    if(Serial.available()){
	char c = Serial.read();
	
	if (c == 'u' || c == 'd'){
	    ETGPacket packet;
	    byte target = (c == 'u' ? 255 : 0);
	    packet.fadeTime = 10000;
	    for(int i=0; i<16; i++){
		packet.pwmLevels[i] = target;
	    }
	    FadeToTarget(packet);
	}
    }
    if (rf12_recvDone()) {  
        byte n = rf12_len;
        if (rf12_crc == 0) {
            Serial.print("OK");
        } else {
            Serial.print(" ?");
            if (n > 20) // print at most 20 bytes if crc is wrong
                n = 20;
        }
        if (config.group == 0) {
            Serial.print("G ");
            Serial.print((int) rf12_grp);
        }
        Serial.print(' ');
        Serial.print((int) rf12_hdr);
        for (byte i = 0; i < n; ++i) {
            Serial.print(' ');
            Serial.print((int) rf12_data[i]);
        }
        Serial.println();
        
		ETGPackedPacket packed;
		
        if (rf12_crc == 0 && n == sizeof(ETGPackedPacket)) {

            if (RF12_WANTS_ACK && (config.nodeId & COLLECT) == 0) {
                Serial.println(" -> ack");
                rf12_sendStart(RF12_ACK_REPLY, 0, 0);
            }
            memcpy(&packed, (void*) rf12_data, sizeof(ETGPackedPacket));
			etg_unpack(packed, packet);
            
			Serial.print("Packet:");
			if(packet.instrType){
				Serial.print("Mem[");
				Serial.print(packet.memIndex);
				Serial.print("] ");
			}else{
				Serial.print("Dim:[");
				for(int i=0;i<16;i++){
					if(i){
						Serial.print(",");
					}
					Serial.print((int) packet.pwmLevels[i]);
				}
				Serial.print("]");
			}
			Serial.print("EL Wire ");
			if(packet.elOn){
				Serial.print("On");
			}else{
				Serial.print("Off");
			}
			Serial.print(" Fade Time ");
			Serial.print(packet.fadeTime);
			Serial.println("ms");
			
            if(packet.instrType) { // case of memory instruction
                    Serial.print("Memory instructions haven't been implemented...");
            } else { //case of fade instruction
                    //elWire.digiWrite(!packet.elOn);
		    FadeToTarget(packet);
            }
        }
    }
    if(fadeRunning){
        if(milliTimer.poll(10)){ //Returns true every 10 milliseconds;
            unsigned long fadeHasRunFor = millis() - fadeStartMillis;
            if(fadeHasRunFor >= fadeTimeMillis){
                //We're done!
                for(int i=0; i<16; i++){
                    currentLevels[i] = targetLevels[i];
                    milliTimer.set(0);
                    fadeRunning = false;
                }
            }else{
		Serial.print(fadeHasRunFor);
		Serial.print(" ");
		Serial.print(deltas[0]);
		Serial.print(" ");
                unsigned long ratio = ( fadeHasRunFor * 256 ) / fadeTimeMillis;
		Serial.print(ratio);
                for(int i=0; i<16; i++){
                    currentLevels[i] = ( ( (long) deltas[i]  ) * ratio / 256)  + startLevels[i];
                }
		Serial.print(" ");
		Serial.println((int) currentLevels[0]);
            }
	    /*
	    for(int i = 0; i < 4 * 16; i++){
		Serial.print(BACKSPACE);
	    }
	    for(int i = 0; i < 16; i++){
		Serial.print(" ");
		PrintByteWithLeadingZeroes(currentLevels[patch(i)]);
	    } */
            dimmer.setMulti(dimmer.PWM0, currentLevels[patch( 0)], currentLevels[patch( 1)],
                                         currentLevels[patch( 2)], currentLevels[patch( 3)],
                                         currentLevels[patch( 4)], currentLevels[patch( 5)],
                                         currentLevels[patch( 6)], currentLevels[patch( 7)],
                                         currentLevels[patch( 8)], currentLevels[patch( 9)],
                                         currentLevels[patch(10)], currentLevels[patch(11)],
                                         currentLevels[patch(12)], currentLevels[patch(13)],
                                         currentLevels[patch(14)], currentLevels[patch(15)], -1);
        }
    }
}