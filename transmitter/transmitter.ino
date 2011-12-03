#include <RF12.h>
#include <Ports.h>
#include <ETG.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

static RF12Config config;

static ETGPacket packet;
static ETGSpecialPacket specialPacket;

typedef enum {NOTHING_TO_SEND = 0, NORMAL_READY_TO_SEND, SPECIAL_READY_TO_SEND} SendStatus;

SendStatus readyToSend = NOTHING_TO_SEND;

static byte dest, stack[RF12_MAXDATA], top, sendLen, cmd;

int value;

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

static void addCh (char* msg, char c) {
    byte n = strlen(msg);
    msg[n] = c;
}

static void addInt (char* msg, word v) {
    if (v >= 10)
        addInt(msg, v / 10);
    addCh(msg, '0' + v % 10);
}

#if DATAFLASH

#define DF_ENABLE_PIN   8           // PB0

#if FLASH_MBIT == 4
// settings for 0.5 Mbyte flash in JLv2
#define DF_BLOCK_SIZE   16          // number of pages erased at same time
#define DF_LOG_BEGIN    32          // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x0700      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x0800      // 2048 pages, i.e. 0.5 Mbyte
#define DF_DEVICE_ID    0x1F44      // see AT25DF041A datasheet
#define DF_PAGE_ERASE   0x20        // erase one block of flash memory
#endif

#if FLASH_MBIT == 8
// settings for 1 Mbyte flash in JLv2
#define DF_BLOCK_SIZE   16          // number of pages erased at same time
#define DF_LOG_BEGIN    32          // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x0F00      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x1000      // 4096 pages, i.e. 1 Mbyte
#define DF_DEVICE_ID    0x1F45      // see AT26DF081A datasheet
#define DF_PAGE_ERASE   0x20        // erase one block of flash memory
#endif

#if FLASH_MBIT == 16
// settings for 2 Mbyte flash in JLv3
#define DF_BLOCK_SIZE   256         // number of pages erased at same time
#define DF_LOG_BEGIN    512         // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x1F00      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x2000      // 8192 pages, i.e. 2 Mbyte
#define DF_DEVICE_ID    0x2020      // see M25P16 datasheet
#define DF_PAGE_ERASE   0xD8        // erase one block of flash memory
#endif

// structure of each page in the log buffer, size must be exactly 256 bytes
typedef struct {
    byte data [248];
    word seqnum;
    long timestamp;
    word crc;
} FlashPage;

// structure of consecutive entries in the data area of each FlashPage
typedef struct {
    byte length;
    byte offset;
    byte header;
    byte data[RF12_MAXDATA];
} FlashEntry;

static FlashPage dfBuf;     // for data not yet written to flash
static word dfLastPage;     // page number last written
static byte dfFill;         // next byte available in buffer to store entries

static byte df_present () {
    return dfLastPage != 0;
}

static void df_enable () {
    // digitalWrite(ENABLE_PIN, 0);
    bitClear(PORTB, 0);
}

static void df_disable () {
    // digitalWrite(ENABLE_PIN, 1);
    bitSet(PORTB, 0);
}

static byte df_xfer (byte cmd) {
    SPDR = cmd;
    while (!bitRead(SPSR, SPIF))
        ;
    return SPDR;
}

void df_command (byte cmd) {
    for (;;) {
        cli();
        df_enable();
        df_xfer(0x05); // Read Status Register
        byte status = df_xfer(0);
        df_disable();
        sei();
        // don't wait for ready bit if there is clearly no dataflash connected
        if (status == 0xFF || (status & 1) == 0)
            break;
    }

    cli();
    df_enable();
    df_xfer(cmd);
}

static void df_deselect () {
    df_disable();
    sei();
}

static void df_writeCmd (byte cmd) {
    df_command(0x06); // Write Enable
    df_deselect();
    df_command(cmd);
}

void df_read (word block, word off, void* buf, word len) {
    df_command(0x03); // Read Array (Low Frequency)
    df_xfer(block >> 8);
    df_xfer(block);
    df_xfer(off);
    for (word i = 0; i < len; ++i)
        ((byte*) buf)[(byte) i] = df_xfer(0);
    df_deselect();
}

void df_write (word block, const void* buf) {
    df_writeCmd(0x02); // Byte/Page Program
    df_xfer(block >> 8);
    df_xfer(block);
    df_xfer(0);
    for (word i = 0; i < 256; ++i)
        df_xfer(((const byte*) buf)[(byte) i]);
    df_deselect();
}

// wait for current command to complete
void df_flush () {
    df_read(0, 0, 0, 0);
}

static void df_wipe () {
    Serial.println("DF W");

    df_writeCmd(0xC7); // Chip Erase
    df_deselect();
    df_flush();
}

static void df_erase (word block) {
    Serial.print("DF E ");
    Serial.println(block);

    df_writeCmd(DF_PAGE_ERASE); // Block Erase
    df_xfer(block >> 8);
    df_xfer(block);
    df_xfer(0);
    df_deselect();
    df_flush();
}

static word df_wrap (word page) {
    return page < DF_LOG_LIMIT ? page : DF_LOG_BEGIN;
}

static void df_saveBuf () {
    if (dfFill == 0)
        return;

    dfLastPage = df_wrap(dfLastPage + 1);
    if (dfLastPage == DF_LOG_BEGIN)
        ++dfBuf.seqnum; // bump to next seqnum when wrapping

    // set remainder of buffer data to 0xFF and calculate crc over entire buffer
    dfBuf.crc = ~0;
    for (byte i = 0; i < sizeof dfBuf - 2; ++i) {
        if (dfFill <= i && i < sizeof dfBuf.data)
            dfBuf.data[i] = 0xFF;
        dfBuf.crc = _crc16_update(dfBuf.crc, dfBuf.data[i]);
    }

    df_write(dfLastPage, &dfBuf);
    dfFill = 0;

    // wait for write to finish before reporting page, seqnum, and time stamp
    df_flush();
    Serial.print("DF S ");
    Serial.print(dfLastPage);
    Serial.print(' ');
    Serial.print(dfBuf.seqnum);
    Serial.print(' ');
    Serial.println(dfBuf.timestamp);

    // erase next block if we just saved data into a fresh block
    // at this point in time dfBuf is empty, so a lengthy erase cycle is ok
    if (dfLastPage % DF_BLOCK_SIZE == 0)
        df_erase(df_wrap(dfLastPage + DF_BLOCK_SIZE));
}

static void df_append (const void* buf, byte len) {
    //FIXME the current logic can't append incoming packets during a save!

    // fill in page time stamp when appending to a fresh page
    if (dfFill == 0)
        dfBuf.timestamp = now();

    long offset = now() - dfBuf.timestamp;
    if (offset >= 255 || dfFill + 1 + len > sizeof dfBuf.data) {
        df_saveBuf();

        dfBuf.timestamp = now();
        offset = 0;
    }

    // append new entry to flash buffer
    dfBuf.data[dfFill++] = offset;
    memcpy(dfBuf.data + dfFill, buf, len);
    dfFill += len;
}

// go through entire log buffer to figure out which page was last saved
static void scanForLastSave () {
    dfBuf.seqnum = 0;
    dfLastPage = DF_LOG_LIMIT - 1;
    // look for last page before an empty page
    for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
        word currseq;
        df_read(page, sizeof dfBuf.data, &currseq, sizeof currseq);
        if (currseq != 0xFFFF) {
            dfLastPage = page;
            dfBuf.seqnum = currseq + 1;
        } else if (dfLastPage == page - 1)
            break; // careful with empty-filled-empty case, i.e. after wrap
    }
}

static void df_initialize () {
    // assumes SPI has already been initialized for the RFM12B
    df_disable();
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
}

static void discardInput () {
    while (Serial.read() >= 0)
        ;
}

static void df_dump () {
    struct { word seqnum; long timestamp; word crc; } curr;
    discardInput();
    for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
        if (Serial.read() >= 0)
            break;
        // read marker from page in flash
        df_read(page, sizeof dfBuf.data, &curr, sizeof curr);
        if (curr.seqnum == 0xFFFF)
            continue; // page never written to
        Serial.print(" df# ");
        Serial.print(page);
        Serial.print(" : ");
        Serial.print(curr.seqnum);
        Serial.print(' ');
        Serial.print(curr.timestamp);
        Serial.print(' ');
        Serial.println(curr.crc);
    }
}

static word scanForMarker (word seqnum, long asof) {
    word lastPage = 0;
    struct { word seqnum; long timestamp; } last, curr;
    last.seqnum = 0xFFFF;
    // go through all the pages in log area of flash
    for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
        // read seqnum and timestamp from page in flash
        df_read(page, sizeof dfBuf.data, &curr, sizeof curr);
        if (curr.seqnum == 0xFFFF)
            continue; // page never written to
        if (curr.seqnum >= seqnum && curr.seqnum < last.seqnum) {
            last = curr;
            lastPage = page;
        }
        if (curr.seqnum == last.seqnum && curr.timestamp <= asof)
            lastPage = page;
    }
    return lastPage;
}

static void df_replay (word seqnum, long asof) {
    word page = scanForMarker(seqnum, asof);
    Serial.print("r: page ");
    Serial.print(page);
    Serial.print(' ');
    Serial.println(dfLastPage);
    discardInput();
    word savedSeqnum = dfBuf.seqnum;
    while (page != dfLastPage) {
        if (Serial.read() >= 0)
            break;
        page = df_wrap(page + 1);
        df_read(page, 0, &dfBuf, sizeof dfBuf); // overwrites ram buffer!
        if (dfBuf.seqnum == 0xFFFF)
            continue; // page never written to
        // skip and report bad pages
        word crc = ~0;
        for (word i = 0; i < sizeof dfBuf; ++i)
            crc = _crc16_update(crc, dfBuf.data[i]);
        if (crc != 0) {
            Serial.print("DF C? ");
            Serial.print(page);
            Serial.print(' ');
            Serial.println(crc);
            continue;
        }
        // report each entry as "R seqnum time <data...>"
        byte i = 0;
        while (i < sizeof dfBuf.data && dfBuf.data[i] < 255) {
            if (Serial.available())
                break;
            Serial.print("R ");
            Serial.print(dfBuf.seqnum);
            Serial.print(' ');
            Serial.print(dfBuf.timestamp + dfBuf.data[i++]);
            Serial.print(' ');
            Serial.print((int) dfBuf.data[i++]);
            byte n = dfBuf.data[i++];
            while (n-- > 0) {
                Serial.print(' ');
                Serial.print((int) dfBuf.data[i++]);
            }
            Serial.println();
        }
        // at end of each page, report a "DF R" marker, to allow re-starting
        Serial.print("DF R ");
        Serial.print(page);
        Serial.print(' ');
        Serial.print(dfBuf.seqnum);
        Serial.print(' ');
        Serial.println(dfBuf.timestamp);
    }
    dfFill = 0; // ram buffer is no longer valid
    dfBuf.seqnum = savedSeqnum + 1; // so next replay will start at a new value
    Serial.print("DF E ");
    Serial.print(dfLastPage);
    Serial.print(' ');
    Serial.print(dfBuf.seqnum);
    Serial.print(' ');
    Serial.println(millis());
}

#else // DATAFLASH

#define df_present() 0
#define df_initialize()
#define df_dump()
#define df_replay(x,y)
#define df_erase(x)

#endif

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

static void df_disable () {
    // digitalWrite(ENABLE_PIN, 1);
    bitSet(PORTB, 0);
}

static void showHelp () {
    showString(helpText1);
    Serial.println("Current configuration:");
    rf12_config();
}

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
			case 'd': //Dimmer command: e.g. 128, 128, 0, ... , 0d sets the first 2 LED channels to 128/255 brightness

				if(top <= 16) { //i.e. there are 16 values on the stack before this one.
					for(int i = 0; i < top; i++) {
						packet.pwmLevels[i] = stack[i];
					}
					if(top < 16){
						for(int i = top; i < 16; i++) {
							packet.pwmLevels[i]=0;
						}
					}
					Serial.print( (int) top);
					Serial.println(F(" values saved."));
				} else {
					Serial.print(F("Need at most 16 values before a 'd' command"));
				}
				break;
			case 't': //time command. Only applies to dimmer commands but sets the time, in milliseconds, it ought to take.
				packet.fadeTime = value;
				break;
			case 'c': //Command type option
				packet.instrType = value;

				break;
/*			case 'e': //EL wire setting
				packet.elOn = value;
				break;  */
			case 'g': // set network group
                config.group = value;
				value = 0;
				memset(stack, 0, sizeof stack);
                saveConfig();
                break;
			case 'm': //memory command e.g. 5m selects macro index 5
				Serial.print(F("Memory commands haven't been implemented yet :s"));
				packet.memIndex = value;
				break;
			case 'i': //set node id
				config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
                saveConfig();
                break;

			case 'a':
				dest = value;
				break;
			case 'f':   // Full
			case 'o':   // Off 
				{  //Block so we can create the 'target' variable.
					byte target = ( c == 'f' ? 255 : 0);
					for(int i = 0; i < 16 ; i++){
						packet.pwmLevels[i] = target;
					}
					packet.which_trees = 0xff;
					break;
				}
			case 'q':  // Specify specific trees - e.g. 1,2,3,q - only trees 1, 2 and 3 will activate. Use 9 to activate all. 
				packet.which_trees = 0;
				for(int i = 0; i < top; i++){
					if(stack[i] == 9){
						packet.which_trees = 0xff;
					}else if(stack[i] > 0 && stack[i] <= 6){
						packet.which_trees = packet.which_trees | ( 1 << ( stack[i] - 1) );
					}
				}
				break;
			case 'p':  // Allocate a tree id to a specific jee node. 
				if(value <= 6){
					specialPacket.mode = SPECIAL_PACKET_TREE_ID;
					specialPacket.tree_id = value;
					if(dest){
						readyToSend = SPECIAL_READY_TO_SEND;
						Serial.print(F("Setting JeeNode "));
						Serial.print(dest);
						Serial.print(F(" to tree "));
						Serial.println(value);
					}else{
						Serial.println(F("Error - can't set tree id without first setting destination (use d)"));
					}
				}
                break;
            case 'x':   // Put all trees into identify mode
                specialPacket.mode = SPECIAL_PACKET_ID_MODE;
                readyToSend = SPECIAL_READY_TO_SEND;
                Serial.print(F("Putting all trees into id mode."));
                break;
			/*case 's':
				cmd = c;
				sendLen = top; //top is a pointer to the top of the stack, so it gives the number of values stored
				dest = value;
				memcpy(testbuf, stack, top);
				break;*/
		}
		value = 0;
		top = 0;
		memset(stack, 0, sizeof stack); //sets the stack to 0 as a letter has been reached
	} else if (c == ';'){
		sendLen = sizeof(packet);
		readyToSend = NORMAL_READY_TO_SEND;
		packet.print();
		Serial.println(F("Ready to send"));
	} else if (c >= 'A')
		showHelp();
}

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

void setup() { //this is complete
	Serial.begin(57600);
	Serial.print(F("\nWelcome to the ETG 2011 Wireless Control Interface (transmitter node)"));

	//Check to see if a config exists in the EEPROM. If not, use a default config then save it to EEPROM
	if (rf12_config()) {
		etg_rf12_setup();
    } else {
		Serial.println(F("Warning - node id not set"));
    }
	showHelp();
	packet.which_trees = 0xff;
}

MilliTimer ackWait;

void loop() { //this is a work in progress
    if (rf12_recvDone()){
		// Ignore... for now. Could check for ACK
	}
	if (Serial.available())
		handleInput(Serial.read());

	if (readyToSend){
		Serial.println(F(" Checking if can send"));
		bool sent = false;
		int send_attempts = 0;
		if(rf12_canSend()) {
			void* data;
			ETGPackedPacket packed;
			if(readyToSend == NORMAL_READY_TO_SEND){
				etg_pack(packet, packed);
				sendLen = sizeof(packed);
				data = &packed;
			}else{
				sendLen = sizeof(specialPacket);
				data = &specialPacket;
			}
			while(( ! sent ) && send_attempts < 20){
				Serial.print(F(" -> "));
				Serial.print((int) sendLen);
				Serial.print(F(" b"));
				byte header = RF12_HDR_ACK;
				if (dest){
                    Serial.print(F("** NOTE: Sending only to node "));
                    Serial.print (dest, DEC);
					header |= RF12_HDR_DST | dest;
				}
                Serial.println();
				rf12_sendStart(header, data, sendLen);
				cmd = 0;
				send_attempts++;
				rf12_sendWait(1); // Waits until we've completed sending before we look for an Ack
				// Wait for Ack
				ackWait.set(25);
				while(true){
					if(rf12_recvDone() )
					{
					   if(rf12_crc == 0  && rf12_len == 0 && (rf12_hdr & RF12_HDR_CTL) ){
						// We got an ACK
						sent = true;
						Serial.print(F("ACK after "));
						Serial.print(send_attempts);
						Serial.println(F(" attempts"));
						break;
					   }else{
						Serial.println(F("RF12 Recv - not valid"));
						Serial.print((int)rf12_crc);
						Serial.print(" ");
						Serial.print((int)rf12_len);
						Serial.print(" ");
						Serial.print((int)rf12_hdr);
					   }
					}
					if(ackWait.poll(0)){
						// We've run out of time to wait.
						break;
						Serial.println("resending");
					}
				}
			}
			if(!sent){
				Serial.println(F("Gave up sending after "));
				Serial.print(send_attempts);
				Serial.println(" attempts");
			}

			readyToSend = NOTHING_TO_SEND;
		}else{
			Serial.println(F("Not ready to send yet"));
		}
    }
}
