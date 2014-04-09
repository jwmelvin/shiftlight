// Code inspired by a post by ivan141 on the arduino forum 
// http://forum.arduino.cc/index.php/topic,8420.0.html
// Modified by Jason Melvin to 
// * use an Adafruit NeoPixel strip
// * use integer math
// * only work with the latest accumulated interrupt cycle
// * use micros for better resolution at high rpm
// * use an encoder and variable settings for brightness, color scheme, rpm setpoints
// * save states to EEPROM
// ** VERSION 2 **//
// * allow for adjustment of the config variables
// * 1k and 100 digit representation with 4-bit binary

#include <Adafruit_NeoPixel.h>
#include <Encoder.h>
#include <Bounce.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>

// serial functionality
#define SERIALDEBUG_MASTER 0 // master switch (requires one below too)
#define SERIALDEBUG_SLOW 1 // button + encoder feedback
#define SERIALDEBUG_LIVE 0 // rpm calculation feedback

#define milBlinkIntervalLEDs 50 // ms interval for blinking LEDs at redline
#define milBlinkIntervalBrake 250 // ms interval for blinking brakes at redline
#define milBlinkIntervalAdj 1000 // ms interval for adjustment blinking
#define numAVG 4 // number of tach interrupts to accumulate
#define milTIMEOUT_DELAY 10000 // exit setting mode if not adjusted for this long (ms)
#define milBUTTON_HOLD 1000 // how long you must hold the button to get into setting mode
#define OFFSET_EEPROM 0 // memory start address
#define CONFIG_START 16 //memory address for config
#define encCHUNK 4 // decrease the sensitivity of the encoder

// list of states
	enum mode_t  { STOP , RUN , BRIGHT , SET , ADJ , BRIEF };
	enum wipe_t  { SOLID , GROW , WIPE };
	enum brake_t { NONE , BRAKE };
	
	uint8_t configBright = 32;
	mode_t mode = RUN;
	uint8_t iConfig = 0;
	
// config parameters saved to EEPROM
	#define CONFIG_VERSION "s02" // version number 
						//***** (make sure to increment when changing structure)
	#define numCONFIGS 5

	struct configStruct { // structure to hold settings
		char 		version[4]; // settings structure version
		uint16_t RPMs[numCONFIGS][4];
		uint8_t 	colors[numCONFIGS][4];
		wipe_t	wipe[numCONFIGS];
		brake_t	brake[numCONFIGS];
	};
	  
	configStruct configurationDefault = { //define defaults
		CONFIG_VERSION,
		{
			{ 5800, 7000, 7800, 8000}, 
			{ 5800, 7000, 7800, 8000}, 
			{ 5800, 7000, 7800, 8000}, 
			{ 5800, 7000, 7800, 8000}, 
			{ 1000, 1000, 7000, 7800}
		},
		{
			{  0 ,  	85, 	170, 	170	},
			{  0 ,  	85,   170, 	170	},
			{  0 , 	0,		100,	170	},
			{  0 , 	0 , 	100,	170	},
			{  0 ,  	0,   	170, 	85		}
		},
		{ WIPE , SOLID , WIPE , SOLID , WIPE },
		{ BRAKE , BRAKE , BRAKE , BRAKE , NONE }
	};
	configStruct config = configurationDefault; // initialize to default params
  
	// dot // bar // wipe // flash // wipe mode // "brake" during wipe
	/* 
	colors:	0     =	green;
				85   =	red;
				170 =	blue;
	
	wipe mode:	0 = solid; 
					1 = growing; 
					2 = wipe (from bar>wipe)
	*/

	/*  // place holder for a possible matrix to scale brightness of individual segments
	float configBrightnessScalers[numCONFIGS][4]	= {
		{ 0.7, 1.0, 1.0, 1.0}, 
		{ 0.7, 1.0, 1.0, 1.0}, 
		{ 0.7, 1.0, 1.0, 1.0}, 
		{ 0.6, 0.6, 1.0, 1.0}, 
		{ 0.3, 0.3, 0.6, 1.0} 
	};
	*/

	void configRead (){
		if (EEPROM.read(OFFSET_EEPROM + 0) <= numCONFIGS){
			iConfig = EEPROM.read(OFFSET_EEPROM + 0); // recall  configuration index
		}
		configBright = EEPROM.read(OFFSET_EEPROM + 1); // recall brightness
		// recall matrix of configs
		if (EEPROM.read(OFFSET_EEPROM + CONFIG_START + 0) == CONFIG_VERSION[0] &&
				EEPROM.read(OFFSET_EEPROM + CONFIG_START + 1) == CONFIG_VERSION[1] &&
				EEPROM.read(OFFSET_EEPROM + CONFIG_START + 2) == CONFIG_VERSION[2] ) {
			EEPROM_readAnything(OFFSET_EEPROM + CONFIG_START, config);
		}
		else {
			configWrite();
		}
	}

	void configWrite (){
		EEPROM_writeAnything(OFFSET_EEPROM + CONFIG_START, config);
	}
	void configDefault(){
	  config = configurationDefault;
	}


// hardware setup
#define numLEDs 8
#define pinLED_DATA 4
#define pinTACH 6 // just for info; defined in interrupt setup below
#define pinBRAKELIGHT 12
#define pinBUTTON 3
#define pinENC_A 8
#define pinENC_B 7

Adafruit_NeoPixel strip = Adafruit_NeoPixel(numLEDs, pinLED_DATA, NEO_GRB + NEO_KHZ800);
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)

// RPM Calculation variables
volatile uint32_t micTachInterrupt;
volatile uint32_t micTachInterrupt_last;
uint32_t rpmInst, rpmAvg;

// Blink
uint8_t blinkStateLEDs = HIGH;
uint32_t milBlinkPrevLEDs = 0;
uint8_t blinkStateBrake = HIGH;
uint32_t milBlinkPrevBrake = 0;
uint8_t blinkStateAdj = HIGH;
uint32_t milBlinkPrevAdj = 0;

// Encoder
Encoder myEnc(pinENC_A, pinENC_B);
int32_t encOld  = 0;
Bounce button = Bounce( pinBUTTON, 5);

// time tracking for config exit delay and display
uint32_t milTimeout;

// config adjustment variables
uint8_t iConfigAdj; // index for config variable
enum adjust_t { COLOR , RPM };
adjust_t parmAdj = COLOR;

// Init function
void setup()
{
	pinMode(pinBRAKELIGHT, OUTPUT);
	digitalWrite(pinBRAKELIGHT, LOW);
	pinMode(pinBUTTON, INPUT);
	digitalWrite(pinBUTTON, HIGH); // enable pullup resistor
	configRead(); // recall config if available
	
	if(SERIALDEBUG_MASTER) {
		Serial.begin(115200);
		Serial.println("Recalled parameters:");
		Serial.print("mode: ");	Serial.println(mode);
		Serial.print(" brightness: "); Serial.println(configBright);
		Serial.println(" iConfig: "); Serial.println(iConfig);
	}
	
	// set up the LED strip
	strip.begin();
	strip.setBrightness(8); 
	rainbow(2); // a colorful intro on powerup
	strip.setBrightness(configBright); 
	strip.show();
	
	// start tachometer interrupts
	attachInterrupt(1, tachMonitor, RISING); 
}

// Main loop
void loop() {
	processTach();  // process log entries to calculate rpm
	inputSerial();  // read serial input if enabled and available
	inputButton();  // check for button presses
	inputEncoder(); // check for encoder motion
	//if (!modeSet) updateDisplay();  // update display based on processed tachometer reading
	checkTimeout();
	updateDisplay(); // update display 
	delay(10);  //wait a while // why?
}


void updateDisplay() {
	if (mode == SET || mode == ADJ || mode == BRIGHT || mode == BRIEF){
		if (mode == SET || mode == BRIGHT || mode == BRIEF) {
			showColors();
		}
		else if (mode == ADJ){
			if (parmAdj == COLOR) {
				showConfigColor();
			}
			else if (parmAdj == RPM) {
				showConfigRPM();
			}
			// compute blink timer
			if( millis() - milBlinkPrevAdj > milBlinkIntervalAdj )  {
				blinkStateAdj = blinkStateAdj ? LOW : HIGH;
				milBlinkPrevAdj = millis();
			}
		}
	}
	else if (mode == RUN){
		//** RPM > redline
		if (rpmAvg >= config.RPMs[iConfig][3]){
			// blinking LEDs at redline
			colorBar(blinkStateLEDs ? Wheel(config.colors[iConfig][3]) : 0, numLEDs); 
			// blinking brake light at redline
			digitalWrite(pinBRAKELIGHT, blinkStateBrake ? HIGH : LOW); 
			// compute the blink timers
			if( millis() - milBlinkPrevLEDs > milBlinkIntervalLEDs )  {
				blinkStateLEDs = blinkStateLEDs ? LOW : HIGH;
				milBlinkPrevLEDs = millis();
			}
			if( millis() - milBlinkPrevBrake > milBlinkIntervalBrake )  {
				blinkStateBrake = blinkStateBrake ? LOW : HIGH;
				milBlinkPrevBrake= millis();
			}
		}
		else {
			// reset the blink timers so light is on immediately above redline
			blinkStateLEDs = HIGH;
			blinkStateBrake = HIGH;
			//** shift < RPM < redline
			if ( rpmAvg >= config.RPMs[iConfig][2] ){
				if( config.wipe[iConfig] == WIPE) barWipe(2); // wipe from previous color to [2]
				else if (config.wipe[iConfig] == GROW) barGrow(2); // growing bar of color [2]
				else colorBar(Wheel(config.colors[iConfig][2]), numLEDs);  // solid bar of color index [2] (wipes == 0)
				
				if( config.brake[iConfig] ) digitalWrite(pinBRAKELIGHT, HIGH ); // optionally flash the brakelight
				else digitalWrite(pinBRAKELIGHT, LOW);
			}
			else {
				digitalWrite(pinBRAKELIGHT, LOW);
				// pwr < RPM < shift
				if ( rpmAvg >= config.RPMs[iConfig][1] ){ 
					barGrow(1); // growing bar from pwr > shift
				}
				// start < RPM < pwr
				else if ( rpmAvg >= config.RPMs[iConfig][0] ) { 
					colorBar(Wheel(config.colors[iConfig][0]),1); // one light above first threshold
				}
				// RPM < start
				else { 
					colorBar(0,numLEDs);
				}
			}
		}
	}
	// if off, then show nothing
	else if (mode == STOP) colorBar(0, numLEDs);
	strip.show(); 
}

// read the latest interval of accumulated interrupts and compute RPM
void processTach() {
	uint32_t micTachInterval; // elapsed time for numAVG tach events
	uint32_t micTachInterrupt_local, micTachInterrupt_last_local;
	noInterrupts(); // disable interrupts while we copy the values
		micTachInterrupt_local =  micTachInterrupt;
		micTachInterrupt_last_local = micTachInterrupt_last;
	interrupts(); // reenable interrupts
	micTachInterval = micTachInterrupt_local - micTachInterrupt_last_local;
	if(SERIALDEBUG_MASTER && SERIALDEBUG_LIVE) { 
			Serial.print("t:"); Serial.print(micTachInterrupt); Serial.print("-"); Serial.println(micTachInterrupt_last);
	}
	
	if ( micTachInterval ){ // avoid divide by 0
		// this is based on an Acura NSX, adjust for other engines
		rpmInst = 20000000UL / micTachInterval * (uint32_t)numAVG; 
		rpmAvg = ( rpmAvg * 2UL + rpmInst ) / 3UL; // rolling average rpm
		if(SERIALDEBUG_MASTER && SERIALDEBUG_LIVE) { 
			Serial.print("micTachInterval: "); Serial.print(micTachInterval);
			Serial.print("  rpmInst: "); Serial.print(rpmInst);
			Serial.print("  rpmAvg: "); Serial.println(rpmAvg);
		}
	}
}

// Tach interrupt function
void tachMonitor(){
	static uint8_t iTachInterrupts;
	// accumulate numAVG interrupts before calculating RPM
	if ( iTachInterrupts < ( numAVG - 1) ) {
		iTachInterrupts++;
	}
	// once accumulated, record the time in microseconds
	else {
		iTachInterrupts = 0;
		micTachInterrupt_last = micTachInterrupt;
		micTachInterrupt = micros();
	}
}

// encoder read function
void inputEncoder() {
	int32_t encNew = myEnc.read();
	int32_t encChange = encNew - encOld;
	if ( abs(encChange) >= encCHUNK ) {
		encOld = encNew;
		milTimeout = millis();
		// if (SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.print("encChange="); Serial.println(encChange); }
		// if in setting mode, change the index of the configuration matrix
		if (mode == SET) {
			encChange /= encCHUNK; // use chunks to ensure fine control (move by a single index)
			if ( encChange > 0) { // increased
				iConfig = constrain( iConfig+1, 0, numCONFIGS - 1 );
			}
			else { // decreased
				iConfig = constrain( iConfig-1, 0, numCONFIGS - 1 );
			}
			if (SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.print("iConfig="); Serial.println(iConfig); }
		}
		// adjust RPM
		else if (mode == ADJ && parmAdj == RPM) {
			encChange /= encCHUNK;
			if ( encChange > 0) { // increased
				config.RPMs[iConfig][iConfigAdj] = 
					constrain (config.RPMs[iConfig][iConfigAdj] + 100, 0, 9000);
				// iConfig = constrain( iConfig+1, 0, numCONFIGS - 1 );
			}
			else { // decreased
				config.RPMs[iConfig][iConfigAdj] = 
					constrain (config.RPMs[iConfig][iConfigAdj] - 100, 0, 9000);
				// iConfig = constrain( iConfig-1, 0, numCONFIGS - 1 );
			}
			if (SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { 
				Serial.print("RPM for iConfig:"); Serial.print(iConfig);
				Serial.print(" pos:"); Serial.print(iConfigAdj);
				Serial.print(" = "); Serial.println(config.RPMs[iConfig][iConfigAdj]);
			}
		}
		// adjust color
		else if (mode == ADJ && parmAdj == COLOR) {
			encChange /= encCHUNK;
			if (iConfigAdj < 4){
				if ( encChange > 0) { // increased
					config.colors[iConfig][iConfigAdj] += 5;
				}
				else { // decreased
					config.colors[iConfig][iConfigAdj] -= 5;
				}
				if (SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { 
					Serial.print("Color for iConfig:"); Serial.print(iConfig);
					Serial.print(" pos:"); Serial.print(iConfigAdj);
					Serial.print(" = "); Serial.println(config.colors[iConfig][iConfigAdj]);
				}
			}
			// adjust wipe mode
			else if (iConfigAdj == 4) {
				if ( encChange > 0) { // increased
					if ( config.wipe[iConfig] == SOLID ) config.wipe[iConfig] = GROW;
					else if ( config.wipe[iConfig] == GROW ) config.wipe[iConfig] = WIPE;
					else if ( config.wipe[iConfig] == WIPE ) config.wipe[iConfig] = SOLID;
				}
				else { // decreased
					if ( config.wipe[iConfig] == SOLID ) config.wipe[iConfig] = WIPE;
					else if ( config.wipe[iConfig] == WIPE ) config.wipe[iConfig] = GROW;
					else if ( config.wipe[iConfig] == GROW ) config.wipe[iConfig] = SOLID;
				}
				if (SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { 
					Serial.print("Wipe for iConfig:"); Serial.print(iConfig);
					Serial.print(" = "); Serial.println(config.wipe[iConfig]);
				}
			}
			// adjust brake while wipe
			else if (iConfigAdj == 5) {
				config.brake[iConfig] = config.brake[iConfig] ? NONE : BRAKE;
				if (SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { 
					Serial.print("Brake for iConfig:"); Serial.print(iConfig);
					Serial.print(" = "); Serial.println(config.brake[iConfig]);
				}
			}
		}
		// adjust brightness // also set to run mode if off
		else {
			// encChange /= encCHUNK; // optionally use chunks to ensure fine control (move by a single index)
			configBright = constrain( configBright + encChange , 0 , 255);
			strip.setBrightness( configBright );
			if (SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.print("brightness="); Serial.println(configBright); }
			mode = BRIGHT;
		}
	}
}

void inputButton(){
	static uint8_t flagBtnEnable;
	
	if (button.update ()) {
		milTimeout = millis(); // check for button state and update timeout if action
	}
	// action when button held down and not in set mode
	if ( !button.read() && button.duration() > milBUTTON_HOLD && flagBtnEnable){
		flagBtnEnable = false; // disable btn so release has no action
		if (mode == RUN || mode == BRIGHT) {
			if(SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.println("btn held; mode=set"); }
			mode = SET;
		}
		// action when held and already in set mode
		else if ( mode == SET ) {
			if(SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.println("btn held; mode=adj"); }
			iConfigAdj = 0;
			mode = ADJ;
		}
		else if ( mode == ADJ ) {
			if(SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.println("btn held; exit Adj"); }
			mode = RUN; 
			configWrite();
		}	// count when button pressed so releasing after long hold doesn't trigger tap action
	}
	// button depressed
	else if ( button.fallingEdge() ) { 
		flagBtnEnable = true; // enable btn 
	}
	// action when button was tapped (held for less than milBUTTON_HOLD)
	else if ( button.risingEdge() && flagBtnEnable) {
		// if in setting mode, store the new settings index and exit setting mode
		if (mode == SET) {
			if(SERIALDEBUG_MASTER) { Serial.println("EEPROM write: iConfig"); }
			EEPROM.write( OFFSET_EEPROM + 0 , iConfig); // store color index
			mode = RUN;
		}
		else if (mode == ADJ){
			if (parmAdj == COLOR) {
				if (iConfigAdj < 4) parmAdj = RPM;
				else {
					iConfigAdj++;
					if (iConfigAdj > 5) iConfigAdj = 0;
				}
			}
			else if (parmAdj == RPM){
				iConfigAdj++;
				parmAdj = COLOR;
			}
		}
		// if in brightness mode, store the new brightness
		else if (mode == BRIGHT){
			if(SERIALDEBUG_MASTER) { Serial.println("EEPROM write: brightness"); }
			EEPROM.write( OFFSET_EEPROM + 1 , configBright); // store brightness
			mode = RUN;
		}
		// if in normal operation, toggle on/off state of display and save setting
		else if (mode == RUN) {
			mode = STOP;
		}
		else if (mode == STOP) {
			mode = BRIEF;
		}
	}
}

void checkTimeout(){
	// exit from pending changes if no action for milTIMEOUT_DELAY
	if ( (mode == SET || mode == BRIGHT) && ( millis() - milTimeout > milTIMEOUT_DELAY ) ){
		if(SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.println("timeout, SET/BRIGHT to RUN"); }
		colorBar(0,numLEDs);
		mode = RUN;
	}
	else if ( mode == ADJ && ( millis() - milTimeout > 4*milTIMEOUT_DELAY ) ){
		if(SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.println("timeout, ADJ to RUN"); }
		colorBar(0,numLEDs);
		mode = RUN;
	}
	else if ( mode == BRIEF && ( millis() - milTimeout > milTIMEOUT_DELAY/4 ) ){
		if(SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.println("timeout, BRIEF to RUN"); }
		colorBar(0,numLEDs);
		mode = RUN;
	}
}

// function for growing bar
inline void barGrow(uint8_t i){
	colorBar(	Wheel(config.colors[iConfig][i]), 
					((rpmAvg - config.RPMs[iConfig][i]) * numLEDs) / (config.RPMs[iConfig][i+1] - config.RPMs[iConfig][i]) + 1
				); 
}

// function for wipe from config.RPMs[iConfig][i] of config.colors[iConfig][i] with background of index [i-1]
inline void barWipe(uint8_t i){ 
	uint8_t numBar = ((rpmAvg - config.RPMs[iConfig][i]) * numLEDs) / (config.RPMs[iConfig][i+1] - config.RPMs[iConfig][i]) + 1;
	colorBarPart(Wheel(config.colors[iConfig][i]), 0, numBar); 
	colorBarPart(Wheel(config.colors[iConfig][i-1]), numBar, numLEDs - numBar);
}

// display a bar of the four color segments, for feedback when changing config/brightness
void showColors() {
	colorBarPart(Wheel(config.colors[iConfig][0] ), 0, 1);
	colorBarPart(Wheel(config.colors[iConfig][1] ), 1, numLEDs - 5);
	colorBarPart(Wheel(config.colors[iConfig][2] ), numLEDs - 4, 2);
	colorBarPart(Wheel(config.colors[iConfig][3] ), numLEDs - 2, 2);
}

void showConfigColor(){
	// first paint the colors
	colorBarPart(Wheel(config.colors[iConfig][0] ), 0, 1);
	colorBarPart(Wheel(config.colors[iConfig][1] ), 1, numLEDs - 5);
	colorBarPart(Wheel(config.colors[iConfig][2] ), numLEDs - 4, 2);
	colorBarPart(Wheel(config.colors[iConfig][3] ), numLEDs - 2, 2);
	// flash color for iConfigAdj
	switch (iConfigAdj){
		case 0:
			colorBarPart(blinkStateAdj ? Wheel(config.colors[iConfig][0]) : 0, 0, 1);			
			break;
		case 1:
			colorBarPart(blinkStateAdj ? Wheel(config.colors[iConfig][1]) : 0, 1, numLEDs - 5);
			break;
		case 2:
			colorBarPart(blinkStateAdj ? Wheel(config.colors[iConfig][2]) : 0, numLEDs - 4, 2);
			break;
		case 3:
			colorBarPart(blinkStateAdj ? Wheel(config.colors[iConfig][3]) : 0, numLEDs - 2, 2);
			break;
		case 4:
			// adjusting wipe mode
			if (config.wipe[iConfig] == SOLID){
				// show that wipe appears as a solid block (blink wipe)
				colorBarPart(blinkStateAdj ? Wheel(config.colors[iConfig][2]) : 0, numLEDs - 4, 2);
			}
			else if (config.wipe[iConfig] == GROW){
				// show that wipe grows from nothing (sequence wipe and blank bar)
				colorBarPart(0, 1, numLEDs - 5);
				colorBarPart(blinkStateAdj ? 0 : Wheel(config.colors[iConfig][2]), numLEDs - 4, 1);
				colorBarPart(blinkStateAdj ? Wheel(config.colors[iConfig][2]) : 0, numLEDs - 3, 1);
			}
			else if (config.wipe[iConfig] == WIPE){
				// show that wipe grows from bar (sequence wipe)
				colorBarPart(blinkStateAdj ? 0 : Wheel(config.colors[iConfig][2]), numLEDs - 4, 1);
				colorBarPart(blinkStateAdj ? Wheel(config.colors[iConfig][2]) : 0, numLEDs - 3, 1);
			}
			break;
		case 5:
			// brake while wipe
			colorBarPart(blinkStateAdj ? Wheel(config.colors[iConfig][2]) : 0, numLEDs - 4, 2);
			digitalWrite(pinBRAKELIGHT,  config.brake[iConfig] ? HIGH : LOW); 
			break;
	}

}

void showConfigRPM(){
	// convert 1000 and 100 digits to 4-bit binary
	uint8_t showRPM_1000 = config.RPMs[iConfig][iConfigAdj] / 1000;
	uint8_t showRPM_100 = (config.RPMs[iConfig][iConfigAdj] - 1000*showRPM_1000) / 100;
	for (uint8_t i=0; i<4; i++){
		strip.setPixelColor(i, (bitRead(showRPM_1000, 3-i) ? Wheel(85) : 0 ) );
	}
	for (uint8_t i=4; i<8; i++){
		strip.setPixelColor(i, (bitRead(showRPM_100, 7-i) ? Wheel(170) : 0 ) );
	}
	if (numLEDs > 8) colorBarPart(0, 8, numLEDs - 8);
}


// allows control by numpad for testing
// press 0 to reset RPM to zero
// press 1/3 to decrease/increase by 10 RPM
// press 4/6 to decrease/increase by 100 RPM
// press 1/9 to decrease/increase by 1000 RPM
void inputSerial(){ 
	uint8_t incomingByte;
	if(SERIALDEBUG_MASTER && SERIALDEBUG_SLOW){
		while(Serial.available()){
			incomingByte = Serial.read();
			// Serial.print("in: "); Serial.println(incomingByte);
			switch (incomingByte) {
				case 48: // 0
					rpmAvg = 0; break;
				case 49: // 1
					rpmAvg -= 10; break;
				case 50: // 2
					break;
				case 51: //3
					rpmAvg += 10; break;
				case 52: //4
					rpmAvg -= 100; break;
				case 53: //5
					break;
				case 54: //6
					rpmAvg += 100; break;
				case 55: //7
					rpmAvg -= 1000; break;
				case 56: //8
					break;
				case 57: //9
					rpmAvg += 1000; break;
			}
		Serial.print("rpmAvg: "); Serial.println(rpmAvg);
		}
	}
}


// Adafruit NeoPixel functions

// input color and number of pixels
void colorBar(uint32_t c, uint8_t num){
	for (uint16_t i=0; i<strip.numPixels();i++){
		if (i < num){
			strip.setPixelColor(i,c);
		}
		else {
			strip.setPixelColor(i,0);
		}
	}
}

// input color, start position, and number of pixels
void colorBarPart(uint32_t c, uint8_t start, uint8_t num){
	for (uint16_t i=(start); i<strip.numPixels();i++){
		if (i < start + num){
			strip.setPixelColor(i,c);
		}
	}
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r. (actually it looks like g-r-b)
uint32_t Wheel(uint8_t WheelPos) {
	if(WheelPos < 85) {
		return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
	} 
	else if(WheelPos < 170) {
		WheelPos -= 85;
		return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
	} 
	else {
		WheelPos -= 170;
		return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
}

// cycle a full bar through the rainbow with a delay betwen changes
void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}
