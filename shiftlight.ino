// Code inspired by a post by ivan141 on the arduino forum 
// http://forum.arduino.cc/index.php/topic,8420.0.html
// Modified by Jason Melvin to 
// * use an Adafruit NeoPixel strip
// * use integer math
// * only work with the latest accumulated interrupt cycle
// * use micros for better resolution at high rpm
// * use an encoder and variable settings for brightness, color scheme, rpm setpoints
// * save states to EEPROM

#include <Adafruit_NeoPixel.h>
#include <Encoder.h>
#include <Bounce.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>

// serial functionality
#define SERIALDEBUG_MASTER 0 // master switch (requires one below too)
#define SERIALDEBUG_SLOW 1 // button + encoder feedback
#define SERIALDEBUG_LIVE 0 // rpm calculation feedback

#define numCONFIGS 5
							
uint16_t	configRPM[numCONFIGS][4]	= {
// dot // start growing bar // start wipe // flashing 
	{ 5800, 7000, 7800, 8000}, 
	{ 5800, 7000, 7800, 8000}, 
	{ 5800, 7000, 7800, 8000}, 
	{ 5000, 5800, 7800, 8000}, 
	{ 1000, 2000, 5800, 7800} 
}; 

uint8_t configColors[numCONFIGS][6] = { // preset color schemes
 // start, bar , wipe, redline //*// wipe mode // brake while wipe 
	{  0 ,  85 , 170 , 170, 2 , 1},
	{  0 ,  85 ,   0 , 170  , 2 , 1},
	{  0 , 170 ,  85 ,  85 , 2 , 1},
	{  0 ,  85 , 170 , 170, 2 , 1},
	{  0 ,   0 , 170 ,  85  , 2 , 0}
	// wipe mode: 0 = solid; 1 = bar; 2 = wipe
	// green = 0; red = 85; blue = 170
};

/*  // place holder for a possible matrix to scale brightness of individual segments
float configBrightnessScalers[numCONFIGS][4]	= {
	{ 0.7, 1.0, 1.0, 1.0}, 
	{ 0.7, 1.0, 1.0, 1.0}, 
	{ 0.7, 1.0, 1.0, 1.0}, 
	{ 0.6, 0.6, 1.0, 1.0}, 
	{ 0.3, 0.3, 0.6, 1.0} 
};
*/
#define milBlinkIntervalLEDs 50 // ms interval for blinking LEDs at redline
#define milBlinkIntervalBrake 250 // ms interval for blinking brakes at redline
#define numAVG 4 // number of tach interrupts to accumulate
#define milEXIT_DELAY 10000 // exit setting mode if not adjusted for this long (ms)
#define milSHOW_DELAY 5000 // show color scheme for this long (ms)
#define milBUTTON_HOLD 1000 // how long you must hold the button to get into setting mode
#define OFFSET_EEPROM 0 // memory start address
#define encCHUNK 4 // decrease the sensitivity of the encoder

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
// uint32_t milLastProcessed; //time in millis of most recent tach interrupt

// Blink
uint8_t blinkStateLEDs = HIGH;
uint32_t milBlinkPrevLEDs = 0;
uint8_t blinkStateBrake = HIGH;
uint32_t milBlinkPrevBrake = 0;

// Encoder
Encoder myEnc(pinENC_A, pinENC_B);
int32_t encOld  = 0;
Bounce button = Bounce( pinBUTTON, 5);

// time tracking for config exit delay and display
uint32_t milAdjustedBright;
uint32_t milAdjustedConfig;
uint32_t milShowedColors;

// operating state variables
bool flagShowColors = false;
bool pendingBright = false;
bool modeRun;
bool modeSet = false;
uint8_t  brightness = 32; // default overall brightness
uint8_t  iConfig = 0; // which configuration is active

// Init function
void setup()
{
	pinMode(pinBRAKELIGHT, OUTPUT);
	digitalWrite(pinBRAKELIGHT, LOW);
	pinMode(pinBUTTON, INPUT);
	digitalWrite(pinBUTTON, HIGH); // enable pullup resistor
	
	if(SERIALDEBUG_MASTER) Serial.begin(115200);

	modeRun = EEPROM.read(OFFSET_EEPROM); // recall mode
	brightness = EEPROM.read(OFFSET_EEPROM + 1); // recall brightness
	iConfig = EEPROM.read(OFFSET_EEPROM + 2); // recall  configuration
	
	if (SERIALDEBUG_MASTER) {
		Serial.println("Recalled parameters:");
		Serial.print("modeRun:    ");	Serial.println(modeRun);
		Serial.print("brightness: "); Serial.println(brightness);
		Serial.print("iConfig:    "); Serial.println(iConfig);
	}
	
	// set up the LED strip
	strip.begin();
	strip.setBrightness(8); 
	rainbow(2); // a colorful intro on powerup
	strip.setBrightness(brightness); 
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
	if (!modeSet) updateDisplay();  // update display based on processed tachometer reading
	delay(10);  //wait a while // why?
}


void updateDisplay() {
	if (flagShowColors){
		if ( millis () - milShowedColors > milSHOW_DELAY ) { // show the lights for a while when settings adjusted
		colorBar(0,numLEDs);
		flagShowColors = false;
		}
	}
	else {
		//** RPM > redline
		if (rpmAvg >= configRPM[iConfig][3]){ 
			// blinking LEDs at redline
			colorBar(blinkStateLEDs ? Wheel(configColors[iConfig][3]) : 0, numLEDs); 
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
			if ( rpmAvg >= configRPM[iConfig][2] ){ 
				if( configColors[iConfig][4] == 2) barWipe(2); // wipe from previous color to [2]
				else if (configColors[iConfig][4] == 1) barGrow(2); // growing bar of color [2]
				else colorBar(Wheel(configColors[iConfig][2]), numLEDs);  // solid bar of color index [2]
				
				if( configColors[iConfig][5] ) digitalWrite(pinBRAKELIGHT, HIGH ); // optionally flash the brakelight
				else digitalWrite(pinBRAKELIGHT, LOW);
			}
			else {
				digitalWrite(pinBRAKELIGHT, LOW);
				// pwr < RPM < shift
				if ( rpmAvg >= configRPM[iConfig][1] ){ 
					barGrow(1); // growing bar from pwr > shift
				}
				// start < RPM < pwr
				else if ( rpmAvg >= configRPM[iConfig][0] ) { 
					colorBar(Wheel(configColors[iConfig][0]),1); // one light above first threshold
				}
				// RPM < start
				else { 
					colorBar(0,numLEDs);
				}
			}
		}
	}
	// if off, then show nothing
	if (!modeRun) colorBar(0, numLEDs);
	strip.show(); 
}

// function for growing bar
inline void barGrow(uint8_t i){
	colorBar(	Wheel(configColors[iConfig][i]), 
					((rpmAvg - configRPM[iConfig][i]) * numLEDs) / (configRPM[iConfig][i+1] - configRPM[iConfig][i]) + 1
				); 
}

// function for wipe from configRPM[iConfig][i] of configColors[iConfig][i] with background of index [i-1]
inline void barWipe(uint8_t i){ 
	uint8_t numBar = ((rpmAvg - configRPM[iConfig][i]) * numLEDs) / (configRPM[iConfig][i+1] - configRPM[iConfig][i]) + 1;
	colorBarPart(Wheel(configColors[iConfig][i]), 0, numBar); 
	colorBarPart(Wheel(configColors[iConfig][i-1]), numBar, numLEDs - numBar);
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
		// milLastProcessed = millis();
	}
	/*
	if ( (millis() > (milLastProcessed + 3000) ) && ( rpmInst > 0 ) ){ // detect stopped engine
		rpmInst=0; 
		rpmAvg=0;    
	}
	*/
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
		if (SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { 
			Serial.print("encChange="); Serial.println(encChange);
		}
		// if in setting mode, change the index of the configuration matrix
		if (modeSet) {
			encChange /= encCHUNK; // use chunks to ensure fine control (move by a single index)
			if ( encChange > 0) { // increased
				iConfig = constrain( iConfig+1, 0, numCONFIGS - 1 );
			}
			else { // decreased
				iConfig = constrain( iConfig-1, 0, numCONFIGS - 1 );
			}
			if (SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.print("iConfig="); Serial.println(iConfig); }
			showColors(); // show the full color bar when adjusted, for feedback
			milAdjustedConfig = millis();
		}
		// if not in setting mode, change the brightness
		else {
			// encChange /= encCHUNK; // optionally use chunks to ensure fine control (move by a single index)
			brightness = constrain( brightness + encChange , 0 , 255);
			strip.setBrightness( brightness );
			if (SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.print("brightness="); Serial.println(brightness); }
			showColors(); // show the full color bar when adjusted, for feedback
			milAdjustedBright = millis();
			pendingBright = true;
		}
	}
	// exit from pending changes if no encoder adjustment for milEXIT_DELAY
	else if ( pendingBright && ( millis() - milAdjustedBright > milEXIT_DELAY ) ){
		pendingBright = false;
	}
	else if ( modeSet && ( millis() - milAdjustedConfig > milEXIT_DELAY ) ){
		modeSet = false;
	}	
}

void inputButton(){
	static uint8_t iBtn;
	button.update (); // check for button state
	// action when button has been held down for milBUTTON_HOLD
	if ( !button.read() && button.duration() > milBUTTON_HOLD && !modeSet) { 
		if(SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.println("btn held; mode=set"); }
		iBtn = 0; // reset the button count
		milAdjustedConfig = millis();
		modeSet = true;
		showColors();
	}
	// count when button pressed so releasing after long hold doesn't trigger tap action
	else if ( button.fallingEdge() ) { 
		iBtn++; // increment the count
	}
	// action when button was tapped (held for less than milBUTTON_HOLD)
	else if ( button.risingEdge() && iBtn ) { 
		iBtn = 0; // reset the count
		// if in setting mode, store the new settings index and exit setting mode
		if (modeSet) {
			if(SERIALDEBUG_MASTER) { Serial.println("EEPROM write: iConfig"); }
			EEPROM.write( OFFSET_EEPROM + 2 , iConfig); // store color index
			modeSet = false;
		}
		// if in brightness mode, store the new brightness
		else if (pendingBright){
			if(SERIALDEBUG_MASTER) { Serial.println("EEPROM write: brightness"); }
			EEPROM.write( OFFSET_EEPROM + 1 , brightness); // store brightness
			pendingBright = false;
		}
		// if in normal operation, toggle on/off state of display and save setting
		else {
			modeRun = modeRun ? false : true;
			if (modeRun) { showColors(); } //rainbow(5);
			EEPROM.write( OFFSET_EEPROM , modeRun ); // store mode
			if(SERIALDEBUG_MASTER && SERIALDEBUG_SLOW) { Serial.print("modeRun: "); Serial.println(modeRun); }
		}
	}
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

// display a bar of the four color segments, for feedback when changing config/brightness
void showColors() {
	flagShowColors = true;
	colorBarPart(Wheel(configColors[iConfig][0] ), 0, 1);
	colorBarPart(Wheel(configColors[iConfig][1] ), 1, numLEDs - 5);
	colorBarPart(Wheel(configColors[iConfig][2] ), numLEDs - 4, 2);
	colorBarPart(Wheel(configColors[iConfig][3] ), numLEDs - 2, 2);
	strip.show();
	milShowedColors = millis();
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
		if (i < start + 1 + num){
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
