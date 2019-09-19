#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#define COLORCORRECTION 3.1372549

// Scottie 1 mode (time in ms)
#define COLORSCANTIME 138.240
#define SEPARATORPULSETIME 1.5
#define SEPARATORPULSEFREQ 1500
#define SYNCPULSETIME 9
#define SYNCPULSEFREQ 1200

#define BUZZPIN 9

// Sd consts
#define SDSLAVE 53
#define SDCLOCK 13
#define SDMOSI 11
#define SDMISO 12

String path = "e.txt";

File myFile;      // File
byte buff[3];     // File read buffer


byte buffR[320]; // Buffer conintating Red values of the line
byte buffG[320]; // Buffer conintating Green values of the line
byte buffB[320]; // Buffer conintating Blue values of the line

uint16_t playPixel(long pixel);
uint16_t freq(uint8_t c);
void calibrationHeader();
void startingSyncPulse();
void transmit(int freq, float duration);
void openFile();
void playPixel(uint8_t r, uint8_t g, uint8_t b);

////


/*
Tone generator
v1  use timer, and toggle any digital pin in ISR
   funky duration from arduino version
   TODO use FindMckDivisor?
   timer selected will preclude using associated pins for PWM etc.
    could also do timer/pwm hardware toggle where caller controls duration
*/


// timers TC0 TC1 TC2   channels 0-2 ids 0-2  3-5  6-8     AB 0 1
// use TC1 channel 0
#define TONE_TIMER TC1
#define TONE_CHNL 0
#define TONE_IRQ TC3_IRQn

// TIMER_CLOCK4   84MHz/128 with 16 bit counter give 10 Hz to 656KHz
//  piano 27Hz to 4KHz

static uint8_t pinEnabled[PINS_COUNT];
static uint8_t TCChanEnabled = 0;
static boolean pin_state = false ;
static Tc *chTC = TONE_TIMER;
static uint32_t chNo = TONE_CHNL;

volatile static int32_t toggle_count;
static uint32_t tone_pin;

// frequency (in hertz) and duration (in milliseconds).

void tone(uint32_t ulPin, uint32_t frequency, int32_t duration)
{
		const uint32_t rc = VARIANT_MCK / 256 / frequency;
		tone_pin = ulPin;
		toggle_count = 0;  // strange  wipe out previous duration
		if (duration > 0 ) toggle_count = 2 * frequency * duration / 1000;
		 else toggle_count = -1;

		if (!TCChanEnabled) {
			pmc_set_writeprotect(false);
			pmc_enable_periph_clk((uint32_t)TONE_IRQ);
			TC_Configure(chTC, chNo,
				TC_CMR_TCCLKS_TIMER_CLOCK4 |
				TC_CMR_WAVE |         // Waveform mode
				TC_CMR_WAVSEL_UP_RC ); // Counter running up and reset when equals to RC

			chTC->TC_CHANNEL[chNo].TC_IER=TC_IER_CPCS;  // RC compare interrupt
			chTC->TC_CHANNEL[chNo].TC_IDR=~TC_IER_CPCS;
			 NVIC_EnableIRQ(TONE_IRQ);
                         TCChanEnabled = 1;
		}
		if (!pinEnabled[ulPin]) {
			pinMode(ulPin, OUTPUT);
			pinEnabled[ulPin] = 1;
		}
		TC_Stop(chTC, chNo);
                TC_SetRC(chTC, chNo, rc);    // set frequency
		TC_Start(chTC, chNo);
}

void noTone(uint32_t ulPin)
{
	TC_Stop(chTC, chNo);  // stop timer
	digitalWrite(ulPin,LOW);  // no signal on pin
}

// timer ISR  TC1 ch 0
void TC3_Handler ( void ) {
	TC_GetStatus(TC1, 0);
	if (toggle_count != 0){
		// toggle pin  TODO  better
		digitalWrite(tone_pin,pin_state= !pin_state);
		if (toggle_count > 0) toggle_count--;
	} else {
		noTone(tone_pin);
	}
}



////






void setup() {
  delay(5000);
  Serial.begin(9600);
  //transmit(1900,300);


  // Sd initialize
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDSLAVE)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");




  // re-open the file for reading:
  myFile = SD.open(path);
  if (myFile) {
    Serial.println("test.txt:");

    /** VOX TONE (OPTIONAL) **/
    transmit(1900, 100000);
    transmit(1500, 100000);
    transmit(1900, 100000);
    transmit(1500, 100000);
    transmit(2300, 100000);
    transmit(1500, 100000);
    transmit(2300, 100000);
    transmit(1500, 100000);

    /** CALIBRATION HEADER **/

    transmit(1900, 300000);
    transmit(1200, 10000);
    transmit(1900, 300000);
    transmit(1200, 30000);
    transmit(1300, 30000);    // 0
    transmit(1300, 30000);    // 0
    transmit(1100, 30000);    // 1
    transmit(1100, 30000);    // 1
    transmit(1100, 30000);    // 1
    transmit(1100, 30000);    // 1
    transmit(1300, 30000);    // 0
    transmit(1300, 30000);    // Even parity
    transmit(1200, 30000);    // VIS stop bit

    /** STARTING SYNC PULSO (FIRST LINE ONLY)  **/

    transmit(1200, 9000);
    int line = 0;
    /** TRANSMIT EACH LINE **/
    while(myFile.available()){
      Serial.println(line++);

      // Read line and store color values in the buffer
      for(uint16_t i = 0; i < 320; i++){
        buffR[i] = myFile.read();
        buffG[i] = myFile.read();
        buffB[i] = myFile.read();
      }

      // Separator pulse
      transmit(1500, 1500);

      // Green Scan
      for(uint16_t i = 0; i < 320; i++){
        transmit(1500 + 3.1372549 * buffG[i], 432);    // .4320ms/pixel
      }

      // Separator Pulse
      transmit(1500, 1500);

      // Blue Scan
      for(uint16_t i = 0; i < 320; i++){
        transmit(1500 + 3.1372549 * buffB[i], 432);    // .4320ms/pixel
      }

      // Sync Pulse
      transmit(1200, 9000);

      // Sync porch
      transmit(1500, 1500);

      // Red Scan
      for(uint16_t i = 0; i < 320; i++){
        transmit(1500 + 3.1372549 * buffR[i], 432);    // .4320ms/pixel
      }
    }

    Serial.println("Finish");

    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void loop() {

  /*
  calibrationHeader();

  startingSyncPulse();

  for(int i = 0; i < 10000; i++){
    playPixel(qbert[i]);
  }

  delay(10000);

  */
}


void transmit(int freq, float duration){
  tone(BUZZPIN, freq, duration);
  delayMicroseconds(duration);
}

void encoder(){

}

void startingSyncPulse(){
  transmit(SYNCPULSEFREQ, SYNCPULSETIME);
}

void playPixel(uint8_t r, uint8_t g, uint8_t b){
  // Separator pulse
  transmit(SEPARATORPULSEFREQ, SEPARATORPULSETIME);

  // Green Scan
  transmit(freq(g), COLORSCANTIME);

  // Separator Pulse
  transmit(SEPARATORPULSEFREQ, SEPARATORPULSETIME);

  // Blue Scan
  transmit(freq(b), COLORSCANTIME);

  // Sync Pulse
  transmit(SYNCPULSEFREQ, SYNCPULSETIME);

  // Sync porch
  transmit(SEPARATORPULSEFREQ, SEPARATORPULSETIME);

  // Red Scan
  transmit(freq(r), COLORSCANTIME);
}

uint16_t playPixel(long pixel){
  uint8_t r = pixel & 0xFF0000 >> 16;
  uint8_t g = pixel & 0x00FF00 >> 8;
  uint8_t b = pixel & 0x0000FF;

  // Separator pulse
  transmit(SEPARATORPULSEFREQ, SEPARATORPULSETIME);

  // Green Scan
  transmit(freq(g), COLORSCANTIME);

  // Separator Pulse
  transmit(SEPARATORPULSEFREQ, SEPARATORPULSETIME);

  // Blue Scan
  transmit(freq(b), COLORSCANTIME);

  // Sync Pulse
  transmit(SYNCPULSEFREQ, SYNCPULSETIME);

  // Sync porch
  transmit(SEPARATORPULSEFREQ, SEPARATORPULSETIME);

  // Red Scan
  transmit(freq(r), COLORSCANTIME);
}

/**
 * Get output frequency given a color component (R, G, B) from 0 to 255
**/
uint16_t freq(uint8_t c){
  return 1500 + (c * COLORCORRECTION);
}

/**


*/

void calibrationHeader(){
  // Leader tone

  //transmit(1200, 9);

  transmit(1900, 300);
  transmit(1200, 10);
  transmit(1900, 300);
  transmit(1200, 30);
  transmit(1300, 30);    // 0
  transmit(1300, 30);    // 0
  transmit(1100, 30);    // 1
  transmit(1100, 30);    // 1
  transmit(1100, 30);    // 1
  transmit(1100, 30);    // 1
  transmit(1300, 30);    // 0
  transmit(1300, 30);    // Even parity
  transmit(1200, 30);    // VIS stop bit


}
