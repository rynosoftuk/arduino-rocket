#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>

#define LIGHTS_PIN 4
#define MP3_RX_PIN 2
#define MP3_TX_PIN 3

#define LAUNCH_PIN  9
#define FLIGHT_PIN  8
#define LANDING_PIN  7

int Initialised = 0;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, LIGHTS_PIN, NEO_GRB + NEO_KHZ800);

// color variables: mix RGB (0-255) for desired yellow
int redPx = 255;
int grnHigh = 120; //110-120 for 5v, 135 for 3.3v
int bluePx = 15; //10 for 5v, 15 for 3.3v

// animation time variables, with recommendations
int burnDepth = 14; //10 for 5v, 14 for 3.3v -- how much green dips below grnHigh for normal burn - 
int flutterDepth = 30; //25 for 5v, 30 for 3.3v -- maximum dip for flutter
int cycleTime = 120; //120 -- duration of one dip in milliseconds

// pay no attention to that man behind the curtain
int fDelay;
int fRep;
int flickerDepth;
int burnDelay;
int burnLow;
int flickDelay;
int flickLow;
int flutDelay;
int flutLow;

void setup() 
{
  if(!Initialised) {
    Serial.begin(115200);
  
    pinMode(LAUNCH_PIN,INPUT_PULLUP);
    pinMode(FLIGHT_PIN,INPUT_PULLUP);
    pinMode(LANDING_PIN,INPUT_PULLUP);
  
    strip.begin();
    strip.setBrightness(100); //adjust brightness here
    strip.show(); // Initialize all pixels to 'off'
  
    MP3_Init();
    Initialised = 1;
  }
}

void loop() 
{
  if(!digitalRead(LAUNCH_PIN)) {
    while(!digitalRead(LAUNCH_PIN))
      delay(10);
    launch();
  } else if(!digitalRead(FLIGHT_PIN)) {
    while(!digitalRead(FLIGHT_PIN))
      delay(10);
    flight();
  } else if(!digitalRead(LANDING_PIN)) {
    while(!digitalRead(LANDING_PIN))
      delay(10);
    landing();
  }
  delay(10);
  
  // Some example procedures showing how to display to the pixels:
  //colorWipe(strip.Color(255, 0, 0), 50); // Red
  //colorWipe(strip.Color(0, 255, 0), 50); // Green
  //colorWipe(strip.Color(0, 0, 255), 50); // Blue
  //rainbow(20);
  //rainbowCycle(20);
}

void delayWithButtonCheck(int ms) {
  uint16_t start = (uint16_t)micros();

  while (ms > 0) {
    // If the mode has changed
    if(!digitalRead(LAUNCH_PIN) || !digitalRead(FLIGHT_PIN) || !digitalRead(LANDING_PIN)) {
      asm volatile ("jmp 0");
    }
    
    if (((uint16_t)micros() - start) >= 1000) {
      ms--;
      start += 1000;
    }
  }
}

void launch() {
  MP3_PlayTrack(1, 15);
  colorWipe(strip.Color(30, 30, 30), 0);
  delayWithButtonCheck(7200);

  unsigned long burnTime;
  burnTime = millis();
  
  while(millis() - burnTime < 20000) {
    fire(random(50)+min((millis()-burnTime) / 100, 50));
    delayWithButtonCheck(random(50)+25);
  }

  colorWipe(strip.Color(0, 0, 0), 0);  
}

void flight() {
  MP3_PlayTrack(2, 20);
  rainbowCycle(19);
  colorWipe(strip.Color(0, 0, 0), 0);  
}

void landing() {
  MP3_PlayTrack(3, 8);

  unsigned long burnTime;
  burnTime = millis();
  
  while(millis() - burnTime < 10000) {
    fire(random(50)+50);
    delayWithButtonCheck(random(50)+25);
  }

  colorWipe(strip.Color(30, 30, 30), 0);
  delayWithButtonCheck(13000);

  colorWipe(strip.Color(0, 0, 0), 0);  
}

/***************************************************************
* Rocket Functions
***************************************************************/

// basic fire funciton - not called in main loop
void fire(int grnLow) {
  for (int grnPx = grnHigh; grnPx > grnLow; grnPx--) {
    for(int i = 0; i< 4; i++)
      strip.setPixelColor(i, redPx, grnPx, bluePx);
    strip.show();
    delayWithButtonCheck(fDelay);
  }  
  for (int grnPx = grnLow; grnPx < grnHigh; grnPx++) {
    for(int i = 0; i< 4; i++)
      strip.setPixelColor(i, redPx, grnPx, bluePx);
    strip.show();
    delayWithButtonCheck(fDelay);
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delayWithButtonCheck(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delayWithButtonCheck(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*4; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delayWithButtonCheck(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

/***************************************
* MP3 Control
***************************************/

#define MP3_CMD_SET_VOLUME 0X06
#define MP3_CMD_PLAY_W_INDEX 0X08
#define MP3_CMD_SEL_DEV 0X09
#define MP3_DEV_TF 0X02
#define MP3_CMD_PLAY 0X0D
#define MP3_CMD_PAUSE 0X0E
#define MP3_CMD_SINGLE_CYCLE 0X19
#define MP3_SINGLE_CYCLE_ON 0X00
#define MP3_SINGLE_CYCLE_OFF 0X01
#define MP3_CMD_PLAY_W_VOL 0X22

static int8_t MP3_SendBuf[8] = {0} ;
SoftwareSerial mp3Serial(MP3_RX_PIN, MP3_TX_PIN);

void MP3_Init() {
  mp3Serial.begin(9600);
  delay(500);//Wait chip initialization is complete
  MP3_sendCommand(MP3_CMD_SEL_DEV, MP3_DEV_TF);//select the TF card  
  delay(200);//wait for 200ms
}

void MP3_PlayTrack(int track, int volume) {
  MP3_sendCommand(MP3_CMD_PLAY_W_VOL, ((volume & 0xFF) << 8) | (track & 0xFF));//play the first song with volume 15 class
}

void MP3_sendCommand(int8_t command, int16_t dat)
{
  delayWithButtonCheck(20);
  MP3_SendBuf[0] = 0x7e; //starting byte
  MP3_SendBuf[1] = 0xff; //version
  MP3_SendBuf[2] = 0x06; //the number of bytes of the command without starting byte and ending byte
  MP3_SendBuf[3] = command; //
  MP3_SendBuf[4] = 0x00;//0x00 = no feedback, 0x01 = feedback
  MP3_SendBuf[5] = (int8_t)(dat >> 8);//datah
  MP3_SendBuf[6] = (int8_t)(dat); //datal
  MP3_SendBuf[7] = 0xef; //ending byte
  for(uint8_t i=0; i<8; i++)//
  {
    mp3Serial.write(MP3_SendBuf[i]) ;
  }
}
