#include <mcp_can.h>
#include <mcp_can_dfs.h>
// using latest CAN library from https://github.com/Seeed-Studio/CAN_BUS_Shield as of April 5, 2019

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// LED Flashing =======================================================================
// RPM --------------------------------------------------------------------------------
long rpmFlashLast = 0;
byte rpmFlash = 1;
// Engine Temp ------------------------------------------------------------------------
long engTempFlashLast = 0;
byte engTempFlash = 1;
// GTFO -------------------------------------------------------------------------------
long gtfoFlashLast = 0;
byte gtfoFlash = 1;

// Shift Registers ====================================================================
#define latchPIN    4 // pin 3 on ATMega328P
#define clockPIN    7 // pin 11 on ATMega328P
#define dataPIN     3 // pin 1 on ATMega328P
#define pinQ0       1
#define pinQ1       2
#define pinQ2       4
#define pinQ3       8
#define pinQ4      16
#define pinQ5      32
#define pinQ6      64
#define pinQ7     128
int SR1 = 0, SR2 = 0, SR3 = 255, SR4 = 255, SR5 = 255, SR6 = 255;

// Shift Lights =======================================================================
#define sRPM1    6800
#define sRPM2    7400
#define sRPM3    8000
#define sRPM4    8600
#define sRPM5    9200
#define sRPM6    9800
#define sRPM7   10400
#define sRPM8   11000
#define sRPM9   11600
#define sRPM10  12000
#define sRPM11  12200
#define sRPM12  12300
#define sLED1   pinQ2 //SR2 
#define sLED2   pinQ3 //SR2 
#define sLED3   pinQ4 //SR2 
#define sLED4   pinQ5 //SR2 
#define sLED5   pinQ6 //SR2
#define sLED6   pinQ7 //SR2
#define sLED7   pinQ7 //SR1
#define sLED8   pinQ6 //SR1
#define sLED9   pinQ2 //SR1
#define sLED10  pinQ3 //SR1
#define sLED11  pinQ4 //SR1
#define sLED12  pinQ5 //SR1
#define rpmFlashTime 75 // RPM Flash timing (ms)

// Warning Lights =====================================================================
#define ledLR  pinQ4
#define ledLG  pinQ3
#define ledLB  pinQ2
#define ledRR  pinQ7
#define ledRG  pinQ6
#define ledRB  pinQ5
#define engTempLow        60    //Blue Blink
#define engTempMedLow     70    //Blue
#define engTempMedHigh    95    //Yellow
#define engTempHigh       105   //Red
#define engTempFlashTime  500   //Time in ms
#define batVoltMed        13    //Yellow
#define batVoltLow        12.5  //Red
int launchArm = 0;

// 7 Segment ==========================================================================
#define ssC pinQ1 //SR3
#define ssD pinQ2 //SR3  
#define ssE pinQ3 //SR3
#define ssG pinQ4 //SR3
#define ssF pinQ5 //SR3
#define ssA pinQ6 //SR3
#define ssB pinQ7 //SR3
int ssCount = 1;
int seg1 = (ssB+ssC); // 1
int seg2 = (ssA+ssB+ssD+ssE+ssG); // 2
int seg3 = (ssA+ssB+ssC+ssD+ssG); // 3 
int seg4 = (ssB+ssC+ssF+ssG); // 4
int seg5 = (ssA+ssC+ssD+ssF+ssG); // 5
int seg6 = (ssA+ssC+ssD+ssE+ssF+ssG); // 6

// CAN Bus ============================================================================
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); 
#define PE1  0xF048
#define PE2  0xF148
#define PE3  0xF248
#define PE4  0xF348
#define PE5  0xF448
#define PE6  0xF548
#define PE7  0xF648
#define PE8  0xF748
#define PE9  0xF848
#define PE10 0xF948
#define PE11 0xFA48
#define PE12 0xFB48
#define PE13 0xFC48
#define PE14 0xFD48
#define PE15 0xFE48
#define PE16 0xFF48
#define PDMs 0X0500
#define PDM1 0x7FF    // FAN | GLOBAL ERROR | BAT VOLT
unsigned char len = 0;
unsigned char buf[64];
unsigned int canID;

// CAN Value Variables ================================================================
// RPM --------------------------------------------------------------------------------
boolean updateRPM = false;
int rpm = 0;
int rpmLast = 0;
// Wheel Speed ------------------------------------------------------------------------
float avgWheelSpeed = 0;
// Engine Temp ------------------------------------------------------------------------
float engTemp = 0;
float engTempLast = 0;
int updateEngTemp = 0;
// Battery Voltage --------------------------------------------------------------------
float batVolt = 0;
float batVoltLast = 0;
int updateBatVolt = 0;
// Oil Pressure -----------------------------------------------------------------------
float oilPres = 0;
float oilPresLast = 0;
float oilPresLow = 23;
float oilPresHigh = 90;
long oilPresFlashLast = 0;
byte oilPresFlash = 1;
int updateOilPres = 0;
// TPS --------------------------------------------------------------------------------
int tps = 0;
int tpsLast = 0;
int updateTPS = 0;
// Fan --------------------------------------------------------------------------------
boolean fan = false;
// PDM --------------------------------------------------------------------------------
boolean pdmError = false;
boolean pdmFaulted = false;

// OLED ===============================================================================
#define OLED_MOSI  9        //Data
#define OLED_CLK   8        //Clk
#define OLED_DC    5//11    //DC/SA0
#define OLED_CS    6//13    //CS
#define OLED_RESET A1       //Rst
Adafruit_SSD1306 display(128, 64, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
int OLEDMode = 1;
int allowCycle = 0;
int switchMode = 0;
// Formula Logo -----------------------------------------------------------------------
static const unsigned char PROGMEM QFSAELogo[] = {
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000001, B11111111, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B11111111, B10000000, B00000000,
  B00000000, B00000000, B11111111, B11111000, B00000000, B00000000, B00000000, B00000011, B00000000, B00000000, B00000000, B00000000, B00011111, B11111111, B00000000, B00000000,
  B00000000, B00000000, B01100000, B00011100, B00000000, B00000000, B00000000, B00011111, B11100000, B00000000, B00000000, B00000000, B00111000, B00000110, B00000000, B00000000,
  B00000000, B00000000, B00110000, B00000111, B11111111, B11111111, B11111111, B11111100, B11111111, B11111111, B11111111, B11111111, B11100000, B00001100, B00000000, B00000000,
  B00000000, B00000000, B00011000, B00000011, B11111111, B11111111, B11111111, B11111000, B00111111, B11111111, B11111111, B11111111, B11000000, B00011000, B00000000, B00000000,
  B00000000, B00000000, B00001100, B00000000, B00000000, B00000000, B00000000, B01100000, B00011100, B00000000, B00000000, B00000000, B00000000, B00110000, B00000000, B00000000,
  B00000000, B00000000, B00000110, B00000000, B00000000, B00000000, B00000001, B11000000, B00001110, B00000000, B00000000, B00000000, B00000000, B01100000, B00000000, B00000000,
  B00000000, B00000000, B00000011, B11111111, B11111111, B10000000, B00000001, B10000000, B00000111, B00000000, B00000011, B11111111, B11111111, B11000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00001111, B11111111, B11111111, B11111111, B10000000, B00000011, B11111111, B11111111, B11111111, B11110000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B00000000, B00000011, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000110, B00000000, B00000001, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00001110, B00000000, B00000000, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011100, B00000000, B00000000, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011100, B00000000, B00000000, B01100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00011111, B11111111, B11111111, B11000000, B00000000, B00000000, B00111000, B00000000, B00000000, B01110000, B00000000, B00000000, B00000111, B11111111, B11111111, B11100000,
  B01111111, B11111111, B11111111, B11111111, B00000000, B00000000, B01110000, B00000000, B00000000, B00111000, B00000000, B00000001, B11111111, B11111111, B11111111, B11111000,
  B11111100, B00000000, B00000011, B11111111, B11000000, B00000000, B11100000, B00000000, B00000000, B00011100, B00000000, B00001111, B11111111, B00000000, B00000000, B11111100,
  B11110000, B00000000, B00000000, B00000011, B11100000, B00000000, B11000000, B00000111, B11000000, B00001110, B00000000, B00001111, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100000, B00000000, B11000000, B00011111, B11110000, B00001110, B00000000, B00011110, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100000, B00000001, B11000000, B00111100, B01111000, B00000110, B00000000, B00011100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B01100000, B00000011, B00000111, B11110000, B00011111, B10000011, B00000000, B00011000, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B01100111, B11111111, B11111111, B11100000, B00001111, B11111111, B11111111, B10011000, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11111111, B11111111, B00001111, B11000000, B00000111, B11000001, B11111111, B11111100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100000, B00000000, B11111011, B10000000, B00000011, B11111110, B00000000, B00011100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100000, B00001111, B10000111, B00000000, B00000001, B11000111, B11100000, B00011100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100000, B11111000, B00001110, B00000000, B00000000, B11100000, B00111100, B00011100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100111, B10000000, B00011100, B00000000, B00000000, B01110000, B00000111, B10011100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11101100, B00000000, B00011100, B00000000, B00000000, B01110000, B00000000, B11011110, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B01100000, B00000000, B00111000, B00000000, B00000000, B00111000, B00000000, B00011000, B00000000, B00000000, B00000000, B00111100,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00011111, B11111110, B00111111, B11111110, B01111111, B11110001, B11000000, B00001100, B11000000, B00001011, B00000000, B00000000, B00010000, B00000000, B00111000,
  B01111000, B00011111, B11111111, B01111111, B11111111, B01111111, B11111101, B11100000, B00011101, B11000000, B00011011, B00000100, B00000000, B00111000, B00000000, B01111000,
  B01111111, B00001111, B11111111, B01110000, B00000111, B01111111, B11111101, B11110000, B00111101, B11000000, B00011011, B00000110, B00000000, B00011100, B00000111, B11111000,
  B00111111, B00000000, B00000000, B01100000, B00000011, B00000000, B00011101, B11111000, B01111101, B11000000, B00011011, B00011111, B00000000, B00001110, B00000111, B11110000,
  B00000000, B00011111, B11111111, B01100000, B00000011, B01111111, B11111001, B11011111, B11101101, B11000000, B00011011, B00001110, B00000000, B00000111, B00000000, B00000000,
  B00000000, B00011111, B11111111, B01100000, B00000011, B01111111, B11111101, B11001111, B11001101, B11000000, B00011011, B00000100, B00000001, B11111111, B10000000, B00000000,
  B00000000, B00011000, B00000000, B01110000, B00000111, B01100000, B00001101, B11000111, B10001101, B11000000, B00011011, B00000000, B00000011, B11111111, B11000000, B00000000,
  B00000000, B00011000, B00000000, B01111111, B11111111, B01100000, B00001101, B11000000, B00001101, B11111111, B11111011, B11111111, B11100111, B00000000, B11100000, B00000000,
  B00000000, B00011000, B00000000, B00111111, B11111110, B01100000, B00001101, B11000000, B00001100, B11111111, B11111011, B11111111, B11101110, B00000000, B01110000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000100, B00000000, B00100000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00011111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000
};

// Inputs =============================================================================
#define cycButPIN     A0
#define pitComPIN     A1
#define launchArmPIN  A2
int pitCom = 0;


void setup() {
  Serial.begin(115200);
  Serial.println("Upper Dash V1.0");
  Serial.println("Initializing Shift Registers..");
  // Initialize Shift Registers -------------------------------------------------------
  pinMode(latchPIN,OUTPUT);
  pinMode(clockPIN,OUTPUT);
  pinMode(dataPIN,OUTPUT);
  digitalWrite(latchPIN,LOW);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR6);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR5);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR4);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR3);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR2);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR1);
  digitalWrite(latchPIN,HIGH);
  Serial.println("Shift registers initialized!");

  // Initialize OLED ------------------------------------------------------------------
  display.begin(SSD1306_SWITCHCAPVCC);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.display();
 // initSequence();
  canOK();
  Serial.println("Good to go");
  
  //debug variable settings COMMENT THIS BEFORE PROD
//  rpm = 6500;
//  engTemp = 75;
//  fan = true;
//  batVolt = 14;
//  pdmError = false;
//  pdmFaulted = false;
//  oilPres = 50;
//  launchArm = false;
}

void canOK(){
    while (CAN_OK != CAN.begin(CAN_250KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(10000);
    }
    Serial.println("CAN BUS Shield init ok!");
    initSequence();
    return;
}

void loop() {
  Serial.println("In loop");   
// reset shift registers to base values (all lights off)
  SR1 = 0;
  SR2 = 0;
  SR3 = 255;
  SR4 = 255;
  SR5 = 255;
  SR6 = 255;
// read data from CAN
  readCAN();
// run shift light method to update SR1, SR2
  shiftLights();
// run status light method to update SR4, SR5, SR6
  statusLights();
// run 7 segment method to update SR3
  gear();
  
//deal with the OLED later

//push new values to shift registers    
  digitalWrite(latchPIN,LOW);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR6);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR5);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR4);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR3);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR2);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR1);
  digitalWrite(latchPIN,HIGH);
}

void initSequence() {
  // fancy startup sequence
  // display logo on OLED
  dispLogo();
  // light each shift light one at a time up and down
  for(int i = 1; i <= 24; i++) {
    switch(i) {
      case 1: SR2 += sLED1; break;
      case 2: SR2 += sLED2; break;
      case 3: SR2 += sLED3; break;
      case 4: SR2 += sLED4; break;
      case 5: SR2 += sLED5; break;
      case 6: SR2 += sLED6; break;
      case 7: SR1 += sLED7; break;
      case 8: SR1 += sLED8; break;
      case 9: SR1 += sLED9; break;
      case 10: SR1 += sLED10; break;
      case 11: SR1 += sLED11; break;
      case 12: SR1 += sLED12; break;
      case 13: SR1 -= sLED12; break;
      case 14: SR1 -= sLED11; break;
      case 15: SR1 -= sLED10; break;
      case 16: SR1 -= sLED9; break;
      case 17: SR1 -= sLED8; break;
      case 18: SR1 -= sLED7; break;
      case 19: SR2 -= sLED6; break;
      case 20: SR2 -= sLED5; break;
      case 21: SR2 -= sLED4; break;
      case 22: SR2 -= sLED3; break;
      case 23: SR2 -= sLED2; break;
      case 24: SR2 -= sLED1; break;
    }
      //randomize colors on the Status LEDS
      int colours[] = {111, 183, 219}; // red green blue 
      randomSeed(analogRead(0));
      SR4 = colours[random(0,3)];
      SR5 = colours[random(0,3)];
      SR6 = colours[random(0,3)];
      
     digitalWrite(latchPIN, LOW);
      shiftOut(dataPIN,clockPIN,MSBFIRST,SR6);
      shiftOut(dataPIN,clockPIN,MSBFIRST,SR5);
      shiftOut(dataPIN,clockPIN,MSBFIRST,SR4);
      shiftOut(dataPIN,clockPIN,MSBFIRST,SR3);
      shiftOut(dataPIN,clockPIN,MSBFIRST,SR2);
      shiftOut(dataPIN,clockPIN,MSBFIRST,SR1);
    digitalWrite(latchPIN, HIGH);

    delay(25);
    Serial.println("Done startup Sequence");
  }
}

//CAN code from Q17, untested on Q19
void readCAN() {
  if(CAN_MSGAVAIL == CAN.checkReceive()) {           // check if data coming
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    canID = CAN.getCanId();
    if (canID == PE1){ // PE1 ====================================================
      // RPM ---------------------------------------------------------------------
      rpm = buf[1]*256 + buf[0];
      if (rpm != rpmLast)
        updateRPM = true;
      rpmLast = rpm;
      // TPS ---------------------------------------------------------------------
      tps = buf[3]*256 + buf[2];
      if (tps > 32767)
        tps -= 65536;
      tps *= 0.1;
      if (tps != tpsLast)
        updateTPS = 1;
      tpsLast = tps;
    } else if (canID == PE3) { // PE3 ============================================
      // Oil Pressure ------------------------------------------------------------
      oilPres = buf[7]*256 + buf[6];
      if (oilPres > 32767)
        oilPres -= 65536;
      oilPres *= 0.001;
      if (oilPres != oilPresLast)
        updateOilPres= 1;
      oilPresLast = oilPres;
    } else if (canID == PE6) { // PE6 ===========================================
      // Engine Temp -------------------------------------------------------------
      engTemp = buf[5]*256 + buf[4];
      if (engTemp > 32767)
        engTemp -= 65536;
      engTemp *= 0.1;
      if (engTemp != engTempLast)
        updateEngTemp = 1;
      engTempLast = engTemp;
    } else if (canID == PE5) { // PE5 ==========================================
      // Average Wheel Speed -----------------------------------------------------
      avgWheelSpeed = buf[7]*256 + buf[6];
      if (avgWheelSpeed > 32767)
        avgWheelSpeed -= 65536;
      avgWheelSpeed *= 0.2;
    } else if (canID == PDM1) { // PDM1 ==========================================
      // Fan ---------------------------------------------------------------------
      fan = buf[0];
      // Battery Voltage ---------------------------------------------------------
      batVolt = buf[2]*0.1216;
      if (batVolt != batVoltLast)
        updateBatVolt = 1;
      batVoltLast = batVolt;
      // PDM Global Error --------------------------------------------------------
      if (buf[1] == 1){
        pdmFaulted = true;
      }
      pdmError = buf[1];
    }
  }
}

void shiftLights() {
  // Should we add blinking when 12 lights are on??
  if(rpm > sRPM12) {
    // 12 lights
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED5+sLED6;
    SR1 += sLED1+sLED2+sLED3+sLED4+sLED5+sLED6;
  }
  else if(rpm > sRPM11) {
    // 11 lights 
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED5+sLED6;
    SR1 += sLED7+sLED8+sLED9+sLED10+sLED11;
  }
  else if(rpm > sRPM10) {
    // 10 lights 
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED5+sLED6;
    SR1 += sLED7+sLED8+sLED9+sLED10;
  }
  else if(rpm > sRPM9) {
    // 9 lights 
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED5+sLED6;
    SR1 += sLED7+sLED8+sLED9;
  }
  else if(rpm > sRPM8) {
    // 8 lights 
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED5+sLED6;
    SR1 += sLED7+sLED8;
  }
  else if(rpm > sRPM7) {
    // 7 lights 
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED5+sLED6;
    SR1 += sLED7;
  }
  else if(rpm > sRPM6) {
    // 6 lights 
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED5+sLED6;
  }
  else if(rpm > sRPM5) {
    // 5 lights 
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED5;
  }
  else if(rpm > sRPM4) {
    // 4 lights 
    SR2 += sLED1+sLED2+sLED3+sLED4;
  }
  else if(rpm > sRPM3) {
    // 3 lights 
    SR2 += sLED1+sLED2+sLED3;
  }
  else if(rpm > sRPM2) {
    // 2 lights 
    SR2 += sLED1+sLED2;
  }
  else if(rpm > sRPM1) {
    // 1 lights 
    SR2 += sLED1;
  }
}

void statusLights() {
  // TMP    FAN
  // BAT    PDM
  // OIL    ARM
  // TMP -----------------------------------------------------------------------
  if(engTemp < engTempLow){
    if(millis() - engTempFlashTime > engTempFlashLast){
      engTempFlashLast = millis();
      engTempFlash = engTempFlash*-1;
    }
    if(engTempFlash == 1) {
      SR6 -= ledLB;
    }
  } else if(engTemp < engTempMedLow) {
    // (blue) getting there..
    SR6 -= ledLB;
  } else if(engTemp > engTempHigh) {
    // (red) too hot!
    SR6 -= ledLR;
  } else if(engTemp > engTempMedHigh) {
    // (yellow) pretty hot..
    SR6 -= ledLR + ledLG;
  } else {
    // (green) good to go!
    SR6 -= ledLG;
  }
  // FAN -----------------------------------------------------------------------
  if(fan) {
    // purple if fan is on
    SR6 -= ledRR + ledRB;
  }
  // BAT -----------------------------------------------------------------------
  if(batVolt < batVoltLow) {
    //red, if voltage is less than 12.5v
    SR5 -= ledLR;
  } else if(batVolt < batVoltMed) {
    //yellow, if voltage is less than 13v
    SR5 -= ledLR + ledLG;
  } else {
    //otherwise green light
    SR5 -= ledLG;
  }
  // PDM -------------------------------------------------------------------------
  if (pdmError) {
    // red light if pdm error
    SR5 -= ledRR;
  } else if (pdmFaulted) {
    // yellow light if pdm fault
    SR5 -= ledRR +ledRG;
  } else {
    // green light if good
    SR5 -= ledRG;
  }
  // OIL -------------------------------------------------------------------------
  if (oilPres < oilPresLow) {
    // blinking red light if oil pressure is less than 23
    if(millis() - engTempFlashTime > oilPresFlashLast){
      oilPresFlashLast = millis();
      oilPresFlash = oilPresFlash*-1;
    }
    if(oilPresFlash == 1) {
      SR4 -= ledLR;
    }
  } else if (oilPres > oilPresHigh) {
    // red light if oil pressure is greater than 90
    SR4 -= ledLR;
  } else {
    SR4 -= ledLG;
  }
  // ARM -------------------------------------------------------------------------
  if (launchArm) { 
    SR4 -= ledRG + ledRR;
  } 
}

void gear() {
  // testing code 
//  ssCount++;
//  if(ssCount == 7) {
//    ssCount = 1;
//  }
//  switch(ssCount) {
//    case 1: SR3 -= seg1; break;
//    case 2: SR3 -= seg2; break;
//    case 3: SR3 -= seg3; break;
//    case 4: SR3 -= seg4; break;
//    case 5: SR3 -= seg5; break;
//    case 6: SR3 -= seg6; break;
//    default: SR3 -= seg1; break;
//  }
}

void dispLogo() {
  Serial.println("Displaying logo");
  display.clearDisplay();
  display.setTextColor(WHITE);  
  display.drawBitmap(0, 7, QFSAELogo, 128, 50, WHITE);
  display.display();
}
