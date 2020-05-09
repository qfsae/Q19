#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <mcp_can.h>

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
#define OLED_RESET 12       //Rst
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
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
// Size Checking ----------------------------------------------------------------------
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// Inputs =============================================================================
#define cycButPIN     A0
#define pitComPIN     A1
#define launchArmPIN  A2
int pitCom = 0;

// Pit Communication ==================================================================
#define pitComDelay 1000 
boolean pit = false;
boolean gtfo = false;
boolean pitComCheck = false;
long pitComTime = 0;
boolean runPitCom = false;

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
#define latchPIN    4
#define clockPIN    7
#define dataPIN     3
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
#define sLED1   pinQ5 //SR2
#define sLED2   pinQ4 //SR2
#define sLED3   pinQ3 //SR2
#define sLED4   pinQ2 //SR2
#define sLED5   pinQ6 //SR1
#define sLED6   pinQ7 //SR1
#define sLED7   pinQ7 //SR2
#define sLED8   pinQ6 //SR2
#define sLED9   pinQ5 //SR1
#define sLED10  pinQ4 //SR1
#define sLED11  pinQ3 //SR1
#define sLED12  pinQ2 //SR1
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
#define ssTL pinQ5 //SR3
#define ssTR pinQ7 //SR3
#define ssBL pinQ3 //SR3
#define ssBR pinQ1 //SR3
#define ssTC pinQ6 //SR3
#define ssCC pinQ4 //SR3
#define ssBC pinQ2 //SR3
// Gear Ratios
#define primaryGearRatio  82/45
#define engineTeeth       11
#define sprocketTeeth     46
double gearRatio[6];
// 46 tooth gear
#define gearOneSlope    0.004545949
#define gearTwoSlope    0.00624494
#define gearThreeSlope  0.007820114
#define gearFourSlope   0.009063836
#define gearFiveSlope   0.010120148
#define gearSixSlope    0.010972012
int gearNum = 0;

// Others =============================================================================
#define runStartup false
#define gtfoFlashTime 100

void setup() { 
  Serial.begin(115200);
  
  // Initialize Shift Registers -------------------------------------------------------
  pinMode(latchPIN,OUTPUT);
  pinMode(clockPIN,OUTPUT);
  pinMode(dataPIN,OUTPUT);
  digitalWrite(latchPIN,LOW);
    shiftOut(dataPIN,clockPIN,MSBFIRST,255);
    shiftOut(dataPIN,clockPIN,MSBFIRST,255);
    shiftOut(dataPIN,clockPIN,MSBFIRST,255);
    shiftOut(dataPIN,clockPIN,MSBFIRST,255);
    shiftOut(dataPIN,clockPIN,MSBFIRST,0);
    shiftOut(dataPIN,clockPIN,MSBFIRST,0);
  digitalWrite(latchPIN,HIGH);

  // Initialize OLED ------------------------------------------------------------------
  display.begin(SSD1306_SWITCHCAPVCC);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.display();

  // Initialize CAN -------------------------------------------------------------------
  boolean x = true;
  while (CAN_OK != CAN.begin(CAN_250KBPS))
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    if (x) {
      dispERROR();
      x = false;
    }
    digitalWrite(latchPIN,LOW);
      shiftOut(dataPIN,clockPIN,MSBFIRST,111);
      shiftOut(dataPIN,clockPIN,MSBFIRST,111);
      shiftOut(dataPIN,clockPIN,MSBFIRST,111);
      shiftOut(dataPIN,clockPIN,MSBFIRST,147);
      shiftOut(dataPIN,clockPIN,MSBFIRST,0);
      shiftOut(dataPIN,clockPIN,MSBFIRST,0);
    digitalWrite(latchPIN,HIGH);
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
  
  // Initialize Others ----------------------------------------------------------------
  pinMode(cycButPIN,INPUT);
  pinMode(launchArmPIN,INPUT);
  pinMode(pitComPIN,INPUT);
  if (runStartup){
    delay(500);
    digitalWrite(latchPIN,LOW);
      shiftOut(dataPIN,clockPIN,MSBFIRST,0);
      shiftOut(dataPIN,clockPIN,MSBFIRST,0);
      shiftOut(dataPIN,clockPIN,MSBFIRST,0);
      shiftOut(dataPIN,clockPIN,MSBFIRST,0);
      shiftOut(dataPIN,clockPIN,MSBFIRST,255);
      shiftOut(dataPIN,clockPIN,MSBFIRST,255);
    digitalWrite(latchPIN,HIGH);
    delay(500);
    dispLogo();
    delay(500);
    int loadBarW = 0;
    for (int loadBarX = 0; loadBarX < 128; loadBarX += 2){
      loadBarW=random(10)+2;
      display.drawRect(loadBarX,59,loadBarW,2,WHITE);
      display.display();
    }
    delay(100);
  }

  // Gear Ratios
//  gearRatio[0] = 34/12;
//  gearRatio[1] = 33/16;
//  gearRatio[2] = 28/17;
//  gearRatio[3] = 7/19;
//  gearRatio[4] = 28/22;
//  gearRatio[5] = 27/23;
//  for (int i = 0; i < 6; i++) {
//    int speedMax = (rpmMax*wheelDiameter*PI*0.06 )/(primaryGearRatio*finalGearRatio*gearRatio[i]);
//    int speedMin = (rpmMin*wheelDiameter*PI*0.06 )/(primaryGearRatio*finalGearRatio*gearRatio[i]);
//    gearSlope[i] = (speedMax - speedMin)/(rpmMax - rpmMin);
//  }
}

void loop() {
  SR1 = 0;
  SR2 = 0;
  SR3 = 255;
  SR4 = 255;
  SR5 = 255;
  SR6 = 255;

  launchArm = digitalRead(launchArmPIN);
  pitCom = analogRead(pitComPIN);

  if (runPitCom) {
    if (pitCom > 200) {
      if (!pitComCheck) {
        pitComTime = millis();
        pitComCheck = true;
      }
      if (millis() - pitComDelay > pitComTime) {
        if (pitCom > 475) {
          gtfo = true;
        } else {
          pit = true;
        } 
      } 
    } else {
      pitComCheck = false;
    } 
  }

  readCAN();
  gear();
  warningLights();
  shiftLights();
  oledControl();
  
  digitalWrite(latchPIN,LOW);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR6);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR5);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR4);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR3);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR2);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR1);
  digitalWrite(latchPIN,HIGH);
}

void readCAN(void){
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

void oledControl(void){
  if (digitalRead(cycButPIN) == 1){
    if(allowCycle == 0){
      allowCycle = 1;
      OLEDMode++;
    }
  } else {
    allowCycle = 0;
  }
  if (OLEDMode > 6)
    OLEDMode = 1;

  if (gtfo) {
    dispGTFO();
    OLEDMode = 0;
  } else if (pit) {
    dispPIT();
    OLEDMode = 0;
  } else {
    if (OLEDMode == 1 && switchMode != 1) {
      switchMode = 1;
      dispLogo();
    } else if (OLEDMode == 2 && (switchMode != 2 || updateEngTemp == 1)) {
      switchMode = 2;
      updateEngTemp = 0;
      dispEngTemp();
    } else if (OLEDMode == 3 && (switchMode != 3 || updateBatVolt == 1)) {
      switchMode = 3;
      updateBatVolt = 0;
      dispBatVolt();
    } else if (OLEDMode == 4 && (switchMode != 4 || updateOilPres == 1)) {
      switchMode = 4;
      updateOilPres = 0;
      dispOilPres();
    } else if (OLEDMode == 5 && (switchMode != 5 || updateRPM)) {
      switchMode = 5;
      updateRPM = false;
      dispRPM();
    } else if (OLEDMode == 6 && (switchMode != 6 || updateTPS == 1)) {
      updateTPS = 0;
      switchMode = 6;
      dispTPS();
    } else if (OLEDMode == 7 && switchMode != 7) {
      switchMode = 7;
      dispERROR();
    }
  }
}

void shiftLights(void){
  if (rpm > sRPM12){
    if ((millis() - rpmFlashTime) > rpmFlashLast){
      rpmFlashLast = millis();
      rpmFlash = rpmFlash*-1;
    }
    if (rpmFlash == 1){
      SR2 += sLED1+sLED2+sLED3+sLED4+sLED7+sLED8;
      SR1 += sLED5+sLED6+sLED9+sLED10+sLED11+sLED12;
    }
  }
  else if (rpm > sRPM11){
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED7+sLED8;
    SR1 += sLED5+sLED6+sLED9+sLED10+sLED11;
  }
  else if (rpm > sRPM10){
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED7+sLED8;
    SR1 += sLED5+sLED6+sLED9+sLED10;
  }
  else if (rpm > sRPM9){
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED7+sLED8;
    SR1 += sLED5+sLED6+sLED9;
  }
  else if (rpm > sRPM8){
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED7+sLED8;
    SR1 += sLED5+sLED6;
  }
  else if (rpm > sRPM7){
    SR2 += sLED1+sLED2+sLED3+sLED4+sLED7;
    SR1 += sLED5+sLED6;
  }
  else if (rpm > sRPM6){
    SR2 += sLED1+sLED2+sLED3+sLED4;
    SR1 += sLED5+sLED6;
  }
  else if (rpm > sRPM5){
    SR2 += sLED1+sLED2+sLED3+sLED4;
    SR1 += sLED5;
  }
  else if (rpm > sRPM4){
    SR2 += sLED1+sLED2+sLED3+sLED4;
  }
  else if (rpm > sRPM3){
    SR2 += sLED1+sLED2+sLED3;
  }
  else if (rpm > sRPM2){
    SR2 += sLED1+sLED2;
  }
  else if (rpm > sRPM1){
    SR2 += sLED1;
  }
}

void warningLights(void){
  // TMP    FAN
  // BAT    PDM
  // OIL    ARM
  // TMP -------------------------------------------------------------------------
  if (engTemp < engTempLow){
    if (millis() - engTempFlashTime > engTempFlashLast){
      engTempFlashLast = millis();
      engTempFlash = engTempFlash*-1;
    }
    if (engTempFlash == 1) {
      SR6 -= ledLB;
    }
  } else if(engTemp < engTempMedLow) {
    SR6 -= ledLB;
  } else if(engTemp > engTempHigh) {
    SR6 -= ledLR;
  } else if(engTemp > engTempMedHigh) {
    SR6 -= ledLR + ledLG;
  } else {
    SR6 -= ledLG;
  }
    // FAN -------------------------------------------------------------------------
  if (fan) {
    SR6 -= ledRR + ledRB;
  }
  // BAT -------------------------------------------------------------------------
  if (batVolt < batVoltLow) {
    SR5 -= ledLR;
  } else if(batVolt < batVoltMed) {
    SR5 -= ledLR + ledLG;
  } else {
    SR5 -= ledLG;
  }
  // PDM -------------------------------------------------------------------------
  if (pdmError) {
    SR5 -= ledRR;
  } else if (pdmFaulted) {
    SR5 -= ledRR +ledRG;
  } else {
    SR5 -= ledRG;
  }
  // OIL -------------------------------------------------------------------------
  if (oilPres < oilPresLow()) {
    SR4 -= ledLR;
  } else if (oilPres > oilPresHigh()) {
    SR4 -= ledLR;
  } else {
    SR4 -= ledLG;
  }
  // ARM -------------------------------------------------------------------------
  if (launchArm){
    SR4 -= ledRG + ledRR;
  }
}

void gear(void){
  if (avgWheelSpeed < (rpm+100)*gearOneSlope) {
    gearNum = 1;
    SR3 -= ssTR+ssBR;
  } else if (avgWheelSpeed < (rpm+100)*gearTwoSlope) {
    gearNum = 2;
    SR3 -= ssTC+ssTR+ssCC+ssBL+ssBC;
  } else if (avgWheelSpeed < (rpm+100)*gearThreeSlope) {
    gearNum = 3;
    SR3 -= ssTC+ssTR+ssCC+ssBR+ssBC;
  } else if (avgWheelSpeed < (rpm+100)*gearFourSlope) {
    gearNum = 4;
    SR3 -= ssTR+ssCC+ssTL+ssBR;
  } else if (avgWheelSpeed < (rpm+100)*gearFiveSlope) {
    gearNum = 5;
    SR3 -= ssTC+ssTL+ssCC+ssBR+ssBC;
  } else if (avgWheelSpeed < (rpm+100)*gearSixSlope) {
    gearNum = 6;
    SR3 -= ssTL+ssBL+ssCC+ssBR+ssBC;
  } else {
    gearNum = 0;
    SR3 -= ssTL+ssBL+ssTR+ssBR+ssBC+ssTC;
  }
}

int oilPresLow(){
  return 23;
}

int oilPresHigh(){
  return 90;
}

void dispLogo(void){
  display.clearDisplay();
  display.drawBitmap(0, 7, QFSAELogo, 128, 50, WHITE);
  display.display();
}

void dispEngTemp(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Eng Temp");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.println(engTemp,0);
  display.display();
}

void dispBatVolt(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Bat Volt");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.println(batVolt,1);
  display.display();
}

void dispOilPres(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Oil Pres");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.println(oilPres,0);
  display.display();
}

void dispRPM(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("RPM");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(4);
  if (rpm < 10)
    display.setCursor(96,25);
  else if (rpm < 100)
    display.setCursor(72,25);
  else if (rpm < 1000)
    display.setCursor(48,25);
  else if (rpm < 10000)
    display.setCursor(24,25);
  else
    display.setCursor(0,25);
  display.println(rpm);
  display.display();
}

void dispA(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Open A");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.println("A");
  display.display();
}

void dispB(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Open B");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.println("B");
  display.display();
}

void dispTPS(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("TPS");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.print(tps);
  display.println("%");
  display.display();
}

void dispERROR(void){
  display.clearDisplay();
  display.setTextSize(5);
  display.setCursor(22,14);
  display.println("ERR");
  display.display();
}

void dispGTFO(void){
  display.clearDisplay();
  display.setTextSize(5);
  display.setCursor(7,14);
  display.println("GTFO");
  display.display();
  
  if (millis() - gtfoFlashTime > gtfoFlashLast){
    gtfoFlashLast = millis();
    gtfoFlash = gtfoFlash*-1;
  }
  if (gtfoFlash == 1){
    SR1 = 255;
    SR2 = 255;
    SR4 = 0;
    SR5 = 0;
    SR6 = 0;
  } else {
    SR1 = 0;
    SR2 = 0;
    SR4 = 255;
    SR5 = 255;
    SR6 = 255;
  }
}

void dispPIT(void){
  display.clearDisplay();
  display.setTextSize(5);
  display.setCursor(22,14);
  display.println("PIT");
  display.display();
  SR4 = 0;
  SR5 = 0;
  SR6 = 0;
}

