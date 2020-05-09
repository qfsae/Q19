#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>


#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

// OLED ======================================================================
#define OLED_MOSI  9        //Data
#define OLED_CLK   8        //Clk
#define OLED_DC    5//11    //DC/SA0
#define OLED_CS    6//13    //CS
#define OLED_RESET A1       //Rst
#define cyclePin   A0       //Dash Cycle
Adafruit_SSD1306 display(128, 64, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
//int OLEDMode = 1;
//int allowCycle = 0;
//int switchMode = 0;
unsigned long previousMillis = 0;
const long interval = 10000;
int currentCycle = 1;


// CAN values ===============================================================
unsigned char len = 0;
unsigned char buf[8];
boolean fan = false;
boolean pdmError = false;
boolean pdmFaulted = false;
float oilPres = 70;
float oilPresLow = 23;
float oilPresHigh = 90;
long oilPresFlashLast = 0;
byte oilPresFlash = 1;
int updateOilPres = 0;

// LED Flashing =======================================================================
// RPM --------------------------------------------------------------------------------
long rpmFlashLast = 0;
byte rpmFlash = 1;
// Engine Temp ------------------------------------------------------------------------
long engTempFlashLast = 0;
byte engTempFlash = 1;
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
int seg0 = (ssA + ssB + ssC + ssD + ssE + ssF); // 0
int seg1 = (ssB + ssC); // 1
int seg2 = (ssA + ssB + ssD + ssE + ssG); // 2
int seg3 = (ssA + ssB + ssC + ssD + ssG); // 3
int seg4 = (ssB + ssC + ssF + ssG); // 4
int seg5 = (ssA + ssC + ssD + ssF + ssG); // 5
int seg6 = (ssA + ssC + ssD + ssE + ssF + ssG); // 6
int seg7 = (ssA + ssB + ssC); // 7
int seg8 = (ssA + ssB + ssC + ssD + ssE + ssF + ssG); // 8
int seg9 = (ssA + ssB + ssC + ssD + ssF + ssG); // 9

// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;            // was 53

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
  Serial.begin(115200);
  // Initialize CAN ------------------------------------------------------------------
  while (CAN_OK != CAN.begin(CAN_250KBPS))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(10000);
  }
  Serial.println("CAN BUS Shield init ok!");

  // Initialize Shift Registers -------------------------------------------------------
  pinMode(latchPIN, OUTPUT);
  pinMode(clockPIN, OUTPUT);
  pinMode(dataPIN, OUTPUT);
  pinMode(cyclePin, INPUT);
  digitalWrite(latchPIN, LOW);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR6);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR5);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR4);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR3);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR2);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR1);
  digitalWrite(latchPIN, HIGH);
  Serial.println("Shift registers initialized!");

  // Initialize OLED ------------------------------------------------------------------
  display.begin(SSD1306_SWITCHCAPVCC);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.display();

  Serial.println("Good to go!");
}

int rpm = 0;
float tps = 0;
float fuel = 0;
float batVolt = 0;
float Coolant = 0;
float engTemp = 0;
float engTempLast = 0;

void loop()
{
  // reset shift registers to base values (all lights off)
  SR1 = 0;
  SR2 = 0;
  SR3 = 255;
  SR4 = 255;
  SR5 = 255;
  SR6 = 255;

  readCAN();

  shiftLights();

  statusLights();

  sevenSegment();

  updateOLED();

  Serial.println(currentCycle);

  //push new values to shift registers
  digitalWrite(latchPIN, LOW);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR6);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR5);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR4);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR3);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR2);
  shiftOut(dataPIN, clockPIN, MSBFIRST, SR1);
  digitalWrite(latchPIN, HIGH);

}

void readCAN() {

  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    unsigned int canId = CAN.getCanId();

    //Serial.println("-----------------------------");
    //Serial.print("Get data from ID: ");
    //Serial.println(canId, HEX);

    if (canId == 0xF048) {
      rpm = buf[1] * 256 + buf[0];
      tps = buf[3] * 256 + buf[2];
      if (tps > 32767)
        tps -= 65536;
      tps = tps * 0.1;
    }
    else if (canId == 0xF548) {
      batVolt = buf[1] * 256 + buf[0];
      if (batVolt > 32767)
        batVolt -= 65536;
      batVolt = batVolt * 0.01;
      Coolant = buf[5] * 256 + buf[4];
      if (Coolant > 32767)
        Coolant -= 65536;
      Coolant = Coolant * 0.1;
      //          Serial.print("Battery Voltage: ");
      //          Serial.println(batVolt);
    }
    else if (canId == 0xF448) {
      fuel = buf[1] * 256 + buf[0];
      if (fuel > 32767)
        fuel -= 65536;
      fuel = fuel * 0.2;
      //Serial.print("Fuel Flow: ");
      //Serial.println(batVolt);
    }
    //Serial.println("Fuel\t Volt\t tps\t rpm\t Coolant");
    //        Serial.print(fuel);
    //        Serial.print("\t ");
    //        Serial.print(batVolt);
    //        Serial.print("\t ");
    //        Serial.print(tps);
    //        Serial.print("\t ");
    //        Serial.print(rpm);
    //        Serial.print("\t ");
    //        Serial.println(Coolant);
  }
}

void dispBatVolt(void) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Bat. Volt");
  display.drawLine(0, 18, 128, 18, WHITE);
  display.setTextSize(5);
  display.setCursor(0, 25);
  display.print(batVolt);
  display.println("V");
  display.display();
}

void dispEngTemp(void) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Eng. Temp");
  display.drawLine(0, 18, 128, 18, WHITE);
  display.setTextSize(5);
  display.setCursor(0, 25);
  display.print(Coolant);
  display.display();
}

void updateOLED() {
  if (digitalRead(cyclePin) == 0) {
    currentCycle %= 2;
    currentCycle++;
    delay(500);

    switch (currentCycle) {
      case 1: dispBatVolt(); break;
      case 2: dispEngTemp(); break;
    }
  }
  
  //update OLED every 10 seconds
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    //save last time oled was updated
    previousMillis = currentMillis;
    switch (currentCycle) {
      case 1: dispBatVolt(); break;
      case 2: dispEngTemp(); break;
    }
  }
}

void shiftLights() {
  // Should we add blinking when 12 lights are on??
  if (rpm > sRPM12) {
    // 12 lights
    SR2 += sLED1 + sLED2 + sLED3 + sLED4 + sLED5 + sLED6;
    SR1 += sLED1 + sLED2 + sLED3 + sLED4 + sLED5 + sLED6;
  }
  else if (rpm > sRPM11) {
    // 11 lights
    SR2 += sLED1 + sLED2 + sLED3 + sLED4 + sLED5 + sLED6;
    SR1 += sLED7 + sLED8 + sLED9 + sLED10 + sLED11;
  }
  else if (rpm > sRPM10) {
    // 10 lights
    SR2 += sLED1 + sLED2 + sLED3 + sLED4 + sLED5 + sLED6;
    SR1 += sLED7 + sLED8 + sLED9 + sLED10;
  }
  else if (rpm > sRPM9) {
    // 9 lights
    SR2 += sLED1 + sLED2 + sLED3 + sLED4 + sLED5 + sLED6;
    SR1 += sLED7 + sLED8 + sLED9;
  }
  else if (rpm > sRPM8) {
    // 8 lights
    SR2 += sLED1 + sLED2 + sLED3 + sLED4 + sLED5 + sLED6;
    SR1 += sLED7 + sLED8;
  }
  else if (rpm > sRPM7) {
    // 7 lights
    SR2 += sLED1 + sLED2 + sLED3 + sLED4 + sLED5 + sLED6;
    SR1 += sLED7;
  }
  else if (rpm > sRPM6) {
    // 6 lights
    SR2 += sLED1 + sLED2 + sLED3 + sLED4 + sLED5 + sLED6;
  }
  else if (rpm > sRPM5) {
    // 5 lights
    SR2 += sLED1 + sLED2 + sLED3 + sLED4 + sLED5;
  }
  else if (rpm > sRPM4) {
    // 4 lights
    SR2 += sLED1 + sLED2 + sLED3 + sLED4;
  }
  else if (rpm > sRPM3) {
    // 3 lights
    SR2 += sLED1 + sLED2 + sLED3;
  }
  else if (rpm > sRPM2) {
    // 2 lights
    SR2 += sLED1 + sLED2;
  }
  else if (rpm > sRPM1) {
    // 1 lights
    SR2 += sLED1;
  }
}

void statusLights() {
  // TMP    FAN
  // BAT    PDM
  // OIL    ARM
  // TMP -----------------------------------------------------------------------
  if (engTemp < engTempLow) {
    if (millis() - engTempFlashTime > engTempFlashLast) {
      engTempFlashLast = millis();
      engTempFlash = engTempFlash * -1;
    }
    if (engTempFlash == 1) {
      SR6 -= ledLB;
    }
  } else if (engTemp < engTempMedLow) {
    // (blue) getting there..
    SR6 -= ledLB;
  } else if (engTemp > engTempHigh) {
    // (red) too hot!
    SR6 -= ledLR;
  } else if (engTemp > engTempMedHigh) {
    // (yellow) pretty hot..
    SR6 -= ledLR + ledLG;
  } else {
    // (green) good to go!
    SR6 -= ledLG;
  }
  // FAN -----------------------------------------------------------------------
  if (fan) {
    // purple if fan is on
    SR6 -= ledRR + ledRB;
  }
  // BAT -----------------------------------------------------------------------
  if (batVolt < batVoltLow) {
    //red, if voltage is less than 12.5v
    SR5 -= ledLR;
  } else if (batVolt < batVoltMed) {
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
    SR5 -= ledRR + ledRG;
  } else {
    // green light if good
    SR5 -= ledRG;
  }
  // OIL -------------------------------------------------------------------------
  if (oilPres < oilPresLow) {
    // blinking red light if oil pressure is less than 23
    if (millis() - engTempFlashTime > oilPresFlashLast) {
      oilPresFlashLast = millis();
      oilPresFlash = oilPresFlash * -1;
    }
    if (oilPresFlash == 1) {
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

void sevenSegment() {
  if (tps >= 0 && tps < 10) {
    SR3 -= seg0;
  } else if (tps >= 10 && tps < 20) {
    SR3 -= seg1;
  } else if (tps >= 20 && tps < 30) {
    SR3 -= seg2;
  } else if (tps >= 30 && tps < 40) {
    SR3 -= seg3;
  } else if (tps >= 40 && tps < 50) {
    SR3 -= seg4;
  } else if (tps >= 50 && tps < 60) {
    SR3 -= seg5;
  } else if (tps >= 60 && tps < 70) {
    SR3 -= seg6;
  } else if (tps >= 70 && tps < 80) {
    SR3 -= seg7;
  } else if (tps >= 80 && tps < 90) {
    SR3 -= seg8;
  } else if (tps >= 90 && tps < 100) {
    SR3 -= seg9;
  }
}

void initSequence() {
  // fancy startup sequence
  // display logo on OLED
  // light each shift light one at a time up and down
  for (int i = 1; i <= 24; i++) {
    switch (i) {
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
    SR4 = colours[random(0, 3)];
    SR5 = colours[random(0, 3)];
    SR6 = colours[random(0, 3)];

    digitalWrite(latchPIN, LOW);
    shiftOut(dataPIN, clockPIN, MSBFIRST, SR6);
    shiftOut(dataPIN, clockPIN, MSBFIRST, SR5);
    shiftOut(dataPIN, clockPIN, MSBFIRST, SR4);
    shiftOut(dataPIN, clockPIN, MSBFIRST, SR3);
    shiftOut(dataPIN, clockPIN, MSBFIRST, SR2);
    shiftOut(dataPIN, clockPIN, MSBFIRST, SR1);
    digitalWrite(latchPIN, HIGH);

    delay(25);
    Serial.println("Done startup Sequence");
  }
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
