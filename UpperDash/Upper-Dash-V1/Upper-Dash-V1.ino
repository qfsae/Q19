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
int SR1 = 0, SR2 = 0, SR3 = 0, SR4 = 0, SR5 = 0, SR6 = 0;

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

// 7 Segment ==========================================================================
#define ssC pinQ1 //SR3
#define ssD pinQ2 //SR3  
#define ssE pinQ3 //SR3
#define ssG pinQ4 //SR3
#define ssF pinQ5 //SR3
#define ssA pinQ6 //SR3
#define ssB pinQ7 //SR3
int ssCount = 1;
int seg1;
int seg2;
int seg3;
int seg4;
int seg5;
int seg6;

void setup() {
  Serial.begin(115200);

  randomSeed(analogRead(0));

  Serial.println("Begin Initialize shift registers");

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

  // 7 Segment values
  seg1 = 255 - (ssB+ssC); // 1
  seg2 = 255 - (ssA+ssB+ssD+ssE+ssG); // 2
  seg3 = 255 - (ssA+ssB+ssC+ssD+ssG); // 3 
  seg4 = 255 - (ssB+ssC+ssF+ssG); // 4
  seg5 = 255 - (ssA+ssC+ssD+ssF+ssG); // 5
  seg6 = 255 - (ssA+ssC+ssD+ssE+ssF+ssG); // 6
  Serial.println(seg1);
  Serial.println(seg2);
  Serial.println(seg3);
  Serial.println(seg4);
  Serial.println(seg5);
  Serial.println(seg6);

}

void loop() {

    SR2 = 255;
    SR1 = 255;
    
//    for(int i = 0; i<256; i++) {
//      SR4 = i;
//      SR5 = i;
//      SR6 = i;
//    }

    //statusLEDS();
    gear();
    
      digitalWrite(latchPIN,LOW);
        shiftOut(dataPIN,clockPIN,MSBFIRST,SR6);
        shiftOut(dataPIN,clockPIN,MSBFIRST,SR5);
        shiftOut(dataPIN,clockPIN,MSBFIRST,SR4);
        shiftOut(dataPIN,clockPIN,MSBFIRST,SR3);
        shiftOut(dataPIN,clockPIN,MSBFIRST,SR2);
        shiftOut(dataPIN,clockPIN,MSBFIRST,SR1);
      digitalWrite(latchPIN,HIGH);

      delay(500);
}

void gear() {
  ssCount++;
  if(ssCount == 7) {
    ssCount = 1;
  }
  switch(ssCount) {
    case 1: SR3 = seg1; break;
    case 2: SR3 = seg2; break;
    case 3: SR3 = seg3; break;
    case 4: SR3 = seg4; break;
    case 5: SR3 = seg5; break;
    case 6: SR3 = seg6; break;
    default: SR3 = seg1; break;
  }
  Serial.println(SR3);
}

void statusLEDS() {
  
}
