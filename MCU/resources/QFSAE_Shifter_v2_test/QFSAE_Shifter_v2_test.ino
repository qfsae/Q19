/* 2017 Shifter Code V2
 * tesitng code w/ serial
 * working with auto shifting code
 */
#define tpsSig A0
#define clutchPot A1
// #define A4 - goes to can shield
// #define A5 - goes to can shield
#define upBut 12
//#define vrliSig 9
#define clutchSol 8
#define downSol 6
#define upSol 7
#define downBut 5
#define shiftCut 3
#define tachSig 2
#define shiftMode 1

int numberRPMSamples = 5;
long engineRpm = 0;
boolean safeShift = true;

int upShiftDelayTime = 200;
int downShiftDelayTime = 400;
int currTime = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(upBut, INPUT);
  pinMode(clutchSol, OUTPUT);
  pinMode(downSol, OUTPUT);
  pinMode(upSol, OUTPUT);
  pinMode(downBut, INPUT);
  pinMode(shiftCut, OUTPUT);
  pinMode(tachSig, INPUT);
  pinMode(shiftMode, INPUT);

  digitalWrite(clutchSol, LOW);
  digitalWrite(downSol, LOW);
  digitalWrite(upSol, LOW);
  digitalWrite(shiftCut, LOW);

  Serial.begin(9600);
  
}

void loop() {

  if(digitalRead(shiftMode)) {
    Serial.println("Auto Mode");
    autoMode(); 
  } else {
    manualMode();
   // Serial.println("Manual Mode");
  }
}

void autoMode() {

  engineRPM = getRPM();
  if (!digitalRead(upBut)) {
    Serial.println("AUTO UP Shift Requested");
    autoUpshift();
  } 
   if(!digitalRead(downBut)) {
   Serial.println("AUTO DOWN Shift Requested");
   autoDownshift();
   
   }
}

void autoUpshift () {
  /*check RPM 
   * int RPM = 
   * while (
   * wait till rpm reaches almost redline
   * timeout if not reached
   * call manualUpshift()
   */
  
}

void autoDownshift () {
/*
  if(checkSafeShift()) {
   //manualDownshift();
   blipDown(); 
  }
*/


}

void blipDown () {
  //wait for tps to reach tpsThres
  //manualDownshift(); 
   
}


void manualMode () {
  //Upshift 
  int time; 
  if (!digitalRead(upBut)) {
    Serial.println("Manual UP Shift Requested");
    time = millis();
    manualUpshift();
    time = millis() - time;
    Serial.print("Shift Time:  ");
    Serial.println(time);
    }    
    
    //Downshift
    if(!digitalRead(downBut)) {
      Serial.println("Manual DOWN Shift Requested");
      time = millis();
      
      manualDownshift();
      time = millis() - time;
      Serial.print("Shift Time:  ");  
      Serial.println(time);
    }
   
}

void manualDownshift () {
      digitalWrite(shiftCut, HIGH);
      digitalWrite(downSol, HIGH);
      delay(downShiftDelayTime);
      digitalWrite(shiftCut, LOW); 
      digitalWrite(downSol, LOW);  
}

void manualUpshift() {
      digitalWrite(shiftCut, HIGH);
      digitalWrite(upSol, HIGH);
      delay(upShiftDelayTime);
      digitalWrite(upSol, LOW); 
      digitalWrite(shiftCut, LOW); 
}

int getRPM () {

//returns the average frequency recorded
//PE3 outputs a 30% duty cycle

  long lowTimeTotal = 0;
  long highTimeTotal = 0;
  long measureNum = 0;
  long periodTotal = 0;
  long highTimeAvg = 0;

  for(byte j = 0; j <= numberRPMSamples; j++){    
     highTimeTotal = highTimeTotal + pulseIn(tachSig ,HIGH, 60);
     measureNum++;
    }
    
  highTimeAvg = highTimeTotal / numberRPMSamples;
  if (highTimeAvg>0 {
    engineRpm = 5/highTimeAvg;
  }
  return engineRpm;

}


void neutralFind() {
  manualDownshift(); 
  manualDownshift(); 
  manualDownshift();
      
      //digitalWrite(shiftCutPin, HIGH);
      digitalWrite(upSol, HIGH);
      delay(upShiftDelayTime/2);
      digitalWrite(upSol, LOW); 
      //digitalWrite(shiftCutPin, LOW);
}


byte getGear () {
  byte gear = 0;
  return gear;
}





