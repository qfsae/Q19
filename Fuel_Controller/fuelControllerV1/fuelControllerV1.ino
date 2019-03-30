#include <PID_v1.h>


const int pressureReading = A0; //Analog input from pressure sensor
const int pumpPWM = 9; //PWM output from arduino

//PWM output must have frequency >1000Hz as well as >2.5V Amplitude

//for testing
const int pressureUnderLED = 3;


//to use in main loop
int setPWM = 0;
int currentPressure = 0;
double pressureDifference;

//PID Control Inputs
double outputPWM;
double inputPressure;
double pressureThreshold = 400;

//PID Tunings
const double kp = 0;
const double ki = 0.00;
const double kd = 0.00;

PID myPID(&inputPressure, &outputPWM, &pressureThreshold, kp, ki, kd, DIRECT);

void setup(){
  inputPressure = 0;
  pressureDifference = 0;
  
 setPWMFrequency(9, 8);
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(2);
  
  setPWM = 120; //resting PWM signal
  analogWrite(pumpPWM, setPWM);
  
  pinMode(pressureReading, INPUT);
  pinMode(pumpPWM, OUTPUT);

  pinMode(pressureUnderLED, OUTPUT);
  digitalWrite(pressureUnderLED, LOW);

  Serial.begin(9600);
  Serial.print("Desired Pressure\t Current Pressure\t Pressure Difference\t OutputPWM\n");
}

void loop(){
  Serial.print(pressureThreshold);
  Serial.print("\t");
  
  inputPressure = analogRead(pressureReading);
  Serial.print(inputPressure);
  Serial.print("\t");
  
  pressureDifference = (inputPressure - pressureThreshold); //positive if current is greater, negative if not
  Serial.print(pressureDifference);
  Serial.print("\n");

  if(pressureDifference < 0){
    Serial.println("Fuel pump requires more pressure, increasing PDM Signal\n");
    delay(100);
    //want to increase the pressure, activate the fuel pump
    digitalWrite(pressureUnderLED, HIGH);
    setPWM = 240;
    Serial.println("Fuel pump PWM is now: " + setPWM);
    delay(100);
    analogWrite(pumpPWM, setPWM);
    //myPID.SetControllerDirection(DIRECT);
  }
  else {
    //wants to let pressure coast until equal 
    digitalWrite(pressureUnderLED, LOW);
    setPWM = 60;
//    myPID.SetControllerDirection(); // set controller to idle until pressure evens out 
//    //not really doing much here 

  Serial.print("\t");
  Serial.print(setPWM);
  Serial.println();
  delay(100);
  myPID.Compute();
}
}
void setPWMFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
