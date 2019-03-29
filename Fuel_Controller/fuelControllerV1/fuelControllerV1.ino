//First testing for Q19 Fuel pump 
// Need to figure out how to offset the PWM 
#include <PID_v1.h>

#define pressureReading A0
#define output 3
#define pressureUnderLED 1
#define pumpingLED 2

const int pumpThreshold = 400;


double input;
double outputPWM;
double pressureDifference;

//PID Tunings
const double kp = 1;
const double ki = 0.00;
const double kd = 0.00;

PID myPID(&input, &outputPWM, &pumpThreshold, kp, ki, kd, DIRECT);

void setup(){
  input = 0;
  pressureDifference = 0;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);

  pinMode(pressureReading, INPUT);
  pinMode(output, OUTPUT);

  pinMode(pressureUnderLED, OUTPUT);
  pinMode(pumpingLED, OUTPUT);

  Serial.begin(9600);
  Serial.print("Desired Pressure\t Current Pressure\t Pressure Difference\n");
}

void loop(){
  Serial.print(pumpThreshold);
  Serial.print("\t");
  
  currentPressure = constrain(analogRead(pressureReading));
  Serial.print(currentPressure);
  Serial.print("\t");
  
  pressureDifference = (currentPressure - pumpThreshold); //positive if current is greater, negative if not
  Serial.print(pressureDifference);
  Serial.print("\n");

  if(currentPressure < pumpThreshold){
    //want to increase the pressure, therefore activates the fuel pump
    digitalWrite(pressureUnderLED, HIGH);
    myPID.SetControllerDirection(DIRECT);
  }
  else {
    myPID.SetControllerDirection(IDLE); // set controller to idle until pressure evens out 
    //not really doing much here 
  }
  myPID.Compute();
}
