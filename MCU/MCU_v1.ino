/* Arduino Code for Formula SAE Team MCU (Master Control Unit)

// PIN DEFINITIONS 
//Analog
A0. TPS1_Signal
A1. TPS2_Signal 
A2. APPS1_Signal
A3. APPS2_Signal
A4. Traction_Signal

//PWM
PWM3. Can Intercept
PWM4. TPS_Speed


//Digital
18. DownButtonSignal (TX1)
19. UpButtonSignal	(RX1)
22. TPS_Direction
23. ShiftCut_Ard
24. ETC Error Flag
25. EngNeut_Ard
26. UpSol
27. DownSol
50. CAN_CS
51. MOSI
52. MISO
53. SCK

*/

//Inputs
#define DownButSig 18


//Outputs
#define TPS_ECU 
#define TPS_Speed 5
#define CAN_INT 3

//ETC Stuff
const int TPS1Pin = A0;  //Pin of TPS1
const int TPS2Pin = A1;  //Pin of TPS2
const int APPS1Pin = A2; // TEMP potentiometer
const int torquePin = 9; //PWM pin to motor driver
const int directionPin = 7; //directionPin

//PID tunings
const double Kp = 1;
const double Ki = 0.00001;
const double Kd = 0.00;

//ETB Constants
const int TPS1Close = 931;
const int TPS1Open = 50;
const int TPS2Close = 90;
const int TPS2Open = 971;

//APPS Constants
const int APPS1Close = 0;
const int APPS1Open = 1024;
const int APPS2Close = 0;
const int APPS2Open = 1024;

double Input;
double OutputTorque;
double DesiredPosition;
int TPS1;
int TPS2;
int TPSdiff;
int APPS1;
int APPS2;
int APPSdiff;

PID myPID(&Input, &OutputTorque, &DesiredPosition, Kp, Ki, Kd, DIRECT);



void setup(){
	Init();
	attachInterrupt(digitalPinToInterrupt(upShift), upShift, CHANGE);
	attachInterrupt(digitalPinToInterrupt(downShift), downShift, CHANGE);
	Input = 0;
	DesiredPosition = 0;
	myPID.SetMode(AUTOMATIC);

	pinMode(directionPin, OUTPUT);
	pinMode(torquePin, OUTPUT);

	myPID.SetSampleTime(10);
}

void Init(){
	//Function that will initailize all pin modes and starting values 
	
	//Inputs
	pinMode(DownButSig, INPUT);
	pinMode(DownButSig, INPUT);
	pinMode(DownButSig, INPUT);
	pinMode(DownButSig, INPUT);
	pinMode(DownButSig, INPUT);
	pinMode(DownButSig, INPUT);
	pinMode(DownButSig, INPUT);
	
	pinMode(DownButSig, INPUT);

	
	//Outputs
	pinMode(TPS_ECU, OUTPUT);
	pinMode(ShiftCut, OUTPUT);
	pinMode(TPS_Direction, OUTPUT);
	pinMode(TPS_Speed, OUTPUT);
	
	//Interrupt
	attachInterrupt(CAN_INT, CANInterupt, __);
}

//Function Protocols 
void CANInterupt();
int getRPM();
void checkSignals();
void blipDown();

void loop(){
	checkSignals();
	
	//ETC Stuff -> Look into making this a function 
	TPS1 = constrain(analogRead(TPS1Pin), TPS1Open, TPS1Close); // Range: 931 - 50
	TPS2 = constrain(analogRead(TPS2Pin), TPS2Close, TPS2Open); // Range: 90 - 971
	TPSdiff = (TPS1Close - TPS1Open) - ((TPS1 - TPS1Open) + (TPS2 - TPS2Close));

	if(TPSdiff > ((TPS1Close - TPS1Open)*0.1)){ // TPS1 and TPS2 differ by more than 10%
		analogWrite(torquePin, 0);
		Serial.println("ERROR 1");
	}

	Input = map(TPS1, TPS1Close, TPS1Open, 0, 1024);

	APPS1 = constrain(analogRead(APPS1Pin), APPS1Close, APPS1Open); // Range: 0 - 1024
	//APPS2 = analogRead(APPS2Pin); // Range: 0 - 1024
	//APPSdiff = APPS1 - APPS2;
  
	if(Input < APPS1){
		myPID.SetControllerDirection(DIRECT);
		digitalWrite(directionPin,LOW);    
	}
	else	{
		myPID.SetControllerDirection(REVERSE);
		digitalWrite(directionPin,HIGH);
	}


	DesiredPosition = APPS1;
	myPID.Compute();
	//Update Write Function AFTER Compute
	analogWrite(torquePin, OutputTorque);
}

void CANInterupt
	
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

void blipDown () {
  //wait for tps to reach tpsThres
  //manualDownshift(); 
   
}

void checkSignals(){
	//Test each individual signal to make sure that it is plausible
	

}

