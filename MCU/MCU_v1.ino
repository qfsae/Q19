/* Arduino Code for Formula SAE Team MCU (Master Control Unit)







-----------------Wiring Schematic-------------------

INPUTS:


Logic psuedocode 

Shifter Buttons are interupt pins 
-> Copy from Q17 Shifting Code 

ETC Code: PID Controlled System
PID: Proportional Integral Differential 

Need to tune each component 
Proportaional control: Error would be 90 when your 0 away, then90 times some value is how much you need to move 

PI Control: Proportional and integral, similar to proportional but with the integral term it helps prevent any overshooting and oscillations, ensure that final error is 0 
PID: Add a differential gain which calculates the ROC, 

If you control these 3, very nice step response -> Will need to do with throttle body 


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
void setup(){
	Init();
	attachInterrupt(digitalPinToInterrupt(upShift), upShift, CHANGE);
	attachInterrupt(digitalPinToInterrupt(downShift), downShift, CHANGE);
}

void loop(){
	Check();
	
	
}

void Check(){
	//Test each individual signal to make sure that it is plausible
	



