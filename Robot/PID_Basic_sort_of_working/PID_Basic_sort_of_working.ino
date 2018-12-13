/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
int encoder_pin = 2;  // The pin the encoder is connected           
unsigned int rpm;     // rpm reading
volatile byte pulses;  // number of pulses
unsigned long timeold; 
// The number of pulses per revolution
// depends on your index disc!!
unsigned int pulsesperturn = 20;

 void counter()
 {
    //Update count
      pulses++;    
 }

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,.7,.6,0, DIRECT);
//PID myPID(&Input, &Output, &Setpoint,3,1,5, DIRECT);
#include <AFMotor.h>
AF_DCMotor motorl(1);
void setup()
{
  Serial.begin(9600);
  //initialize the variables we're linked to
 // Input = analogRead(5);
  //Input = rpm;
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //Set motor initial speed
  //motorl.setSpeed (Output);
   //motorl.setSpeed(100);
// Speed encoder
   pinMode(encoder_pin, INPUT);
   
   //Interrupt 0 is digital pin 2, so that is where the IR detector is connected
   //Triggers on FALLING (change from HIGH to LOW)
   attachInterrupt(0, counter, FALLING);
   // Initialize
   pulses = 0;
   rpm = 0;
   timeold = 0;
}


void loop()
{
  //Speed Encoder
  if (millis() - timeold >= 1000){  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
 
  //Don't process interrupts during calculations
   detachInterrupt(0);
   //Note that this would be 60*1000/(millis() - timeold)*pulses if the interrupt
   //happened once per revolution
   rpm = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses;
   timeold = millis();
   pulses = 0;
   
   //Write it out to serial port
   Serial.print("RPM = ");
   Serial.println(rpm,DEC);
   //Restart the interrupt processing
   attachInterrupt(0, counter, FALLING);
   }
  //PID Control
 // Input = analogRead(0);
 Input = (rpm);
  myPID.Compute();
  //analogWrite(3,Output);
  // analogWrite(3,Output)
   motorl.setSpeed (Output);
   motorl.run(FORWARD);
   //pidprint();
}
void pidprint(){
   Serial.print("inputt");
   Serial.println(Input);
   Serial.print("output");
   Serial.println(Output);
}


