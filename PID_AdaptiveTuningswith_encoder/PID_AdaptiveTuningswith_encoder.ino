/********************************************************
 * PID Adaptive Tuning Example
 * One of the benefits of the PID library is that you can
 * change the tuning parameters at any time.  this can be
 * helpful if we want the controller to be agressive at some
 * times, and conservative at others.   in the example below
 * we set the controller to use Conservative Tuning Parameters
 * when we're near setpoint and more agressive Tuning
 * Parameters when we're farther away.
 ********************************************************/
#include <AFMotor.h>
AF_DCMotor motorl(1);
int encoder_pin = 2;  // The pin the encoder is connected         
unsigned int rpm;     // rpm reading
volatile byte pulses;  // number of pulses
unsigned long timeold; 
// The number of pulses per revolution
// depends on your index disc!!
unsigned int pulsesperturn = 40;

 void counter()
 {
    //Update count
      pulses++;    
 }

#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=0.19, aggKi=0.15, aggKd=.15;
double consKp=.09, consKi=0.05, consKd=0.05;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{
  Serial.begin(9600);
    //Use statusPin to flash along with interrupts
   pinMode(encoder_pin, INPUT);
   
   //Interrupt 0 is digital pin 2, so that is where the IR detector is connected
   //Triggers on FALLING (change from HIGH to LOW)
   attachInterrupt(0, counter, FALLING);
   // Initialize
   pulses = 0;
   rpm = 0;
   timeold = 0;

  //initialize the variables we're linked to
  Input = (rpm);
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //Test startup
    motorl.setSpeed (100);
   motorl.run(FORWARD);
}

void loop()
{
  //Speed encoder
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
   //EndSpeed encoder
  Input = rpm;
  
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  myPID.Compute();
  analogWrite(3,Output);
   motorl.setSpeed (Output);
   motorl.run(FORWARD);
}


