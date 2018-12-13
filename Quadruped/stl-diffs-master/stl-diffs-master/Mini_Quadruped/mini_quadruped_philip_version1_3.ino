/***************************************************/
/////////////////////////////////////////////////////
///                                               ///
///    Code for printed Mini 9G Quadruped         ///
///    http://www.thingiverse.com/thing:38159     ///
///    TheCase v.1 Dec2012                        ///
///                                               ///
///    derived from Chopstick Junior 1.0          ///
///    version 1.0.6                              ///
///    by Lutz Michaelis, November 2011           ///  
///    more about this robot and contact form:    ///
///    http://letsmakerobots.com/node/29708       ///
///    powered by                                 ///
///    http://www.roboter-selbstgebaut.com        ///
///                                               ///
///                                               ///
///                                               ///
///    Updates done by Philip Booysen (zer0tilt)   ///
///    Updates done to provided ThigiVerse code    ///
///    derived from TheCase v.1                    ///
///    http://www.thingiverse.com/zerotilt/designs ///
///    http://www.thingiverse.com/make:48438       ///
///    zer0tilt v1.3 Oct 2014                       ///
///     - Added myuberkewl_leftturn                ///
///     - Added myuberkewl_rightturn               ///
///     - Added myuberkewl_forward                 ///
///     - Added myuberkewl_backward                ///
///     - Added stretch()                          ///
///     - Added wink()                             ///
///     - Adapted sweep()                          ///
///     - Adapted gym()                            ///
///     - Re-implemented NewPing in proper manner  ///
///     - Added averageDistance Sonar Ping method  ///
///     - Adpated rightturn() ... became run?!     ///
///     - Changed the intelligence in loop()       ///
///     - Everything is work in progress still     ///
///   Status:
//flatout(); //done
//stand(); //done
//sleep(); //needs more work
//standup(); //done
//laydown(); //needs more work
//wink(); //done
//gym(); //needs more work
//stretch(); //done
//tapdance(); //needs more work
//basic_testing_of_directions(); //done
//myuberkewl_rightturn(); //done
//myuberkewl_leftturn(); //done
//myuberkewl_forward(); //done
//myuberkewl_backward(); //done

/*
I was however subsequently forced to re-engineer the code for
forward walking, reverse, left turn and right turn.

See reference to myuberkewl_* functions.

Primary reason was that the provided code did not work out of the
box via thing:38159 . The original code is based on the
great work for his Chopsticks Junior robot
by Lutz Michaelis (http://letsmakerobots.com/node/29708).

I'll still post the interesting spreadsheet method
(creche method, no degree needed :) , which I devised for
motion planning and me reverse engineering the cat walk
(gait creep) on four-legged motion principles into creating
servo values for motion from this. Fancy name for this is
inverse kinematics, I just empirically produced them, will
still write a function which would make it dead simple for
any-one to re-use in Arduino Code.

The code should work if you install the servo's as mine
in the pictures and make 90 degrees (neutral) on each servo
such that the Mini Quadruped sits flat with legs stretch out
(as in the main photo in http://www.thingiverse.com/make:48438 )

 - PB (aka zer0tilt)

/////////////////////////////////////////////////////
*/

#include <NewPing.h>
#include <Servo.h>
// set the names of the servos
/*
frh = front right hip    5
 frl = front right leg   6
 flh = front left hip    7
 fll = front left leg    8
 rlh = rear left hip     9
 rll = rear left leg    10
 rrh = rear right hip   11
 rrl = rear right leg   12
 eye = sevor for sensor
 */
Servo frh;
Servo frl;
Servo flh;
Servo fll;
Servo rlh;
Servo rll;
Servo rrh;
Servo rrl;
Servo eye;

// set variables needed for some tasks

//set logical values for physical home position (cheap Chinese servos are cheap!)

int home_frh = 94;
int home_frl = 90;
int home_flh = 90;
int home_fll = 86;
int home_rlh = 90;
int home_rll = 94;
int home_rrh = 86;
int home_rrl = 86;

int b;
int x;
int w;
int up;
int up2;
int up3;
int down;
int down2;
int steps;
int rightsteps;
int leftsteps;
int back;
int pos;
int tapdo;
int sensor = A0;
int distance = 0;


#define TRIGGER_PIN  3
#define ECHO_PIN     4
#define MAX_DISTANCE 200
 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

unsigned long echo = 0;
int ultraSoundPulse = 3;
int ultraSoundEcho = 4; // Ultrasound signal pin
unsigned long ultrasoundValue = 0;

const int buttonPin = 2;     
int buttonState = 0;

const int numOfReadings = 10; // number of readings to take/ items in the array
int readings[numOfReadings]; // stores the distance readings in an array
int arrayIndex = 0; // arrayIndex of the current item in the array
int total = 0; // stores the cumlative total
int averageDistance = 0; // stores the average value

//////////////////////////////////////
void setup()
{
  pinMode(13, OUTPUT);  // LED pin
  pinMode(buttonPin, INPUT);   
  // assign servos to pins and center servos
  frh.attach(5);
  frh.write(home_frh);
  frl.attach(6);
  frl.write(home_frl);
  flh.attach(7);
  flh.write(home_flh);
  fll.attach(8);
  fll.write(home_fll);
  rlh.attach(9);
  rlh.write(home_rlh);
  rll.attach(10);
  rll.write(home_rll);
  rrh.attach(11);
  rrh.write(home_rrh);
  rrl.attach(12);
  rrl.write(home_rrl);
  //
  //eyes
  Serial.begin(9600);
  pinMode(ultraSoundPulse,OUTPUT);
  pinMode(ultraSoundEcho,INPUT);

  // create array loop to iterate over every item in the array
  for (int thisReading = 0; thisReading < numOfReadings; thisReading++) {
  readings[thisReading] = 0;
  randomSeed(millis());
  }

  flatout();
  delay(5000);
  standup();

}
//////////////////////////////////////
void idle() // this is the delay between the steps -> walking speed
{
  delay(50);  // if set to a bigger number (more delay between the steps -> slower walking) you will see the walking pattern more clearly
}
////////////////////////////////////// 
void test() /* just for debugging -> if need a delay between the subroutines 
 you can let the LED blink as an indicator that something is still running  */
{
  for(b = 0; b < 3; b++) // this let the LED blink 5 times 
  {
    digitalWrite(13, HIGH);   // turn the LED on
    delay(300);               // wait for .5 second
    digitalWrite(13, LOW);    // turn the LED off
    delay(300);
  }  
}
//////////////////////////////////////
void flatout()
{
  frh.write(home_frh);
  frl.write(home_frl);
  flh.write(home_flh);
  fll.write(home_fll);
  rlh.write(home_rlh);
  rll.write(home_rll);
  rrh.write(home_rrh);
  rrl.write(home_rrl);
}

void standup()
{
  up2 = 0;
  up3 = 0;
  frh.write(home_frh-0); //-20
  flh.write(home_flh-0); //-10
  rlh.write(home_rlh+0); //+30
  rrh.write(home_rrh-0); //-20
  for(up = 0; up < 70; up++)
  {
    frl.write(home_frl-up);
    up2 = up2 - 1;
    fll.write(home_fll-up2);
    delay(20);
  }
/*
  Serial.print("frl is home_frl-up where up is : ");
  Serial.println(up);
  Serial.print("fll is home_fll-up2 where up2 is : ");
  Serial.println(up2);
*/
  for(up = 0; up < 70; up++)
  {
    rll.write(home_rll-up);
    up3 = up3 - 1;
    rrl.write(home_rrl-up3);
    delay(15);
  }
/*
  Serial.print("rrl is home_rrl-up where up is : ");
  Serial.println(up);
  Serial.print("rll is home_rll-up2 where up3 is : ");
  Serial.println(up3);
*/
}

//////////////////////////////////////
void sleep()
{
  // hips
  frh.write(20+home_frh);
  flh.write(home_flh);
  rlh.write(20+home_rlh);
  rrh.write(home_rrh);
  // legs
  frl.write(90+home_frl);
  fll.write(90-home_fll);
  rll.write(90+home_rll);
  rrl.write(90-home_rrl);
}
//////////////////////////////////////
void stand()
{
  
  /*  For Standup, once up, the following holds true:
frl is home_frl-up where up is : 70
fll is home_fll-up2 where up2 is : -70
rrl is home_rrl-up where up is : 70
rll is home_rll-up2 where up3 is : -70
  */
  
  frh.write(home_frh);
  frl.write(home_frl-70);
  delay(20);
  flh.write(home_flh);
  fll.write(home_fll+70);
  delay(20);
  rlh.write(home_rlh);
  rll.write(home_rll-70);
  delay(20);
  rrh.write(home_rrh);
  rrl.write(home_rrl+70); 
  delay(20);
}
//////////////////////////////////////
void forward()
{
  // lift front right leg, move front right hip forward and rear right hip backward, lower front right leg
  rll.write(home_rll-50);  // lower the diagonal opposite leg a bit to keep the balance
  frl.write(home_frl-30);  
  idle();
  frh.write(home_frh+45);
  rrh.write(home_rrh-60); 
  idle();

  frl.write(home_frl-80);  //orig 170
  rll.write(home_rll-60);  //orig 170  //put the diagonal opposite leg down to keep the balance
  // lift rear left leg, move rear left hip forward and front right hip backward, lower rear left leg 
  frl.write(home_frl-80);  //orig 140 //lower the diagonal opposite leg a bit to keep the balance
  rll.write(home_rll-50); 
  idle();
  rlh.write(home_rlh-30); //orig 120
  frh.write(home_frh-20);
  idle();
  rll.write(home_rll-60);  // orig 170
  fll.write(home_fll-80);  // put the diagonal opposite leg down to keep the balance
  // lift front left leg, move front left hip forward and rear left hip backward, lower front left leg
  rrl.write(home_rrl+20);  // orig 50  //lower the diagonal opposite leg a bit to keep the balance
  fll.write(home_fll+40);
  idle();
  flh.write(home_flh-60); //orig 110
  rlh.write(home_rlh+50);//orig 60
  idle();
  fll.write(home_fll+70);
  rrl.write(home_rrl+70);  // put the diagonal opposite leg down to keep the balance
  // lift rear right leg, move rear right hip forward and front left hip backward, lower rear right leg 
  fll.write(home_fll+40);  // lower the diagonal opposite leg a bit to keep the balance 
  rrl.write(home_rrl+40); 
  idle();
  rrh.write(home_rrh+20); //orig 30
  flh.write(home_flh); //orig 50
  idle();
  rrl.write(home_rrl+50); //orig 20
  fll.write(home_fll+70);  // orig 20 //put the diagonal opposite leg down to keep the balance
  idle();

}

void myuberkewl_leftturn()
{
//Time Period 1
frl.write(40);
frh.write(81);
rrl.write(136);
rrh.write(116);
fll.write(136);
flh.write(120);
rll.write(44);
rlh.write(81);
idle();

//Time Period 2
frl.write(40);
frh.write(77);
rrl.write(136);
rrh.write(112);
fll.write(136);
flh.write(116);
rll.write(44);
rlh.write(73);
idle();

//Time Period 3
frl.write(40);
frh.write(73);
rrl.write(136);
rrh.write(107);
fll.write(136);
flh.write(111);
rll.write(44);
rlh.write(69);
idle();

//Time Period 4
frl.write(40);
frh.write(68);
rrl.write(136);
rrh.write(103);
fll.write(136);
flh.write(107);
rll.write(44);
rlh.write(64);
idle();

//Time Period 5
frl.write(40);
frh.write(64);
rrl.write(136);
rrh.write(99);
fll.write(136);
flh.write(103);
rll.write(44);
rlh.write(60);
idle();

//Time Period 6
frl.write(40);
frh.write(64);
rrl.write(136);
rrh.write(95);
fll.write(136);
flh.write(99);
rll.write(44);
rlh.write(60);
idle();

//Time Period 7
frl.write(61);
frh.write(79);
rrl.write(136);
rrh.write(90);
fll.write(136);
flh.write(94);
rll.write(65);
rlh.write(75);
idle();

//Time Period 8
frl.write(70);
frh.write(94);
rrl.write(136);
rrh.write(86);
fll.write(136);
flh.write(90);
rll.write(74);
rlh.write(90);
idle();

//Time Period 9
frl.write(61);
frh.write(109);
rrl.write(136);
rrh.write(82);
fll.write(136);
flh.write(86);
rll.write(65);
rlh.write(105);
idle();

//Time Period 10
frl.write(40);
frh.write(124);
rrl.write(136);
rrh.write(77);
fll.write(136);
flh.write(81);
rll.write(44);
rlh.write(120);
idle();

//Time Period 11
frl.write(40);
frh.write(124);
rrl.write(136);
rrh.write(73);
fll.write(136);
flh.write(77);
rll.write(44);
rlh.write(120);
idle();

//Time Period 12
frl.write(40);
frh.write(120);
rrl.write(136);
rrh.write(69);
fll.write(136);
flh.write(73);
rll.write(44);
rlh.write(116);
idle();

//Time Period 13
frl.write(40);
frh.write(115);
rrl.write(136);
rrh.write(65);
fll.write(136);
flh.write(69);
rll.write(44);
rlh.write(111);
idle();

//Time Period 14
frl.write(40);
frh.write(111);
rrl.write(136);
rrh.write(60);
fll.write(136);
flh.write(64);
rll.write(44);
rlh.write(107);
idle();

//Time Period 15
frl.write(40);
frh.write(107);
rrl.write(136);
rrh.write(56);
fll.write(136);
flh.write(60);
rll.write(44);
rlh.write(103);
idle();

//Time Period 16
frl.write(40);
frh.write(103);
rrl.write(136);
rrh.write(56);
fll.write(136);
flh.write(60);
rll.write(44);
rlh.write(99);
idle();

//Time Period 17
frl.write(40);
frh.write(98);
rrl.write(115);
rrh.write(71);
fll.write(115);
flh.write(75);
rll.write(44);
rlh.write(94);
idle();

//Time Period 18
frl.write(40);
frh.write(94);
rrl.write(106);
rrh.write(86);
fll.write(106);
flh.write(90);
rll.write(44);
rlh.write(90);
idle();

//Time Period 19
frl.write(40);
frh.write(90);
rrl.write(115);
rrh.write(101);
fll.write(115);
flh.write(105);
rll.write(44);
rlh.write(86);
idle();

//Time Period 20
frl.write(40);
frh.write(85);
rrl.write(136);
rrh.write(116);
fll.write(136);
flh.write(120);
rll.write(44);
rlh.write(81);
idle();

  
}


void myuberkewl_rightturn()
{
//Time Period 1
frl.write(40);
frh.write(107);
rrl.write(136);
rrh.write(56);
fll.write(136);
flh.write(60);
rll.write(44);
rlh.write(99);
idle();

//Time Period 2
frl.write(40);
frh.write(111);
rrl.write(136);
rrh.write(60);
fll.write(136);
flh.write(64);
rll.write(44);
rlh.write(107);
idle();

//Time Period 3
frl.write(40);
frh.write(115);
rrl.write(136);
rrh.write(65);
fll.write(136);
flh.write(69);
rll.write(44);
rlh.write(111);
idle();

//Time Period 4
frl.write(40);
frh.write(120);
rrl.write(136);
rrh.write(69);
fll.write(136);
flh.write(73);
rll.write(44);
rlh.write(116);
idle();

//Time Period 5
frl.write(40);
frh.write(124);
rrl.write(136);
rrh.write(73);
fll.write(136);
flh.write(77);
rll.write(44);
rlh.write(120);
idle();

//Time Period 6
frl.write(40);
frh.write(124);
rrl.write(136);
rrh.write(77);
fll.write(136);
flh.write(81);
rll.write(44);
rlh.write(120);
idle();

//Time Period 7
frl.write(61);
frh.write(109);
rrl.write(136);
rrh.write(82);
fll.write(136);
flh.write(86);
rll.write(65);
rlh.write(105);
idle();

//Time Period 8
frl.write(70);
frh.write(94);
rrl.write(136);
rrh.write(86);
fll.write(136);
flh.write(90);
rll.write(74);
rlh.write(90);
idle();

//Time Period 9
frl.write(61);
frh.write(79);
rrl.write(136);
rrh.write(90);
fll.write(136);
flh.write(94);
rll.write(65);
rlh.write(75);
idle();

//Time Period 10
frl.write(40);
frh.write(64);
rrl.write(136);
rrh.write(95);
fll.write(136);
flh.write(99);
rll.write(44);
rlh.write(60);
idle();

//Time Period 11
frl.write(40);
frh.write(64);
rrl.write(136);
rrh.write(99);
fll.write(136);
flh.write(103);
rll.write(44);
rlh.write(60);
idle();

//Time Period 12
frl.write(40);
frh.write(68);
rrl.write(136);
rrh.write(103);
fll.write(136);
flh.write(107);
rll.write(44);
rlh.write(64);
idle();

//Time Period 13
frl.write(40);
frh.write(73);
rrl.write(136);
rrh.write(107);
fll.write(136);
flh.write(111);
rll.write(44);
rlh.write(69);
idle();

//Time Period 14
frl.write(40);
frh.write(77);
rrl.write(136);
rrh.write(112);
fll.write(136);
flh.write(116);
rll.write(44);
rlh.write(73);
idle();

//Time Period 15
frl.write(40);
frh.write(81);
rrl.write(136);
rrh.write(116);
fll.write(136);
flh.write(120);
rll.write(44);
rlh.write(77);
idle();

//Time Period 16
frl.write(40);
frh.write(85);
rrl.write(136);
rrh.write(116);
fll.write(136);
flh.write(120);
rll.write(44);
rlh.write(81);
idle();

//Time Period 17
frl.write(40);
frh.write(90);
rrl.write(115);
rrh.write(101);
fll.write(115);
flh.write(105);
rll.write(44);
rlh.write(86);
idle();

//Time Period 18
frl.write(40);
frh.write(94);
rrl.write(106);
rrh.write(86);
fll.write(106);
flh.write(90);
rll.write(44);
rlh.write(90);
idle();

//Time Period 19
frl.write(40);
frh.write(98);
rrl.write(115);
rrh.write(71);
fll.write(115);
flh.write(75);
rll.write(44);
rlh.write(94);
idle();

//Time Period 20
frl.write(40);
frh.write(103);
rrl.write(136);
rrh.write(56);
fll.write(136);
flh.write(60);
rll.write(44);
rlh.write(99);
idle();

}

void myuberkewl_forward(){

//Time Period 1
frl.write(40);
frh.write(81);
rrl.write(136);
rrh.write(56);
fll.write(136);
flh.write(60);
rll.write(44);
rlh.write(81);
idle();

//Time Period 2
frl.write(40);
frh.write(77);
rrl.write(115);
rrh.write(101);
fll.write(136);
flh.write(64);
rll.write(44);
rlh.write(86);
idle();

//Time Period 3
frl.write(40);
frh.write(73);
rrl.write(106);
rrh.write(86);
fll.write(136);
flh.write(69);
rll.write(44);
rlh.write(90);
idle();

//Time Period 4
frl.write(40);
frh.write(68);
rrl.write(115);
rrh.write(101);
fll.write(136);
flh.write(73);
rll.write(44);
rlh.write(94);
idle();

//Time Period 5
frl.write(40);
frh.write(64);
rrl.write(136);
rrh.write(116);
fll.write(136);
flh.write(77);
rll.write(44);
rlh.write(99);
idle();

//Time Period 6
frl.write(40);
frh.write(64);
rrl.write(136);
rrh.write(116);
fll.write(136);
flh.write(81);
rll.write(44);
rlh.write(103);
idle();

//Time Period 7
frl.write(61);
frh.write(79);
rrl.write(136);
rrh.write(112);
fll.write(136);
flh.write(86);
rll.write(44);
rlh.write(107);
idle();

//Time Period 8
frl.write(70);
frh.write(94);
rrl.write(136);
rrh.write(107);
fll.write(136);
flh.write(90);
rll.write(44);
rlh.write(111);
idle();

//Time Period 9
frl.write(61);
frh.write(109);
rrl.write(136);
rrh.write(103);
fll.write(136);
flh.write(94);
rll.write(44);
rlh.write(116);
idle();

//Time Period 10
frl.write(40);
frh.write(124);
rrl.write(136);
rrh.write(99);
fll.write(136);
flh.write(99);
rll.write(44);
rlh.write(120);
idle();

//Time Period 11
frl.write(40);
frh.write(124);
rrl.write(136);
rrh.write(95);
fll.write(136);
flh.write(103);
rll.write(44);
rlh.write(120);
idle();

//Time Period 12
frl.write(40);
frh.write(120);
rrl.write(136);
rrh.write(90);
fll.write(136);
flh.write(107);
rll.write(65);
rlh.write(105);
idle();

//Time Period 13
frl.write(40);
frh.write(115);
rrl.write(136);
rrh.write(86);
fll.write(136);
flh.write(111);
rll.write(74);
rlh.write(90);
idle();

//Time Period 14
frl.write(40);
frh.write(111);
rrl.write(136);
rrh.write(82);
fll.write(136);
flh.write(116);
rll.write(65);
rlh.write(75);
idle();

//Time Period 15
frl.write(40);
frh.write(107);
rrl.write(136);
rrh.write(77);
fll.write(136);
flh.write(120);
rll.write(44);
rlh.write(60);
idle();

//Time Period 16
frl.write(40);
frh.write(103);
rrl.write(136);
rrh.write(73);
fll.write(136);
flh.write(120);
rll.write(44);
rlh.write(60);
idle();

//Time Period 17
frl.write(40);
frh.write(98);
rrl.write(136);
rrh.write(69);
fll.write(115);
flh.write(105);
rll.write(44);
rlh.write(64);
idle();

//Time Period 18
frl.write(40);
frh.write(94);
rrl.write(136);
rrh.write(65);
fll.write(106);
flh.write(90);
rll.write(44);
rlh.write(69);
idle();

//Time Period 19
frl.write(40);
frh.write(90);
rrl.write(136);
rrh.write(60);
fll.write(115);
flh.write(75);
rll.write(44);
rlh.write(73);
idle();

//Time Period 20
frl.write(40);
frh.write(85);
rrl.write(136);
rrh.write(56);
fll.write(136);
flh.write(60);
rll.write(44);
rlh.write(77);
idle(); 
  
}

void myuberkewl_backward() {

//Time Period 1
rlh.write(77);
rll.write(44);
flh.write(60);
fll.write(136);
rrh.write(56);
rrl.write(136);
frh.write(85);
frl.write(40);
idle();

//Time Period 2
rlh.write(73);
rll.write(44);
flh.write(75);
fll.write(115);
rrh.write(60);
rrl.write(136);
frh.write(90);
frl.write(40);
idle();

//Time Period 3
rlh.write(69);
rll.write(44);
flh.write(90);
fll.write(106);
rrh.write(65);
rrl.write(136);
frh.write(94);
frl.write(40);
idle();

//Time Period 4
rlh.write(64);
rll.write(44);
flh.write(105);
fll.write(115);
rrh.write(69);
rrl.write(136);
frh.write(98);
frl.write(40);
idle();

//Time Period 5
rlh.write(60);
rll.write(44);
flh.write(120);
fll.write(136);
rrh.write(73);
rrl.write(136);
frh.write(103);
frl.write(40);
idle();

//Time Period 6
rlh.write(60);
rll.write(44);
flh.write(120);
fll.write(136);
rrh.write(77);
rrl.write(136);
frh.write(107);
frl.write(40);
idle();

//Time Period 7
rlh.write(75);
rll.write(65);
flh.write(116);
fll.write(136);
rrh.write(82);
rrl.write(136);
frh.write(111);
frl.write(40);
idle();

//Time Period 8
rlh.write(90);
rll.write(74);
flh.write(111);
fll.write(136);
rrh.write(86);
rrl.write(136);
frh.write(115);
frl.write(40);
idle();

//Time Period 9
rlh.write(105);
rll.write(65);
flh.write(107);
fll.write(136);
rrh.write(90);
rrl.write(136);
frh.write(120);
frl.write(40);
idle();

//Time Period 10
rlh.write(120);
rll.write(44);
flh.write(103);
fll.write(136);
rrh.write(95);
rrl.write(136);
frh.write(124);
frl.write(40);
idle();

//Time Period 11
rlh.write(120);
rll.write(44);
flh.write(99);
fll.write(136);
rrh.write(99);
rrl.write(136);
frh.write(124);
frl.write(40);
idle();

//Time Period 12
rlh.write(116);
rll.write(44);
flh.write(94);
fll.write(136);
rrh.write(103);
rrl.write(136);
frh.write(109);
frl.write(61);
idle();

//Time Period 13
rlh.write(111);
rll.write(44);
flh.write(90);
fll.write(136);
rrh.write(107);
rrl.write(136);
frh.write(94);
frl.write(70);
idle();

//Time Period 14
rlh.write(107);
rll.write(44);
flh.write(86);
fll.write(136);
rrh.write(112);
rrl.write(136);
frh.write(79);
frl.write(61);
idle();

//Time Period 15
rlh.write(103);
rll.write(44);
flh.write(81);
fll.write(136);
rrh.write(116);
rrl.write(136);
frh.write(64);
frl.write(40);
idle();

//Time Period 16
rlh.write(99);
rll.write(44);
flh.write(77);
fll.write(136);
rrh.write(116);
rrl.write(136);
frh.write(64);
frl.write(40);
idle();

//Time Period 17
rlh.write(94);
rll.write(44);
flh.write(73);
fll.write(136);
rrh.write(101);
rrl.write(115);
frh.write(68);
frl.write(40);
idle();

//Time Period 18
rlh.write(90);
rll.write(44);
flh.write(69);
fll.write(136);
rrh.write(86);
rrl.write(106);
frh.write(73);
frl.write(40);
idle();

//Time Period 19
rlh.write(86);
rll.write(44);
flh.write(64);
fll.write(136);
rrh.write(101);
rrl.write(115);
frh.write(77);
frl.write(40);
idle();

//Time Period 20
rlh.write(81);
rll.write(44);
flh.write(60);
fll.write(136);
rrh.write(56);
rrl.write(136);
frh.write(81);
frl.write(40);
idle();
  
}

void gait_stand() {
  
  frl.write(40); 
  frh.write(94);
  rrl.write(136); //! Was 36
  rrh.write(86); 
  fll.write(136);
  flh.write(90);
  rll.write(44); //! Was 144
  rlh.write(90);

}

/////////////////////////////////////
void rightturn()
{
  
  // lift front right leg
  frl.write(70);
  idle();
  // move front right hip forward
  frh.write(124);
  // move rear right hip backward
  rrh.write(56);
  idle();
  // lower front right leg
  frl.write(40);


  // lift rear left leg
  rll.write(74); 
  idle();
  // move rear left hip forward  
  rlh.write(60);
  // front right hip backward  
  frh.write(64);
  idle();
  // lower rear left leg   
  rll.write(44);


  // lift front left leg
  fll.write(106);
  idle();
  // move front left hip forward  
  flh.write(60);
  // rear left hip backward  
  rlh.write(120);
  idle();
  // lower front left leg  
  fll.write(136);


  // lift rear right leg
  rrl.write(106); 
  idle();
  // move rear right hip forward
  rrh.write(116);
  // front left hip backward  
  flh.write(120);
  idle();
  // lower rear right leg    
  rrl.write(136);
  idle();


  /*
  // lift front right leg, move front right hip forward and rear right hip backward, lower front right leg
  frl.write(home_frl+60);
  idle();
  frh.write(home_frh-60);
  rrh.write(home_rrh);
  idle();
  frl.write(home_frl+80);
  // lift rear left leg, move rear left hip forward and front right hip backward, lower rear left leg 
  rll.write(home_rll+60); 
  idle();
  rlh.write(home_rlh+40);
  frh.write(home_frh+10);
  idle();
  rll.write(home_rll+80);
  // lift front left leg, move front left hip forward and rear left hip backward, lower front left leg
  fll.write(home_fll-50);
  idle();
  flh.write(home_flh+40);
  rlh.write(home_rlh-40);
  idle();
  fll.write(home_fll-70);
  // lift rear right leg, move rear right hip forward and front left hip backward, lower rear right leg  
  rrl.write(home_rrl-30); 
  idle();
  rrh.write(home_rrh-20);
  flh.write(home_flh-40);
  idle();
  rrl.write(home_rrl-70);
  idle();
  */
}
/////////////////////////////////////
void leftturn()
{
  // lift front right leg, move front right hip forward and rear right hip backward, lower front right leg
  frl.write(home_frl+60);
  idle();
  frh.write(home_frh-60);
  rrh.write(home_rrh+10);
  idle();
  frl.write(home_frl+80);
  // lift rear left leg, move rear left hip forward and front right hip backward, lower rear left leg 
  rll.write(home_rll+60); 
  idle();
  rlh.write(home_rlh);
  frh.write(home_frh+30);
  idle();
  rll.write(home_rll+80);
  // lift front left leg, move front left hip forward and rear left hip backward, lower front left leg
  fll.write(home_fll-30);
  idle();
  flh.write(home_flh);
  rlh.write(home_rlh-30);
  idle();
  fll.write(home_fll-70);
  // lift rear right leg, move rear right hip forward and front left hip backward, lower rear right leg  
  rrl.write(home_rrl-30); 
  idle();
  rrh.write(home_rrh-60);
  flh.write(home_flh-10);
  idle();
  rrl.write(home_rrl-70);
  idle();
}
/////////////////////////////////////
void backward()
{
  // lift front right leg, move front right hip backward and rear right hip forward, lower front right leg
  frl.write(home_frl+60);
  idle();
  frh.write(home_frh+10);
  rrh.write(home_rrh-40);
  idle();
  frl.write(home_frl+80);
  // lift rear left leg, move rear left hip backward and front right hip forward, lower rear left leg 
  rll.write(home_rll+40); 
  idle();
  rlh.write(home_rlh-30);
  frh.write(home_frh-45);
  idle();
  rll.write(home_rll+80);
  // lift front left leg, move front left hip backward and rear left hip forward, lower front left leg
  fll.write(home_fll-50);
  idle();
  flh.write(home_flh-20);
  rlh.write(home_rlh+30);
  idle();
  fll.write(home_fll-70);
  // lift rear right leg, move rear right hip backward and front left hip forward, lower rear right leg  
  rrl.write(home_rrl-30); 
  idle();
  rrh.write(home_rrh+20);
  flh.write(home_flh+20);
  idle();
  rrl.write(home_rrl-70);
  idle();
}
/////////////////////////////////////
void laydown() // lay down
{
  frh.write(home_frh-20);
  flh.write(home_flh-10); 
  for (down = 80; down > 0; down = down - 1){
    frl.write(home_frl+down);
    down2 = 100 - down;
    fll.write(down2-home_fll);
    delay(15);
  } 
  delay(1000);
  rlh.write(home_rlh-10);
  rrh.write(home_rrh-20);
  for (down = 80; down > 0; down = down - 1){
    rll.write(home_rll+down);
    down2 = 100 - down;
    rrl.write(down2-home_rrl);
    delay(15);
  }
}
/////////////////////////////////////

void mylaydown()
{
/*  For Standup, once up, the following holds true:
frl is home_frl-up where up is : 70
fll is home_fll-up2 where up2 is : -70
rrl is home_rrl-up where up is : 70
rll is home_rll-up2 where up3 is : -70
and
home_frl = 90;
home_fll = 86;
home_rll = 94;
home_rrl = 86;
so
frl=20
fll=156
rll=164
rrl=16
*/

  up2 = 0;
  up3 = 0;
  //stand();
  for(up = 70; up > 0; up--)
  {
    rll.write(home_rll+up);
    up3 = up3 + 1;
    rrl.write(home_rrl+up3);
    delay(15);
  }

  for(up = 70; up > 0; up--)
  {
    frl.write(home_frl+up);
    up2 = up2 + 1;
    fll.write(home_fll+up2);
    delay(20);
  }
  //flatout();
}
/////////////////////////////////////

void gym() // ok, this is not very serious but I needed to cheer me up a bit ;-) just see...
{
  int y;
  frh.write(home_frh+20);
  rll.write(home_rll-40);
  delay(200);
  fll.write(home_fll+50);
  flh.write(home_flh-20);
  fll.write(home_fll+70);
  delay(20);
  frl.write(home_frl+30);
  delay(20);
  rlh.write(home_rlh-30);
  delay(20);
  rrh.write(home_rrh+20);
  rrl.write(home_rrl+70); 
  delay(20); 

/*  
  delay(800);
  rrl.write(home_rrl-70); 
  rrh.write(home_rrh);
  delay(20);
  rlh.write(home_rlh);
  delay(20);
  frl.write(home_frl-70);
  delay(20);
  fll.write(home_fll+70);
  flh.write(home_flh);
  delay(200);
  rll.write(home_rll-70);
  frh.write(home_frh);

  delay(800);
*/

}
/////////////////////////////////////
void wink()
{
  for (b = 0; b < 4; b++){
    for (w = 20; w < -20; w = w -1)
    {
      frl.write(home_frl+w);
      delay(10);
    }
    for (w = -20; w < 20; w++)
    {
      frl.write(home_frl+w);
      delay(10);
    }
    delay(200);
  }
  frl.write(home_frl-70);
}
/////////////////////////////////////
void sweep()
{  
  for(pos = 20; pos < 160; pos += 1)   // goes from 0 degrees to 180 degrees 
  {                                    // in steps of 1 degree 
    eye.write(pos);                    // tell servo to go to position in variable 'pos' 
    delay(10);                         // waits 15ms for the servo to reach the position 
    distance = analogRead(sensor);
    //Serial.print(distance);
    //Serial.print(" - Position ");
    //Serial.print(pos);
    //Serial.println();
  } 
  for(pos = 160; pos >= 20; pos -= 1)      // goes from 180 degrees to 0 degrees 
  {                                
    eye.write(pos);                    // tell servo to go to position in variable 'pos' 
    delay(10);                         // waits 15ms for the servo to reach the position 
    distance = analogRead(sensor);
    //Serial.print(distance);
    //Serial.print(" - Position ");
    //Serial.print(pos);  
    //Serial.println();
  }       
}

/////////////////////////////////////
void tapdance()
{

  for(tapdo = 1; tapdo <= 4; tapdo += 1)      // goes from 1 to 4 times tapdance 
  {                                
    frl.write(home_frl+70);
    delay(400);
    frl.write(home_frl-70);

    delay(500);
  
    fll.write(home_frl-70);
    delay(400);
    fll.write(home_frl+70);
    
    delay(500);
  }

  //rll.write(home_frl+70);
  //delay(400);
  //rll.write(home_frl-70);

  //rrl.write(home_frl-70);
  //delay(400);
  //rrl.write(home_frl+70);

}
/////////////////////////////////////
void stretch() {
  // hips
  frh.write(home_frh-15);
  flh.write(home_flh+15);
  rlh.write(home_rlh-15);
  rrh.write(home_rrh+5);
  // legs
  frl.write(home_frl);
  fll.write(home_fll);
  rll.write(home_rll);
  rrl.write(home_rrl);

  delay(1000);

  // hips
  frh.write(home_frh-15+90);
  flh.write(home_flh+15-90);
  rlh.write(home_rlh-15+90);
  rrh.write(home_rrh+5-90);
  // legs
  frl.write(home_frl);
  fll.write(home_fll);
  rll.write(home_rll);
  rrl.write(home_rrl);

  delay(1000);

  // hips
  frh.write(home_frh-15+90);
  flh.write(home_flh+15-90);
  rlh.write(home_rlh-15+90);
  rrh.write(home_rrh+5-90);
  // legs
  frl.write(home_frl+90);
  fll.write(home_fll-90);
  rll.write(home_rll+90);
  rrl.write(home_rrl-90);

  delay(1000);

  // hips
  frh.write(home_frh-15);
  flh.write(home_flh+15);
  rlh.write(home_rlh-15);
  rrh.write(home_rrh+5);
  // legs
  frl.write(home_frl+90);
  fll.write(home_fll-90);
  rll.write(home_rll+90);
  rrl.write(home_rrl-90);

  delay(1000);

}


/////////////////////////////////////

void basic_testing_of_directions() {
  rll.write(home_rll-10);
  delay(1000);
  rll.write(home_rll);
  delay(1000);
  rlh.write(home_rlh-10);
  delay(1000);
  rlh.write(home_rlh);
  delay(1000);

  fll.write(home_fll+10);
  delay(1000);
  fll.write(home_fll);
  delay(1000);
  flh.write(home_flh-10);
  delay(1000);
  flh.write(home_flh);
  delay(1000);

  rrl.write(home_rrl+10);
  delay(1000);
  rrl.write(home_rrl);
  delay(1000);
  rrh.write(home_rrh+10);
  delay(1000);
  rrh.write(home_rrh);
  delay(1000);

  frl.write(home_frl-10);
  delay(1000);
  frl.write(home_rll);
  delay(1000);
  frh.write(home_frh+10);
  delay(1000);
  frh.write(home_frh);
  delay(1000);
}

/////////////////////////////////////
unsigned long ping(){
  // Switch signalpin to output
  digitalWrite(ultraSoundPulse, LOW); // Send low pulse
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(ultraSoundPulse, HIGH); // Send high pulse
  delayMicroseconds(5); // Wait for 5 microseconds
  digitalWrite(ultraSoundPulse, LOW); // Holdoff
  // Switch signalpin to input
  digitalWrite(ultraSoundPulse, HIGH); // Turn on pullup resistor
  echo = pulseIn(ultraSoundEcho, HIGH); //Listen for echo
  ultrasoundValue = (echo / 58.138) * .39; //convert to CM then to inches
  return ultrasoundValue;
}
////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////
unsigned long myping(){
  int uS = sonar.ping();
  ultrasoundValue=(uS / US_ROUNDTRIP_CM);
  //Serial.print("Ping: ");
  //Serial.print(uS / US_ROUNDTRIP_CM);
  //Serial.println("cm"); 
  return ultrasoundValue;
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void loop()
{
  //distance = analogRead(sensor);
  buttonState = digitalRead(buttonPin);

  back=0;
  while(back < numOfReadings){        // just a loop for 5 steps backwards since he run into an obstacle
    distance = myping();
    //Serial.print("Distance: ");
    //Serial.println(distance);
    total= total - readings[arrayIndex]; // subtract the last distance
    readings[arrayIndex] = distance; // add distance reading to array
    total= total + readings[arrayIndex]; // add the reading to the total
    arrayIndex = arrayIndex + 1; // go to the next item in the array
    // At the end of the array (10 items) then start again
    if (arrayIndex >= numOfReadings) {
    arrayIndex = 0;
    }
  
    back++;
  }

  //Serial.print("Average Distance: ");
  //Serial.println(averageDistance);  
  averageDistance = total / numOfReadings; // calculate the average distance

  if (averageDistance < 1)  // Aha! Endless space infront of us, set it sufficiently large
  {
    //Serial.println("Average Distance is 0 (ZERO) setting to 50 (FIFTY)");
    averageDistance = 50 ;
  }
  
  
  if (averageDistance < 3)  // Aha! Do some tricks, we put our hand infront of it's eyes
  {
    //Serial.print("Average Distance less than 3 (THREE) ");

    test();

    flatout();
    sleep();
    flatout();
    gym();
    flatout();
    standup();
    wink();
    stand();
    stretch();

  }  
  else if (averageDistance < 12)  // oh, there is an obstacle in my path need to reverse and then turn right to avoid a crash
  {   
    //Serial.print("Average Distance less than 12 (TWELVE) ");

    test();
    
    stand();
    delay(1000);
    wink();
    delay(1000);
    back = 0;
    while(back < 3){        // just a loop for 3 steps backwards since he run into an obstacle
      myuberkewl_backward();
      back++;
    }
    leftsteps = 0;
    while(leftsteps < 3)        // just a loop for 2 steps left turn since he run into an obstacle
    {
      myuberkewl_rightturn();
      leftsteps++;
    }
  
  }
  else if (averageDistance < 18)  // oh, there is an obstacle in my path but no need to panic it's still not in the danger zone, just steer to the left to avoid a crash
  {
    //Serial.print("Average Distance less than 30 (THIRTY) ");
    test();

    back = 0;
    leftsteps = 0;
    while(leftsteps < 2)        // just a loop for 2 steps left turn since he run into an obstacle
    {
      myuberkewl_leftturn();
      leftsteps++;
    }

  }
  else
  {
  //Serial.println("Walking Forward");
  myuberkewl_forward(); 
  }

//delay(100); // Only for when debugging to not over thunder the serial port on prints

/* debug area 

##############
*/

   //flatout(); //done
   //stand(); //done
   //sleep(); //needs more work
   //standup(); //done
   //laydown(); //needs more work
   //wink(); //done
   //gym(); //needs more work
   //stretch(); //done
   //tapdance(); //needs more work
   //basic_testing_of_directions(); //done
   //myuberkewl_rightturn(); //done
   //myuberkewl_leftturn(); //done
   //myuberkewl_forward(); //done
   //myuberkewl_backward(); //done

/*
  back=0;
  while(back < 10){        // just a loop for 5 steps backwards since he run into an obstacle
  myuberkewl_forward();
  back++;
  }

  back=0;
  while(back < 2){        // just a loop for 5 steps backwards since he run into an obstacle
  myuberkewl_rightturn();
  back++;
  }

  back=0;
  while(back < 10){        // just a loop for 5 steps backwards since he run into an obstacle
  myuberkewl_backward();
  back++;
  }

  back=0;
  while(back < 2){        // just a loop for 5 steps backwards since he run into an obstacle
  myuberkewl_leftturn();
  back++;
  }

  delay(5000);

##############

  if  (buttonState == HIGH){
    flatout();
  //}else if (distance > 2){
    //backward();
   // flatout();
  }else{ 
    forward();
  }
*/

}

