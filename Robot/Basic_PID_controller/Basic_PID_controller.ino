//From Lets Make Robots website
// MD03A_Motor_basic
// Test MD03a / Pololu motor

#define InA1            10                  // INA motor pin
#define InB1            11                  // INB motor pin 
#define PWM1            6                   // PWM motor pin

void setup() {
 pinMode(InA1, OUTPUT);
 pinMode(InB1, OUTPUT);
 pinMode(PWM1, OUTPUT);
}

void loop() {
 motorForward(200);                        //(25%=64; 50%=127; 100%=255)
 delay(5000);

 motorStop();
 delay(2000);

 motorBackward(200);
 delay(5000);
}

void motorForward(int PWM_val)  {
 analogWrite(PWM1, PWM_val);
 digitalWrite(InA1, LOW);
 digitalWrite(InB1, HIGH);
}

void motorBackward(int PWM_val)  {
 analogWrite(PWM1, PWM_val);
 digitalWrite(InA1, HIGH);
 digitalWrite(InB1, LOW);
}

void motorStop()  {
 analogWrite(PWM1, 0);
 digitalWrite(InA1, LOW);
 digitalWrite(InB1, LOW);
}
