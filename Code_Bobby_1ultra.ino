#include <util/atomic.h>
#include <Ultrasonic.h>
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 9
#define IN2 6
Ultrasonic ultrasonic2(7, 8);
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
int distance1 = 0;
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float eintegral = 0;
float eprev = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);


  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {

  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i;
    velocity2 = velocity_i;
  }
  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float velocity1 = (pos - posPrev) / deltaT;
  // Serial.print(pos);
  //Serial.print("  ");
  posPrev = pos;
  prevT = currT;

  float v1 = velocity1 / 15 / 31 * 60.0;
  float v2 = velocity2 / 341.0 * 60.0;

  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
  v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
  v2Prev = v2;

  float vt = 80;

  float kp = 4.1;
  float ki = 0.41;
  float kd = 1;
  float e = vt - v1Filt;
  eintegral = eintegral + e * deltaT;
  float dedt = (e - eprev) / (deltaT);
  float u = kp * e + ki * eintegral + kd * dedt;

  int dir = 1;
  if (u < 0) {
    dir = 1;
  }
  distance1 = ultrasonic2.read();
  int pwr = 0.1;
  if (distance1 < 50 && distance1>= 0) {
    pwr = 0;
  }
  else pwr = (int) fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }
  setMotor(dir, pwr, PWM, IN1, IN2);
  eprev = e;

  Serial.print(vt);
  Serial.print(" ");
  // Serial.print(v2);
  // Serial.println(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(50);

  /*
    int a=digitalRead(ENCA);
    int b=digitalRead(ENCB);
    Serial.print(a*5);
    Serial.print(" ");
    Serial.print(b*5);
    Serial.println();
  */
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Motor speed
  if (dir == 1) {
    // Turn one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCA); //left motor ENCB , Right motor ENCA
  int increment = 0;
  if (b > 0) {
    // If B is high, increment forward
    increment = 1;
  }
  else {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i)) / 1.0e6;
  velocity_i = increment / deltaT;
  prevT_i = currT;
}
