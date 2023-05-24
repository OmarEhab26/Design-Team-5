#include <util/atomic.h>
#include <Ultrasonic.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 9
#define IN2 6
Ultrasonic ultrasonic2(7, 8);
Ultrasonic ultrasonic1(10, 11);
Ultrasonic ultrasonic3(12, 13);

#define LEFT_MOTOR

#ifdef LEFT_MOTOR
//Velocities
#define VMAX 79.93   //   0.22m/s=((2*pi*VMAX)/60)*32.5*10^-3 (r)
#endif

#ifdef RIGHT_MOTOR
//Velocities
#define VMAX 80   //   0.22m/s=((2*pi*VMAX)/60)*32.5*10^-3 (r)
#endif

#define VMIN 50   //  0.1 m/s

// globals
int Ultrasonic1_close = 0, Ultrasonic2_close = 0, Ultrasonic3_close = 0;
int distance1 = 0, distance2 = 0, distance3 = 0;
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile long prevT_i = 0;
float kp = 0, ki = 0, kd = 0;
float v1Filt = 0;
float v1Prev = 0;
float eintegral = 0;
float eprev = 0;
int vel = 0;
int dir = 1;
int pwr = 0;

// ROS variables
int start_stop = 0;
int camera = 0;

#ifdef LEFT_MOTOR
//voltage sensor
#define ANALOG_IN_PIN A0
float adc_voltage = 0;
float in_voltage = 0;
float R1 = 30000;
float R2 = 7500;
float ref_voltage = 5;
int adc_value = 0;
#endif

#ifdef RIGHT_MOTOR
//Buzzer pin
#define BUZZER_PIN A1
#endif

/**** ROS *****/
void message_StartStop(const std_msgs::Float64 &msg) { //function of subscriber node to get setpoint
  start_stop = msg.data;
}

void message_Camera(const std_msgs::Float64 &msg) { //function of subscriber node to get setpoint
  camera = msg.data;
}

ros::NodeHandle nh;
std_msgs::Float64 input_volt_sens;

ros::Publisher IN_VOLTAGE_Pub("battery", &input_volt_sens);  //publisher of input voltage of battery
ros::Subscriber<std_msgs::Float64> sub("Start/Stop", &message_StartStop); //subscriber

ros::Subscriber<std_msgs::Float64> sub_a("PersonFound", &message_Camera); //subscriber


/*****Setting motor direction and power****/
void setMotor(int pwmVal, int pwm, int in1, int in2) {
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

/*****Encoder readings******/

void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCA); //left motor ENCA , Right motor ENCB
  if (b < 0) {
    // If B is high, increment forward
    pos_i--;
  }
  else {
    // Otherwise, increment backward
    pos_i++;
  }
}

/****Adjusting PID*****/

float pid(float velocity1, float vt) {
  // Convert count/s to RPM
  float v1 = velocity1 / (15.0 * 31.0) * 60.0; //counts(11)*gear ratio(31)=11*31=341 _    *60 to convert from s to min

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
  /*
    Serial.print(vt);
    Serial.print(" ");
    Serial.print(v1Filt);
    Serial.println();
  */
  // Compute the control signal u
  float e = vt - v1Filt;
  return e;
}

/*****Ultrasonic function to get setpiont velocity****/

void ultra() {
  distance1 = ultrasonic1.read();
  distance2 = ultrasonic2.read();
  distance3 = ultrasonic3.read();

  if (distance1 > 30) {
    Ultrasonic1_close = 0;
  }
  else Ultrasonic1_close = 1;

  if (distance2 > 30) {
    Ultrasonic2_close = 0;
  }
  else Ultrasonic2_close = 1;

  if (distance3 > 30) {
    Ultrasonic3_close = 0;
  }
  else Ultrasonic3_close = 1;

#ifdef RIGHT_MOTOR
  if ((Ultrasonic1_close == 0 && Ultrasonic2_close == 0) || (Ultrasonic1_close == 0 && Ultrasonic3_close == 1)) {
    dir = 1;
    vel = VMAX;
  }
  else {
    dir = -1;
    vel = VMIN;
  }
#endif

#ifdef LEFT_MOTOR
  if ((Ultrasonic2_close == 0 && Ultrasonic3_close == 0) || (Ultrasonic1_close == 1 && Ultrasonic3_close == 0)) {
    dir = 1;
    vel = VMAX;
  }
  else {
    dir = -1;
    vel = VMIN;
  }
#endif
  //Certain Conditions which require the robot to move back and rotate to the right to not get stuck
  //For left and right motor
  if ((Ultrasonic3_close == 0 && Ultrasonic2_close == 1 && Ultrasonic1_close == 0) || (Ultrasonic3_close == 1 && Ultrasonic2_close == 0 && Ultrasonic1_close == 1) || (Ultrasonic3_close == 1 && Ultrasonic1_close == 1 && Ultrasonic2_close == 1)) {
    //For Moving Backwards
    for (unsigned long i = 0; i < 32000000; i++) {
      dir = -1;
      vel = VMAX;
    }
    //For Rotation
    for (unsigned long i = 0; i < 48000000; i++) {
#ifdef RIGHT_MOTOR
      dir = 1;
      vel = VMAX;
#endif
#ifdef LEFT_MOTOR
      dir = -1;
      vel = VMIN;
#endif
    }
  }
}

/*****Voltage sensor*****/

#ifdef LEFT_MOTOR
void voltage_sensor() {
  //Read Analog input
  adc_value = analogRead(ANALOG_IN_PIN);

  //Determine voltage at ADC input
  adc_voltage = (adc_value * ref_voltage) / 1024;

  //Calculate voltae at driver input
  in_voltage = adc_voltage / (R2 / (R1 + R2));
  input_volt_sens.data = in_voltage;
  IN_VOLTAGE_Pub.publish(&input_volt_sens);
}
#endif

/******Void setup****/

void setup() {
  //(nh.getHardware())->setPort(&Serial1);
  //(nh.getHardware())->setBaud(115200);

  nh.initNode();
  nh.advertise(IN_VOLTAGE_Pub);
  nh.subscribe(sub);
  nh.subscribe(sub_a);

  //Serial.begin(9600);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
#ifdef RIGHT_MOTOR
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(A5, OUTPUT);
  digitalWrite(A5, LOW);
#endif
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

/*******Main code*****/

void loop() {
  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float velocity1 = (pos - posPrev) / deltaT;
  // Serial.print(pos);
  //Serial.print("  ");
  posPrev = pos;
  prevT = currT;

  // PID values for both motors
  kp = 5; //36 , 40.5 ,0.8 ,0.18,1.1,0.8,1.5, 0.69 2
  ki = 3; //60 , 92.3 ,1.3 ,0.3,0.5,0.4,1.5, 0.33 5
  kd = 1;//                     0.2,0.6  , 0.7  0.3

  ultra();
  float e = pid(velocity1, vel);
  float dedt = (e - eprev) / deltaT;
  eintegral = eintegral + e * deltaT;

  float u = kp * e + ki * eintegral + kd * dedt;
  /*
    // Set the motor speed and direction
    if (u < 0) {
      dir = -1;
    }
  */

  if (start_stop == 4) {
    pwr = (int) fabs(u);
  }
  if (start_stop == 8 || camera == 10) {
    pwr = 0;
    eintegral = 0;
    if (camera == 10)
    {

      analogWrite(PWM, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

#ifdef RIGHT_MOTOR
      digitalWrite(BUZZER_PIN, HIGH);
#endif

      delay(5000);
      camera = 0;
#ifdef RIGHT_MOTOR
      digitalWrite(BUZZER_PIN, LOW);
#endif
    }
  }

  eprev = e;

  setMotor(pwr, PWM, IN1, IN2);
#ifdef LEFT_MOTOR
  voltage_sensor();
#endif
  /*  if(v1Filt <= VMIN+2 && v1Filt >=VMIN-2){
      eintegral = 0;
    }
  */
  nh.spinOnce();
  delay(50);
}