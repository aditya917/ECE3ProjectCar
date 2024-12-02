#include <ECE3.h>

const int left_nslp = 31;
const int left_dir = 29;
const int left_pwm = 40;
const int right_nslp = 11;
const int right_dir = 30;
const int right_pwm = 39;
const int LED_RF = 41;
const int bumper0 = 24;
//const int bumper1 = 25;
//const int bumper2 = 6;
//const int bumper3 = 27;
//const int bumper4 = 8;
//const int bumper5 = 28;

int store[5000][2];
int counter = 0;
uint16_t readVal[8];
int sensorValue[8];

uint16_t crosspiece[8] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
int crosspieceCount = 0;

int ignore = 0;

int speed = 25;
int oldError = 0;

void setup() {
  // put your setup code here, to run once:
  ECE3_Init();
  
  Serial.begin(9600);

  pinMode(left_nslp, OUTPUT);
  pinMode(left_dir, OUTPUT);
  pinMode(left_pwm, OUTPUT);

  pinMode(right_nslp, OUTPUT);
  pinMode(right_dir, OUTPUT);
  pinMode(right_pwm, OUTPUT);

  pinMode(bumper0, INPUT);

  motorsOn(true);
  resetEncoderCount_left();
  resetEncoderCount_right();

  pinMode(LED_RF, OUTPUT);

  delay(3000);
  
}

void loop() {
  int fusedError = runSensor();
  int changeSpeed = PIDController(fusedError, oldError);
  oldError = fusedError;
  int speedLeft = speed - changeSpeed;
  int speedRight = speed + changeSpeed;
  
  double distance = averageDistance();

  //Serial.println(distance);
  
  //automatically stop after certain amount of time, can use encoder but that's a later problem
  if(distance > 805) {
    runMotors(0, 0, LOW, LOW);
    exit(0);
  }

  // if sensor sees a crosspiece, increment; else reset
  if(memcmp(readVal, crosspiece, 8) == 0) {
    crosspieceCount++;
  } else {
    crosspieceCount = 0;
  }
  
  // if sensor reads 2 crosspieces in a row, do a half turn
  if(crosspieceCount == 2 && (distance > 350) && (distance < 450)) {
    runMotors(30, 30, HIGH, LOW);
    delay(1750);
    runMotors(0, 0, LOW, LOW);
    delay(100);

  // at this time, blindly drive forward to ignore the loop intersection
  if ((distance > 23.5 && distance < 24.5) || (distance > 65.5 && distance < 66.5) || (distance > 732.5 && distance < 733.5) || (distance > 772.5 && distance < 773.5)) {
    digitalWrite(LED_RF, HIGH);
    runMotors(speed, speed, LOW, LOW);
    delay(750);
    digitalWrite(LED_RF, LOW);
  }

  if(speedLeft < 0){
    //Serial.print("speedLeft: ");
    //Serial.println(speedLeft);
    runMotors(-speedLeft, speedRight, HIGH, LOW);
  } else if(speedRight < 0){
    //Serial.print("speedRight: ");
    //Serial.println(speedRight);
    runMotors(speedLeft, -speedRight, LOW, HIGH);
  } else{
   //Serial.println("normal operation");
  runMotors(speedLeft, speedRight, LOW, LOW);
  }
}

void storeData(int data1, int data2) {
  if(counter < 5000) {
    store[counter][0] = data1;
    store[counter][1] = data2;
    counter++;
  }
}

double averageDistance() { //in cm
  int leftDistance = getEncoderCount_left();
  int rightDistance = getEncoderCount_right();
  return (leftDistance + rightDistance)*0.03054;
}

int runSensor(){
  ECE3_read_IR(readVal);
  int minimum = 10000;
  int maximum = 0;
  for(int i = 0; i < 8; i++){
    sensorValue[i] = readVal[i];
    if(sensorValue[i] < minimum){
      minimum = sensorValue[i];
    }
    if(sensorValue[i] > maximum){
      maximum = sensorValue[i];
    }
  }
  int normalizedSensor[8];
  for(int i = 0; i < 8; i++){
    normalizedSensor[i] = ((sensorValue[i]-minimum)*1000)/maximum;
  }
  int fusion1 = normalizedSensor[0]*(-15) + normalizedSensor[1]*(-14) + normalizedSensor[2]*(-12) + normalizedSensor[3]*(-8) + normalizedSensor[4]*(8) + normalizedSensor[5]*(12) + normalizedSensor[6]*(14) + normalizedSensor[7]*(15);
  return fusion1/8;
}

void motorsOn(bool motorOn) {
  digitalWrite(left_nslp, motorOn);
  digitalWrite(right_nslp, motorOn);
}

void runMotors(int leftSpeed, int rightSpeed, bool l_dir, bool r_dir){
  digitalWrite(left_dir, l_dir);
  digitalWrite(right_dir, r_dir);

  analogWrite(left_pwm, leftSpeed);
  analogWrite(right_pwm, rightSpeed);

  //positive speed and low means go forward
  //positive speed and high means go backward
  //negative speed and low means do nothing
  //negative speed and high means do nothing 
}

int PIDController(int input, int oldInput){
  double Kp = 0.03;
  double Kd = 0.07;
  int errorP = input*Kp;
  int errorD = (input - oldInput)*Kd;
  int output = errorD + errorP;
  //Serial.println(output);
  return output;
}
