#include <ECE3.h>

const int left_nslp = 31;
const int left_dir = 29;
const int left_pwm = 40;
const int right_nslp = 11;
const int right_dir = 30;
const int right_pwm = 39;

uint16_t sensorValues[8];



void setup() {
  // put your setup code here, to run once:
  ECE3_Init();
  Serial.begin(9600);

  pinMode(leftnslp, OUTPUT);
  pinMode(left_dir, OUTPUT);
  pinMode(left_pwm, OUTPUT);

  pinMode(right_nslp, OUTPUT);
  pinMode(right_dir, OUTPUT);
  pinMode(right_pwm, OUTPUT);
  
  digitalWrite(left_dir, LOW);
  digitalWrite(left_nslp, HIGH);

  digitalWrite(right_dir, HIGH);
  digitalWrite(right_nslp, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  runSensor();
  
}

void runSensor(){
  ECE3_read_IR(sensorValues);
  int error1 = [-392, -1499, -2041, -1047, -1211, -750, -496, -509, -237, -252, -9, 224, 268, 379, 578, 473, 1057, 978, 2002, 1930, 459];
  int error2 = [-379, -1414, -1978, -1231, -1938, -1441, -1169, -1549, -807, -995, -40, 885, 963, 1199, 1625, 1072, 1863, 1265, 1974, 1771, 390];

  int fusion1 = sensorValue[0]*(-8) + sensorValue[1]*(-4) + sensorValue[2]*(-2) + sensorValue[3]*(-1) + sensorValue[4]*(1) + sensorValue[5]*(2) + sensorValue[6]*(4) + sensorValue[7]*(8);
  int fusion2 = sensorValue[0]*(-15) + sensorValue[1]*(-14) + sensorValue[2]*(-12) + sensorValue[3]*(-8) + sensorValue[4]*(8) + sensorValue[5]*(12) + sensorValue[6]*(14) + sensorValue[7]*(18);

  Serial.println(fusion1);
  Serial.println(fusion2);
}
