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
  Serial.begin(19200);

  pinMode(left_nslp, OUTPUT);
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
  //int error1[] = [-392, -1499, -2041, -1047, -1211, -750, -496, -509, -237, -252, -9, 224, 268, 379, 578, 473, 1057, 978, 2002, 1930, 459];
  //int error2[] = [-379, -1414, -1978, -1231, -1938, -1441, -1169, -1549, -807, -995, -40, 885, 963, 1199, 1625, 1072, 1863, 1265, 1974, 1771, 390];

  int fusion1 = (sensorValues[0]*(-8) + sensorValues[1]*(-4) + sensorValues[2]*(-2) + sensorValues[3]*(-1) + sensorValues[4]*(1) + sensorValues[5]*(2) + sensorValues[6]*(4) + sensorValues[7]*(8))/4;
  int fusion2 = (sensorValues[0]*(-15) + sensorValues[1]*(-14) + sensorValues[2]*(-12) + sensorValues[3]*(-8) + sensorValues[4]*(8) + sensorValues[5]*(12) + sensorValues[6]*(14) + sensorValues[7]*(18))/8;

  Serial.println(fusion1);
  delay(1000);
  Serial.println(fusion2);
  delay(1000);
}
