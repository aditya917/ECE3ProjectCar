#include <ECE3.h>

const int left_nslp = 31;
const int left_dir = 29;
const int left_pwm = 40;
const int right_nslp = 11;
const int right_dir = 30;
const int right_pwm = 39;

uint16_t sensorValue[8];
double rightSpeed = 30;
double leftSpeed = 30;
void setup() {
  // put your setup code here, to run once:
  ECE3_Init();
<<<<<<< HEAD
  
  Serial.begin(9600);
=======
  Serial.begin(19200);
>>>>>>> f12b105e3f43db54c2c5e85350336e79223a000e

  pinMode(left_nslp, OUTPUT);
  pinMode(left_dir, OUTPUT);
  pinMode(left_pwm, OUTPUT);

  pinMode(right_nslp, OUTPUT);
  pinMode(right_dir, OUTPUT);
  pinMode(right_pwm, OUTPUT);

  delay(3000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  double fusedError = runSensor();
  double changeSpeed = PIDController(fusedError);
  int changeLeft = leftSpeed + changeSpeed;
  int changeRight = rightSpeed - changeSpeed;
  delay(100);
  runMotors(changeLeft, changeRight);
  Serial.println(String(fusedError) + ", " + String(changeSpeed));
  
}

<<<<<<< HEAD
int runSensor(){
  ECE3_read_IR(sensorValue);
    int minimum = 10000;
    int maximum = 0;
  for(int i = 0; i < 8; i++){
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
  int fusion1 = normalizedSensor[0]*(-8) + normalizedSensor[1]*(-4) + normalizedSensor[2]*(-2) + normalizedSensor[3]*(-1) + normalizedSensor[4]*(1) + normalizedSensor[5]*(2) + normalizedSensor[6]*(4) + normalizedSensor[7]*(8);
  return fusion1;

//  
//  doublefusion2 = sensorValue[0]*(-15) + sensorValue[1]*(-14) + sensorValue[2]*(-12) + sensorValue[3]*(-8) + sensorValue[4]*(8) + sensorValue[5]*(12) + sensorValue[6]*(14) + sensorValue[7]*(18);
//
//  Serial.println(fusion1);
//  Serial.println(fusion2);
}
void runMotors(int rightSpeed, int leftSpeed){
   digitalWrite(left_dir, LOW);
   digitalWrite(right_dir, LOW);
   
   digitalWrite(left_nslp, HIGH);
   digitalWrite(right_nslp, HIGH);

  analogWrite(left_pwm, leftSpeed);
  analogWrite(right_pwm, rightSpeed);
}

double PIDController(int input){
  double Kp = 10;
  //doubleKd = 1;
  double errorP = input*Kp/1000;
  //double errorD = input*Kd
  double output = errorP;
  return output;
=======
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
>>>>>>> f12b105e3f43db54c2c5e85350336e79223a000e
}
