#include <ECE3.h>

const int left_nslp = 31;
const int left_dir = 29;
const int left_pwm = 40;
const int right_nslp = 11;
const int right_dir = 30;
const int right_pwm = 39;

uint16_t sensorValue[8];
double rightSpeed = 10;
double leftSpeed = 10;
double oldError = 0;
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

  delay(3000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  double fusedError = runSensor();
  double changeSpeed = PIDController(fusedError, oldError);
  double oldError = fusedError;
  int changeLeft = leftSpeed + changeSpeed;
  int changeRight = rightSpeed - changeSpeed;

  if(changeLeft < 0){
    Serial.print("changeLeft: ");
    Serial.println(changeLeft);
    runMotors(-changeLeft, changeRight, HIGH, LOW);
  }
  else if(changeRight < 0){
    Serial.print("changeRight: ");
    Serial.println(changeRight);
    runMotors(changeLeft, -changeRight, LOW, HIGH);
  }
  //delay(100);
  else{
   //Serial.println("normal operation");
  runMotors(changeLeft, changeRight, LOW, LOW);
  }
  //Serial.println(String(fusedError) + ", " + String(changeSpeed));
  
}

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
void runMotors(int rightSpeed, int leftSpeed, bool l_dir, bool r_dir){
   digitalWrite(left_dir, l_dir);
   digitalWrite(right_dir, r_dir);
   
   digitalWrite(left_nslp, HIGH);
   digitalWrite(right_nslp, HIGH);

  analogWrite(left_pwm, leftSpeed);
  analogWrite(right_pwm, rightSpeed);

  //positive speed and low means go forward
  //positive speed and high means go backward
  //negative speed and low means do nothing
  //negative speed and high means do nothing 
}

double PIDController(int input, int oldInput){
  double Kp = 2;
  double Kd = 5;
  double errorP = input*Kp/1000;
  double errorD = (input - oldInput)*Kd/1000;
  //Serial.println(errorD);
  double output = errorD + errorP;  
  return output;
}
