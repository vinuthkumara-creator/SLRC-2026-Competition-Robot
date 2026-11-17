#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>
MPU6050 mpu;
#include <QTRSensors.h>

QTRSensors qtr;
#define EEPROM_FLAG 0
#define EEPROM_RSP 4
#define EEPROM_LSP 8
// The goal for readLine (center)
#define KP 0.026
#define KD 0.29
// The D value in PID
#define MAX_SPEED 148  // The max speed to set motors to
#define SET_SPEED 150  // The goal speed to set motors to
#define MIN_SPEED 0    // The min speed to set motors to
#define NUM_SENSORS 5  // The number of QTR sensors
#define ENA 6
#define IN1 32
#define IN2 33
#define ENB 7
#define IN3 35
#define IN4 34


#define WHITE 1
#define BLACK 0

int32_t SETPOINT = 3500;
int32_t lineCenter = 3500;


// PID **************************************
int lastError = 0;  // For storing PID error
const uint8_t SensorCount = 8;
const uint8_t sensorPins[SensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7};

uint16_t rawValues[SensorCount];
uint16_t minVal[SensorCount];
uint16_t maxVal[SensorCount];
uint16_t scaledValues[SensorCount];


unsigned long lastTime;
int globallsp = 88;
int globalrsp = 88;

void setup() {
  Serial.begin(115200);
  pinMode(8, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  delay(2000);
  digitalWrite(8, HIGH);


  // button pressed
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(1000);
  // ESP32 ADC: 0–4095

  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);

  calibrate();
  digitalWrite(8, LOW);
  delay(4000);

void loop() {
  if (readQTRAndCheck()){
    delay(50);
    if (digitalRead(FS)==1){
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      delay(900);
      analogWrite(ENA, 90);
      analogWrite(ENB, 88);
      delay(650);
      }
    else {
      turn_right();
    }

    }
  
  if (readQTRAndCheckwhite()){
    analogWrite(ENA, 90);
    analogWrite(ENB, 88);
    delay(750);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(2000);
    
    while ((digitalRead(FS) != 1)){
        analogWrite(ENA, 100);
        analogWrite(ENB, 100);
      }
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      delay(600);
       }

  else {
    if (detectLeftJunction()) {
       turn_left();

    }

    if (detectRightJunction()){
      turn_right();

    }

    if (digitalRead(FS)==1 && digitalRead(LS)==1 && digitalRead(RS)==0){
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      delay(500);
      analogWrite(ENA, 92);
      analogWrite(ENB, 88);
      delay(750);
       }

    if (digitalRead(FS)==1 && digitalRead(RS)==1 && digitalRead(LS)==0){
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      delay(500);
      analogWrite(ENA, 92);
      analogWrite(ENB, 88);
      delay(750);
       }

    else {
       goforward();
  }
  } 
}
/*
void calibrateGyro() {
  Serial.println("Calibrating gyro... Keep robot still");
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(3);
  }

  gyroZoffset = sum / 1000.0;
  Serial.println("Calibration done");
  delay(1000);
}

void maintainStraight() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // Convert raw gyro to deg/sec (131 LSB = 1 deg/sec)
  float gyroZ = (gz - gyroZoffset) / 131.0;

  // Integrate to get yaw angle
  yaw += gyroZ * dt;

  // PID correction (P-only)
  float correction = Kp1 * yaw;

  int rightSpeed  = baseSpeed - correction;
  int leftSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed,  0, 140);
  rightSpeed = constrain(rightSpeed, 0, 140);

  globallsp = leftSpeed;
  globalrsp = rightSpeed;

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // Debug
  //Serial.print("Yaw: ");
  //Serial.print(yaw);
  //Serial.print("  L: ");
  //Serial.print(leftSpeed);
  //Serial.print("  R: ");
  //Serial.println(rightSpeed);
}

void getspeed(){

  for (int i = 0; i < 60; i++){

    if(Wire.getWireTimeoutFlag())
{
    Wire.clearWireTimeoutFlag();
    Wire.end();
    Wire.begin();
    // put the code to reinit MPU6050 
}
  delay(200);
  maintainStraight();
  Serial.print("Left Speed: ");
  Serial.print(globallsp);
  Serial.print(" Right Speed: ");
  Serial.println(globalrsp);
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

}

void saveCalibration() {
  EEPROM.put(EEPROM_RSP, globalrsp);
  EEPROM.put(EEPROM_LSP, globallsp);
  EEPROM.write(EEPROM_FLAG, 0xAA);  // 0xAA means "calibration valid"
}

void loadCalibration() {
  EEPROM.get(EEPROM_RSP, globalrsp);
  EEPROM.get(EEPROM_LSP, globallsp);
  
}
*/
void calibrate() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    minVal[i] = 4095;
    maxVal[i] = 0;
  }

  Serial.println("Manual calibration started");
  Serial.println("Move sensors over BLACK and WHITE");
  delay(3000);

  // Manual calibration loop
  unsigned long startTime = millis();
  while (millis() - startTime < 8000)  // 4 seconds
  {
    qtr.read(rawValues);

    for (uint8_t i = 0; i < SensorCount; i++) {
      if (rawValues[i] < minVal[i]) minVal[i] = rawValues[i];
      if (rawValues[i] > maxVal[i]) maxVal[i] = rawValues[i];
    }
    delay(5);
  }
}

int32_t getLinePositionAnalog() {
  qtr.read(rawValues);

  int64_t weightedSum = 0;
  int32_t sum = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    int32_t value;

    if (maxVal[i] > minVal[i])
      value = (rawValues[i] - minVal[i]) * 1000L / (maxVal[i] - minVal[i]);
    else
      value = 0;

    value = constrain(value, 0, 1000);

    weightedSum += (int64_t)value * (i * 1000);
    sum += value;
  }
  Serial.println();

  if (sum == 0)
    return -1;  // line lost

  // 0 – 7000, center ≈ 3500
  return weightedSum / sum;
}

bool detectLeftJunction() {
  int L = digitalRead(LS);
  int R = digitalRead(RS);
  int F = digitalRead(FS);

  // Left junction condition
  if (L == 1 && R == 0 && F == 0) {
    return true;
  }

  return false;
}

void goforward() {
  unsigned int linePos = getLinePositionAnalog();


  // Compute the error
  int error = SETPOINT - linePos;

  // Compute the motor adjustment
  int adjust = error * KP + KD * (error - lastError);

  // Record the current error for the next iteration
  lastError = error;

  // Adjust motors, one negatively and one positively
  analogWrite(ENA, constrain(globallsp - adjust, MIN_SPEED, MAX_SPEED));
  analogWrite(ENB, constrain(globalrsp + adjust, MIN_SPEED, MAX_SPEED));
}

void turn_left() {
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      delay(2000);

     /* if (digitalRead(FS == 1)){
        analogWrite(ENA, 0);
        analogWrite(ENB, 108);
        delay(1000);
      }*/

     
      while ((digitalRead(FS) != 1)){
        analogWrite(ENA, 0);
        analogWrite(ENB, 108);
      }
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      delay(600);
 
}

void go_straight() {
  analogWrite(ENA, 90);
  analogWrite(ENB, 90);
}

bool detectTJunction() {
  int L = digitalRead(LS);
  int F = digitalRead(FS);
  int R = digitalRead(RS);


  // Left junction condition
  if (L == 1 && F == 0 && R == 1) {
    return true;
  }

  return false;
}

bool detectXJunction() {
  int L = digitalRead(LS);
  int F = digitalRead(FS);
  int R = digitalRead(RS);
  

  // Left junction condition
  if (L == 1 && F == 1 && R == 1) {
    return true;
  }

  return false;
}

void turn_right() {
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      delay(2000);

      /*if (digitalRead(FS == 1)){
        analogWrite(ENA, 110);
        analogWrite(ENB, 0);
        delay(1000);
      }*/

     
      while ((digitalRead(FS) != 1)){
        analogWrite(ENA, 110);
        analogWrite(ENB, 0);
      }
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      delay(600);
 
}

bool readQTRAndCheck()

{ 
  qtr.read(rawValues);

  int64_t weightedSum = 0;
  int32_t sum = 0;
  bool allAbove900 = true;

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    int32_t value;

    if (maxVal[i] > minVal[i])
      value = (rawValues[i] - minVal[i]) * 1000L /
              (maxVal[i] - minVal[i]);
    else
      value = 0;

    value = constrain(value, 0, 1000);

    weightedSum += (int64_t)value * (i * 1000);
    sum += value;

    if (value <= 900)
      allAbove900 = false;
  }

  return allAbove900;
}
bool detectRightJunction() {
  int L = digitalRead(LS);
  int R = digitalRead(RS);
  int F = digitalRead(FS);

  // Left junction condition
  if (L == 0 && R == 1 && F == 0) {
    return true;
  }

  return false;
}

bool readQTRAndCheckwhite()

{ 
  qtr.read(rawValues);

  int64_t weightedSum = 0;
  int32_t sum = 0;
  bool allbelow900 = true;

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    int32_t value;

    if (maxVal[i] > minVal[i])
      value = (rawValues[i] - minVal[i]) * 1000L /
              (maxVal[i] - minVal[i]);
    else
      value = 0;

    value = constrain(value, 0, 1000);

    weightedSum += (int64_t)value * (i * 1000);
    sum += value;

    if (value > 900)
      allbelow900 = false;
  }

  return allbelow900;
}