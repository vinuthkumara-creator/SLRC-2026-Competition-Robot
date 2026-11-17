#include <EEPROM.h>
#include <QTRSensors.h>
QTRSensors qtr;
float distance_cm = 0.0;
int x_offset = 0;

int frontdis = 13;
#define CALIB_SWITCH 10

#define ENA   6
#define IN1   32
#define IN2   33
#define ENC_L 2
#define led 29

// Left motor
#define ENB   7
#define IN3   35
#define IN4   34
#define ENC_R 3

#define EEPROM_FLAG 0   // to check if data exists
#define EEPROM_MIN  10
#define EEPROM_MAX  50

#define MAX_SPEED 255  // The max speed to set motors to  // The goal speed to set motors to
#define MIN_SPEED 0    // The min speed to set motors to
#define NUM_SENSORS 8 

#define KP 0.228
#define KD 0.33

#define leftend 46
#define beforeleft 47
#define rightend 49
#define beforeright 48
#define middle1 51
#define middle2 52

int left;
int right;
int befleft;
int befright;
bool whitetaken = false;
bool boxdetected = false; 

const int trigPin1 = 30; // Left Side Sensor
const int echoPin1 = 29;
const int trigPin2 = 36; // Right Side Sensor
const int echoPin2 = 37;

// Front-facing sensor
const int frontTrig = 41;
const int frontEcho = 40;

const float TARGET_DISTANCE_CM = 39.0;
const float DISTANCE_TOLERANCE_CM = 2.0; 
const int REQUIRED_CONSECUTIVE_READS = 3;

int box1Detections = 0; // Left tracking
int box2Detections = 0; // Right tracking

int32_t SETPOINT = 3500;

// PID **************************************
int lastError = 0;  // For storing PID error
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

uint16_t rawValues[SensorCount];
uint16_t minVal[SensorCount];
uint16_t maxVal[SensorCount];
uint16_t scaledValues[SensorCount];

unsigned long lastTime;
int globallsp = 150;
int globalrsp = 150;

// Robot parameters (in mm)
const float WHEEL_DIAMETER_MM   = 65.0;
const float WHEELBASE_MM        = 190.0;

// for angle tuning:
//   robot turns MORE than asked  → decrease this value
//   robot turns LESS than asked  → increase this value
const float WHEELBASE_EFFECTIVE = 190.0;

const int   GEAR_RATIO          = 45;
const int   ENCODER_PPR         = 11;
const int   CPR                 = ENCODER_PPR * GEAR_RATIO * 2; // 990 (CHANGE mode)
const float WHEEL_CIRC_MM       = PI * WHEEL_DIAMETER_MM;       // ~204.2 mm

// Speed zones in PWM values (must stay in order: FULL > rotate > SLOW > CRAWL > MOTOR_MIN)
const int SPEED_FULL   = 150;
const int SPEED_rotate =  80;
const int SPEED_SLOW   =  110;  // just below rotate

const int SPEED_CRAWL  =  90;  // just above motor stall
const int MOTOR_MIN    =  85;  // absolute minimum — below this motor stalls

// Remaining pulse thresholds (for speed zone switching)
const int SLOW_THRESH  = 80;   // pulses remaining → switch to SLOW
const int CRAWL_THRESH = 25;   // pulses remaining → switch to CRAWL

// PID tuning
// Kp — main correction force (too high = zigzag, too low = drift)
// Ki — fixes persistent motor imbalance over time
// Kd — dampens overcorrection and oscillation
const float Kp1      = 2.3;
const float Ki1      = 0.01;
const float Kd1      = 1.2;
const int   PID_MAX = 50;

// Encoder counters
volatile long leftPulses  = 0;
volatile long rightPulses = 0;

void countLeft()  { leftPulses++;  }
void countRight() { rightPulses++; }

// PID state variables
float pidIntegral  = 0;
float pidLastError = 0;

void resetPID() {
  pidIntegral  = 0;
  pidLastError = 0;
}

// PID sync correction for motors
// +ve result → left wheel too fast → slow left, speed right
// -ve result → right wheel too fast → speed left, slow right
int computePID(long lp, long rp) {
  float error      = (float)(lp - rp);
  pidIntegral     += error;
  float derivative = error - pidLastError;
  pidLastError     = error;

  // clamp integral to prevent windup
  pidIntegral = constrain(pidIntegral, -200, 200);

  float output = (Kp1 * error) + (Ki1 * pidIntegral) + (Kd1 * derivative);
  return (int)constrain(output, -PID_MAX, PID_MAX);
}

void applyPID(int baseSpeed, long lp, long rp) {
  int correction = computePID(lp, rp);
  // clamp to MOTOR_MIN so PID never pushes a wheel into stall
  int leftSpd    = constrain(baseSpeed - correction, MOTOR_MIN, 255);
  int rightSpd   = constrain(baseSpeed + correction, MOTOR_MIN, 255);
  analogWrite(ENA, leftSpd);
  analogWrite(ENB, rightSpd);
}

void driveStraight(int speed) {
  leftPulses  = 0;
  rightPulses = 0;
  resetPID();

  // both wheels forward
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);

  // keep running until stopMotors() is called externally
  while (true) {
    applyPID(speed, leftPulses, rightPulses);
  }
}

void rotate(float angle, int speed = SPEED_FULL) {
  if (angle == 0) return;

  float arcMM        = (abs(angle) / 360.0) * PI * WHEELBASE_EFFECTIVE;
  long  pulsesNeeded = (long)((arcMM / WHEEL_CIRC_MM) * CPR);

  leftPulses  = 0;
  rightPulses = 0;

  bool leftFwd  = (angle > 0);
  bool rightFwd = (angle < 0);

  // ? - short form for condition ? value_if_true : value_if_false
  digitalWrite(IN1, leftFwd  ? HIGH : LOW);
  digitalWrite(IN2, leftFwd  ? LOW  : HIGH);
  digitalWrite(IN3, rightFwd ? HIGH : LOW);
  digitalWrite(IN4, rightFwd ? LOW  : HIGH);

  while (true) {
    long lp        = leftPulses;
    long rp        = rightPulses;
    long avg       = (lp + rp) / 2;
    long remaining = pulsesNeeded - avg;

    if (remaining <= 0) break;

    // ── Speed zone ──────────────────────────────────
    int spd = speed;
    if      (remaining < CRAWL_THRESH) spd = SPEED_CRAWL;
    else if (remaining < SLOW_THRESH)  spd = SPEED_SLOW;

    // ── Sync correction ─────────────────────────────
    // If one wheel is ahead, slow it down slightly
    int leftSpd  = spd;
    int rightSpd = spd;
    long diff = lp - rp;
    if      (diff >  3) leftSpd  = max(spd - 20, SPEED_CRAWL);
    else if (diff < -3) rightSpd = max(spd - 20, SPEED_CRAWL);

    analogWrite(ENA, leftSpd);
    analogWrite(ENB, rightSpd);
  }

  stopMotors();
}

// Stop with active brake
void stopMotors() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  delay(30); // do not increase this
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}


// Drive forward and backward in cm
// distanceCM : +ve = forward,  -ve = backward
// speed      : PWM 0-255
void driveCM(float distanceCM, int speed = SPEED_FULL) {
  if (distanceCM == 0) return;

  float distanceMM   = abs(distanceCM) * 10.0;
  long  pulsesNeeded = (long)((distanceMM / WHEEL_CIRC_MM) * CPR);

  leftPulses  = 0;
  rightPulses = 0;
  resetPID();

  bool goForward = (distanceCM > 0);

  if (goForward) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } 
  else {
    digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);  digitalWrite(IN4, LOW);
  }

  while (true) {
    long lp        = leftPulses;
    long rp        = rightPulses;
    long avg       = (lp + rp) / 2;
    long remaining = pulsesNeeded - avg;

    if (remaining <= 0) break;

    // Speed zone — slows down near target to reduce overshoot
    int spd = speed;
    if      (remaining < CRAWL_THRESH) spd = SPEED_CRAWL;
    else if (remaining < SLOW_THRESH)  spd = SPEED_SLOW;

    // PID keeps both wheels in sync → drives straight
    applyPID(spd, lp, rp);
  }

  stopMotors();
}

void calibrate(){
  digitalWrite(led, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    delay(20);
  }

  EEPROM.write(EEPROM_FLAG, 1);

  // Save min & max
  for (int i = 0; i < NUM_SENSORS; i++) {
    EEPROM.put(EEPROM_MIN + i * 2, qtr.calibrationOn.minimum[i]);
    EEPROM.put(EEPROM_MAX + i * 2, qtr.calibrationOn.maximum[i]);
  }
  delay(1000);
  digitalWrite(led, LOW); 
}

void stop(){
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void start(){
  while (whitetaken != true){
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);

    analogWrite(ENA, 140);
    analogWrite(ENB, 180);
    if (digitalRead(middle1)==0 || digitalRead(middle2)==0){
      whitetaken=true;
    }
    delay(5);
    whitetaken=false;
  }
  driveCM(12, 140);
  delay(1000);

  rotate(-90, 180);
  delay(1000);
}

long readUltrasonic(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;

  return (duration * 0.0343) / 2;
}

void collectRbox(){
  rotate(90, 180);
  while (readUltrasonic(41, 40) >=16){
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, 140);
    analogWrite(ENB, 180);
  }

  //grab fucntion
  rotate(360, 180);
  while (whitetaken != true){
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);

    analogWrite(ENA, 140);
    analogWrite(ENB, 180);
    if (digitalRead(middle1)==0 || digitalRead(middle2)==0){
      whitetaken=true;
    }
    delay(5);
    whitetaken=false;
  }
  driveCM(12, 140);
  delay(1000);
  rotate(90,180);
  delay(1000);

}

void collectLbox(){
  rotate(-90, 180);
  while (readUltrasonic(41, 40) >=16){
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, 140);
    analogWrite(ENB, 180);
  }

  //grab fucntion
  rotate(360, 180);
  while (whitetaken != true){
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);

    analogWrite(ENA, 140);
    analogWrite(ENB, 180);
    if (digitalRead(middle1)==0 || digitalRead(middle2)==0){
      whitetaken=true;
    }
    delay(5);
    whitetaken=false;
  }
  driveCM(12, 140);
  delay(1000);
  rotate(-90,180);
  delay(1000);

}

bool isBoxDetected(int tPin, int ePin, int &consecutiveDetections) {
  long distance = readUltrasonic(tPin, ePin);

  if (distance > 0 && abs(distance - TARGET_DISTANCE_CM) <= DISTANCE_TOLERANCE_CM) {
    consecutiveDetections++;
    if (consecutiveDetections >= REQUIRED_CONSECUTIVE_READS) {
      consecutiveDetections = 0; 
      return true; 
    }
  } else {
    consecutiveDetections = 0; 
  }
  return false; 
}

void setup() {
  Serial.begin(115200); 
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(leftend,INPUT);
  pinMode(beforeleft,INPUT);
  pinMode(rightend,INPUT);
  pinMode(beforeright,INPUT);
  pinMode(middle1,INPUT);
  pinMode(middle2,INPUT);
  pinMode(led,OUTPUT);

  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  digitalWrite(led, LOW);

  pinMode(CALIB_SWITCH, INPUT_PULLUP);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  // CHANGE — counts both rising and falling edges → 2x resolution
  attachInterrupt(digitalPinToInterrupt(ENC_L), countLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R), countRight, CHANGE);

  stopMotors();

  if (digitalRead(CALIB_SWITCH) == LOW) {  // switch ON
    calibrate();
    }
  else{
    qtr.calibrate();

    
    for (int i = 0; i < NUM_SENSORS; i++) {
      EEPROM.get(EEPROM_MIN + i * 2, qtr.calibrationOn.minimum[i]);
      EEPROM.get(EEPROM_MAX + i * 2, qtr.calibrationOn.maximum[i]);
  }
  }
  driveCM(4,140);
  start();
  }


void loop() {
  float distance_cm = 0.0;
int x_offset = 0;

void setup() {
  Serial.begin(115200); 
}

void loop() {

  if (Serial.available() > 0) {
    distance_cm = Serial.parseFloat();
    x_offset = Serial.parseInt();
    
    if (Serial.read() == '\n') {
      // --- ECHO BACK TO THE PI ---
      Serial.print("Mega Confirms -> Dist: ");
      Serial.print(distance_cm);
      Serial.print(" | X-Off: ");
      Serial.println(x_offset);
      analogWrite(ENA,0);
      analogWrite(ENB,0);

    }
  }
}
  right = digitalRead(rightend);
  left = digitalRead(leftend);
  befright = digitalRead(beforeright);
  befleft = digitalRead(beforeleft);

  bool detectRightBox = isBoxDetected(trigPin2, echoPin2, box2Detections);
  bool detectLeftBox  = isBoxDetected(trigPin1, echoPin1, box1Detections);
  
  if (right==0){
    driveCM(frontdis);
    delay(600);
    rotate(90,180);
    delay(600);
     }


  else if (left==0){
    driveCM(frontdis);
    delay(600);
    rotate(-90,180);
    delay(600);
     }

  if (detectRightBox){
    collectRbox();

  }

  if (detectLeftBox){
    collectLbox();

  }

  goforward();
  }


void goforward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  

  uint16_t position = qtr.readLineWhite(sensorValues);
  // Compute the error
  int error = SETPOINT - position;

  // Compute the motor adjustment
  int adjust = error * KP + KD * (error - lastError);

  // Record the current error for the next iteration
  lastError = error;

  // Adjust motors, one negatively and one positively
  analogWrite(ENB, constrain(globallsp - adjust, MIN_SPEED, MAX_SPEED));
  analogWrite(ENA, constrain(globalrsp + adjust, MIN_SPEED, MAX_SPEED));
}

