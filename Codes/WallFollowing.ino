#define ENA 6
#define IN1 32
#define IN2 33

#define ENB 7
#define IN3 35
#define IN4 34

#define TRIG_LEFT   42
#define ECHO_LEFT   43
#define TRIG_RIGHT  45
#define ECHO_RIGHT  44
#define TRIG_FRONT  41
#define ECHO_FRONT  40

float kpWall = 0.28;
float kdWall = 3.5;


enum juncType { STRAIGHT,
                DEAD_END,
                RIGHT_BEND,
                RIGHT_BRANCH,
                LEFT_BEND,
                LEFT_BRANCH,
                T_JUNC };

float lastErr_Wall = 0;
unsigned long prevTime_Wall = 0;
bool leftbranch = false;
bool rightbranch = false;
bool Tbranch = false;

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

void setForward(int leftSpeed, int rightSpeed)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENB, leftSpeed);
  analogWrite(ENA, rightSpeed);
}

void turnRight(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA,90);
  analogWrite(ENB,90);
  delay(1000);
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}

void turnLeft(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA,90);
  analogWrite(ENB,90);
  delay(1000);
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}

void turn360(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA,90);
  analogWrite(ENB,90);
  delay(2000);
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}

void gountil(){
  while (readUltrasonic(TRIG_FRONT, ECHO_FRONT) > 10)
    {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA,90);
    analogWrite(ENB,90);
    delay(50);
    }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(50);
    
}

void gofront(){
   digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA,90);
  analogWrite(ENB,90);

}

void stopmotors(){
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

}



void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

}

void loop() {
   mazeFollowing();
  }

void mazeFollowing(){
  float distR = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  delay(50);
  float distL = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  delay(50);
  float distF = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  delay(50);

  Serial.println(distF);

  juncType j = junc(distL, distR, distF);
  if (j == STRAIGHT) {
    readWall(distL, distR);
  } 
  else if (j == DEAD_END) {
    analogWrite(ENA,0);
    analogWrite(ENB,0);
    delay(800);
    turn360();
  } 
  else if (j == LEFT_BEND) {
    gountil();
    delay(1500);
    turnLeft();
    delay(990);
    gofront();
    delay(1000);
    }
  else if (j == LEFT_BRANCH) {
     if (leftbranch != true){
      rightbranch = true;
      stopmotors();
      delay(1000);
      gofront();
      delay(1500);
      }
    else {
      leftbranch = false;
      gountil();
      delay(1500);
      turnLeft();
      delay(990);
      gofront();
      delay(1000);

    }

  }

  else if (j == RIGHT_BEND) {
    gountil();
    delay(1500);
    turnRight();
    delay(990);
    gofront();
    delay(1000);
    
  }

  else if (j == RIGHT_BRANCH){
    if (rightbranch != true){
      leftbranch = true;
      stopmotors();
      delay(1000);
      gofront();
      delay(1500);
      }
    else {
      gountil();
      delay(1500);
      turnRight();
      delay(990);
      gofront();
      delay(1000);
    }


  } 

  else if (j == T_JUNC) {
    if (Tbranch != true){
    Tbranch = true;
    gountil();
    delay(1500);
    turnRight();
    delay(990);
    gofront();
    delay(1000);
    }
    else{
      Tbranch = false;
      gountil();
      delay(1500);
      turnRight();
      delay(990);
      gofront();
      delay(1000);


    }


}
}

juncType junc(float L, float R, float F) {
  if (L < 30.0 && R < 30.0) {
    if (F < 20.0) {
      return DEAD_END;
    }
    return STRAIGHT;
  } 
  
  else if ((L > 30 && R < 30) || (L < 30 && R > 30) || (L > 30 && R > 30)) {
    L = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
    delay(50);
    R = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
    delay(50);
    F = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    delay(50);

    if (L > 30 && R < 30) {
      if (F < 30) {
        return LEFT_BEND;
      }
      return LEFT_BRANCH;
    } 
    
    else if (L < 30 && R > 30) {
      if (F < 30) {
      return RIGHT_BEND;
      }
      return RIGHT_BRANCH;
    } 
    
    else if (L > 30 && R > 30) {
      return T_JUNC;
    }
  }
  return STRAIGHT;

}

void readWall(float leftDist,float rightDist){
  unsigned long now_Wall = millis();
  float dt_Wall = (now_Wall - prevTime_Wall);
  if (dt_Wall < 1) dt_Wall = 1;   // prevent divide-by-zero
  prevTime_Wall = now_Wall;

  // --- WALL POSITION ERROR (mm) ---
  // Positive → robot closer to LEFT wall
  float pos_Wall = leftDist - rightDist;

  // Normalize error to controller range
  float err_Wall = map(pos_Wall, -50, 50, -1000, 1000);
  err_Wall = constrain(err_Wall, -1000, 1000);

  // --- PD CONTROLLER ---
  float diff_Wall = kpWall * err_Wall
                  + kdWall * (err_Wall - lastErr_Wall) / dt_Wall;

  lastErr_Wall = err_Wall;

  // --- MOTOR SPEEDS ---
  int baseSpeed_Wall = 90;

  int leftMotor_Wall  = baseSpeed_Wall - diff_Wall;
  int rightMotor_Wall = baseSpeed_Wall + diff_Wall;

  // Safety limits
  leftMotor_Wall  = constrain(leftMotor_Wall, 0, 150);
  rightMotor_Wall = constrain(rightMotor_Wall, 0, 150);

  // Drive motors
  setForward(leftMotor_Wall, rightMotor_Wall);

}



