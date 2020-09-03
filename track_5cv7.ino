#include <AFMotor.h>
#include <Ultrasonic.h>
#include <SimpleTimer.h>

#define lineSensor0 A0//most right,active low(0:white,1:black/nothing)
#define lineSensor1 A1
#define lineSensor2 A2//middle
#define lineSensor3 A3
#define lineSensor4 A4//most left
#define colorSensor A5
#define drivePastDelay 200
#define pauseMoveDelay 600

AF_DCMotor RightMotor(1);//MOTOR12_64KHZ,MOTOR12_8KHZ,MOTOR12_1KHZ(default)
AF_DCMotor LeftMotor(2);

Ultrasonic ultrasonic(10, 9); // (Trig PIN,Echo PIN)
SimpleTimer ultrasonicTimer(500);

int lineSensor0_val;
int lineSensor1_val;
int lineSensor2_val;
int lineSensor3_val;
int lineSensor4_val;
int lineSensor_val;
int lineSensor_val_previous;
int lineSensor_val_beforeWhite;
int colorSensor_val;
int colorSensor_val_previous;
int colorSensor_val_filt;

int motorSpeed;
int baseSpeed = 100;
int motorMaxSpeed = 100;
int sum_error = 0;

// PID
int error = 0;
int pre_error = 0;
int Kp = 20;
int Kd = 0.5;
int Ki = 0;
int max_sum_error = 10;
int min_sum_error = -10;

String path;
String path_reverse;
int path_index = 0;
bool isOptimum = false;

bool distFlag = false;
bool breakFlag = false;
bool blackLineDetect = false;
int blackLineCnt = 0;
int checkResult;

void setup() {
  Serial.begin(9600);// set up Serial library at 9600 bps
  Serial.println("Initial");
  path.reserve(100);
  path_reverse.reserve(100);

  pinMode(lineSensor0, INPUT);
  pinMode(lineSensor1, INPUT);
  pinMode(lineSensor2, INPUT);
  pinMode(lineSensor3, INPUT);
  pinMode(lineSensor4, INPUT);

  // turn on motor
  RightMotor.setSpeed(120);
  LeftMotor.setSpeed(120);

  //  RightMotor.run(FORWARD);
  //  LeftMotor.run(FORWARD);
  //
  //  while(true){}

  pauseMove();
  //path="SLBLBLBLLXBLB";

  //path = "LBLLBLL";
  //SimplifyPath();

  Serial.println(" wait");
  while (ultrasonic.Ranging(CM) > 5) {//wait hand in
    delay(1);
  }
  Serial.println(F(" ready"));
  while (ultrasonic.Ranging(CM) < 10) {//wait hand out
    delay(1);
  }
  Serial.println(F(" go"));
  delay(1000);

  //  moveForward();
  //  unsigned long t0, t1;
  //
  //  while (true) {
  //    lineSensor_val_previous = lineSensor_val;
  //    lineSensor_val = sensorReading();
  //    if ((lineSensor_val != 0) && (lineSensor_val != 4)) {
  //      t0 = millis();
  //      break;
  //    }
  //  }
  //  while (true) {
  //    lineSensor_val_previous = lineSensor_val;
  //    lineSensor_val = sensorReading();
  //    if (lineSensor_val == 0) {
  //      t1 = millis();
  //      break;
  //    }
  //  }
  //  Serial.println(t1 - t0);
  //  while (true) {
  //    pauseMove();
  //  }
}

void loop() {
  MazeSolving();
  SimplifyPath();// Simplify the learned path.
  RunningBack();
  //FastestRouteRunning();
}
void MazeSolving()
{
  Serial.println(F("MazeSolving"));
  while (true) {
    movement_task();
    //Sensor_task();
    //communication_task();
    if (breakFlag) {
      breakFlag = false;
      break;
    }
  }
}
void RunningBack()
{
  Serial.println(F("RunningBack"));
  while (true) {
    runBack_task();
    //Sensor_task();
    //communication_task();
    if (breakFlag) {
      breakFlag = false;
      break;
    }
  }
}
//  Wall Following -- Left Hand Rule
//    Always turn left if you can
//  If you cannot turn left, go straight
//  If you cannot turn left, or go straight, turn right
//  If you cannot turn left, go straight, or turn right, turn around because you must be at a dead end

// The path variable will store the path that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
// You should check to make sure that the path_length of your
// maze design does not exceed the bounds of the array.

//    Reduction
//    LBR = B
//    LBS = R
//    LBL = S
//    SBL = R
//    SBS = B
//    RBL = B

void SimplifyPath()
{
  int Xindex = path.indexOf('X');
  path = path.substring(0, Xindex);
  Serial.println(F("SimplifyPath"));
  Serial.print(F(" Search path: "));//LBLBLLBSLLSRBLLSSLBRLBLL
  Serial.println(path);
  do {
    path.replace("LBR", "B");
    path.replace("LBS", "R");
    path.replace("RBL", "B");
    path.replace("SBL", "R");
    path.replace("SBS", "B");
    path.replace("LBL", "S");
    isOptimum = !((path.indexOf("LBR") >= 0) || (path.indexOf("LBS") >= 0) || (path.indexOf("RBL") >= 0) || (path.indexOf("SBL") >= 0) || (path.indexOf("SBS") >= 0) || (path.indexOf("LBL") >= 0));
    Serial.print(" ");
    Serial.println(path);
  } while (!isOptimum);
  Serial.print(F(" Optimal path: "));//RRLLRR
  Serial.println(path);

  path_reverse = path;
  path_reverse.replace("B", "b");
  path_reverse.replace("L", "r");
  path_reverse.replace("R", "l");
  path_reverse.replace("S", "s");
  Serial.print(F(" Optimal path_reverse: "));//llrrll
  Serial.println(path_reverse);
  path_index = path_reverse.length() - 1;
  Serial.println(path_index);

  //Serial.println(path_reverse.charAt(path_index-1));
} // end SimplifyPath

void FastestRouteRunning()
{
  Serial.println(F("FastestRouteRunning"));
  while (true) {
    //movement_task();
    //Sensor_task();
    //communication_task();
    if (breakFlag) {
      breakFlag = false;
      break;
    }
  }
}

void movement_task() {
  lineSensor_val_previous = lineSensor_val;
  lineSensor_val = sensorReading();
  decisionMaking(lineSensor_val);
  carAction(lineSensor_val);
}

void Sensor_task() {
  if (ultrasonicTimer.isReady()) {
    if (ultrasonic.Ranging(CM) < 5) {
      distFlag = true;
    }
    if (ultrasonic.Ranging(CM) > 10) {
      if (distFlag == true) {
        breakFlag = true;
        distFlag = false;
      }
    }
    ultrasonicTimer.reset();
  }
}
void runBack_task() {
  lineSensor_val_previous = lineSensor_val;
  lineSensor_val = sensorReading();
  runBack();
}
int sensorReading() {
  //(0:white,1:black/nothing)
  lineSensor0_val = !digitalRead(lineSensor0);//right
  lineSensor1_val = !digitalRead(lineSensor1);
  lineSensor2_val = !digitalRead(lineSensor2);//middle
  lineSensor3_val = !digitalRead(lineSensor3);
  lineSensor4_val = !digitalRead(lineSensor4);//left

  //  colorSensor_val = analogRead(colorSensor);
  //  colorSensor_val_filt = (0.2 * colorSensor_val) + (0.8 * colorSensor_val_previous);
  //  if (colorSensor_val_filt > 500) {//(1024->0:white,0->1:black/nothing)
  //    lineSensor5_val = 0;
  //  } else {
  //    lineSensor5_val = 1;
  //  }
  //  colorSensor_val_previous = colorSensor_val_filt;

  int val =  (lineSensor4_val * 16) + (lineSensor3_val * 8) + (lineSensor2_val * 4) + (lineSensor1_val * 2) + lineSensor0_val;

  return val;
}

//     16 8421
// 0b0000 0000 -  0 all white
// 0b0000 0001 -  1 most right
// 0b0000 0011 -  3
// 0b0000 0010 -  2
// 0b0000 0110 -  6
// 0b0000 0100 -  4 middle
// 0b0000 1100 - 12
// 0b0000 1000 -  8
// 0b0001 1000 - 24
// 0b0001 0000 - 16


// 7,28,31 found intersection

// 0b0001 1100 - 28 turn left detect
// 0b0001 1110 - 30

// 0b0000 0111 -  7 turn right detect
// 0b0000 1111 - 15

// 0b0001 1111 - 31 (all black) T,cross,stop detect

//unknown
// 0b0000 0101 - 5
// 0b0000 0111 - 7
// 0b0000 1001 - 9
// 0b0000 1010 - 10
// 0b0000 1011 - 11
// 0b0000 1101 - 13
// 0b0000 1110 - 14

// 0b0001 0001 - 17
// 0b0001 0010 - 18
// 0b0001 0011 - 19
// 0b0001 0100 - 20
// 0b0001 0101 - 21
// 0b0001 0110 - 22
// 0b0001 0111 - 23
// 0b0001 1001 - 25
// 0b0001 1010 - 26
// 0b0001 1011 - 27
// 0b0001 1101 - 29
// 0b0001 1110 - 30

void decisionMaking(int sensorValue) {
  if (lineSensor_val_previous != sensorValue) {
    Serial.print(F(" "));
    Serial.print(sensorValue);
    Serial.print(F(" "));
    Serial.println(path);
  }
  if (sensorValue != 0) {
    lineSensor_val_beforeWhite = sensorValue;
  }
  switch (sensorValue) {//1,3,2,6,4,12,8,24,16 line following
    case 0: //line detected by none = all white
      error = 0;
      break;
    case 1://line detected by right sensor
      error = -4;
      break;
    case 3:
      error = -3;
      break;
    case 2:
      error = -2;
      break;
    case 6:
      error = -1;
      break;
    case 4://line detected by middle sensor
      error = 0;
      break;
    case 12:
      error = 1;
      break;
    case 8:
      error = 2;
      break;
    case 24:
      error = 3;
      break;
    case 16: //line detected by left sensor
      error = 4;
      break;
    case 30: //turn left
    case 28: //turn left
      error = 0;
      break;
    case 15: //turn right
    case 7: //turn right
      error = 0;
      break;
    case 31://line detected all black
      if (lineSensor_val_previous != sensorValue) {
        blackLineCnt++;
      }
      error = 0;
      break;
    default:
      Serial.print(F(" unknown "));
      Serial.println(sensorValue);
      error = 0;
      break;
  }

  //return something?????
}


void carAction(int sensorValue) {
  if (lineSensor_val_previous != sensorValue) {
    if (lineSensor_val == 6) {
      RightMotor.setSpeed(40);
      LeftMotor.setSpeed(120);
    } else if (lineSensor_val == 12) {
      RightMotor.setSpeed(120);
      LeftMotor.setSpeed(40);
    } else {
      RightMotor.setSpeed(120);
      LeftMotor.setSpeed(120);
    }
  }
  switch (sensorValue) {
    case 0: //line detected by none = all white
      if ((lineSensor_val_beforeWhite == 6) || (lineSensor_val_beforeWhite == 4) || (lineSensor_val_beforeWhite == 12)) {
        checkResult = straightCheck();
        if (checkResult == 0) {//can't go straight
          //U-turn
          Serial.println(F(" U-turn"));
          turnAround();
        }
      } else if (lineSensor_val_beforeWhite == 1) {
        //left out
        Serial.println(F(" left out"));
      } else if (lineSensor_val_beforeWhite == 16) {
        //right out
        Serial.println(F(" right out"));
      }
      break;
    case 1://line detected by right sensor
      rotateRight();
      break;
    case 3:
      rotateRight();
      break;
    case 2:
      RightMotor.run(RELEASE);
      LeftMotor.run(FORWARD);
      break;
    case 6:
      moveForward();
      //RightMotor.run(RELEASE);
      //LeftMotor.run(FORWARD);
      break;
    case 4: //line detected by middle sensor
      //move forward
      moveForward();
      break;
    case 12:
      moveForward();
      //RightMotor.run(FORWARD);
      //LeftMotor.run(RELEASE);
      break;
    case 8:
      RightMotor.run(FORWARD);
      LeftMotor.run(RELEASE);
      break;
    case 24:
      rotateLeft();
      break;
    case 16: //line detected by left sensor
      rotateLeft();
      break;
    case 30:
    case 28: //turn left
      moveForward();
      delay(50);//repeat read sensor <130 ms
      lineSensor_val_previous = lineSensor_val;
      lineSensor_val = sensorReading();
      if (lineSensor_val_previous != lineSensor_val) {
        delay(5);
        lineSensor_val_previous = lineSensor_val;
        lineSensor_val = sensorReading();
      }
      Serial.print(F(" ^"));
      Serial.println(lineSensor_val);
      if ((lineSensor_val == 28) || (lineSensor_val == 30)) {
        checkResult = straightCheck();
        if (checkResult > 0) {//can go straight
          path += 'L';
          turnLeft();
        } else {
          turnLeft();
        }
      }
      break;
    case 15:
    case 7: //turn right
      delay(50);//repeat read sensor <130 ms
      lineSensor_val_previous = lineSensor_val;
      lineSensor_val = sensorReading();
      if (lineSensor_val_previous != lineSensor_val) {
        delay(5);
        lineSensor_val_previous = lineSensor_val;
        lineSensor_val = sensorReading();
      }
      Serial.print(F(" ^"));
      Serial.println(lineSensor_val);
      if ((lineSensor_val == 7) || (lineSensor_val == 15)) {
        checkResult = straightCheck();
        if (checkResult > 0) {//can go straight
          path += 'S';
          moveForward();
        } else {
          turnRight();
        }
      }
      break;
    case 31://line detected all black
      Serial.println(F(" =31="));
      //pauseMove();
      checkResult = straightCheck();
      if (checkResult > 0) {//can go straight ,4 junction
        turnLeft();
        path += 'L';
      } else if (checkResult == 0) {//can't go straight ,3 junction
        turnLeft();
        path += 'L';
      } else {
        path += 'X';
        turnAround();
        pauseMove();
        breakFlag = true;
        delay(10000);
      }
      break;
    default:
      pauseMove();
      break;
  }

  calculatePID();
}
void runBack(void) {
  if (lineSensor_val_previous != lineSensor_val) {
    Serial.print(F(" "));
    Serial.print(lineSensor_val);
    Serial.print(F(" "));
    Serial.println(path_reverse);
  }
  switch (lineSensor_val) {
    case 0: //line detected by none = all white
      pauseMove();
      path_index--;
      break;
    case 1://line detected by right sensor
      rotateRight();
      break;
    case 3:
      rotateRight();
      break;
    case 2:
      RightMotor.run(RELEASE);
      LeftMotor.run(FORWARD);
      break;
    case 6:
      RightMotor.run(RELEASE);
      LeftMotor.run(FORWARD);
      break;
    case 4: //line detected by middle sensor
      //move forward
      moveForward();
      break;
    case 12:
      RightMotor.run(FORWARD);
      LeftMotor.run(RELEASE);
      break;
    case 8:
      RightMotor.run(FORWARD);
      LeftMotor.run(RELEASE);
      break;
    case 24:
      rotateLeft();
      break;
    case 16: //line detected by left sensor
      rotateLeft();
      break;
    case 30:
    case 28: //turn left
      delay(50);//repeat read sensor <130 ms
      lineSensor_val_previous = lineSensor_val;
      lineSensor_val = sensorReading();
      if ((lineSensor_val == 28) || (lineSensor_val == 30)) {
        Serial.print(F(" 28/30"));
        checkResult = straightCheck();
        if (checkResult > 0) {//can go straight
          Serial.println(path_reverse.charAt(path_index));
          if (path_reverse.charAt(path_index) == 's') {
            moveForward();
          } else if (path_reverse.charAt(path_index) == 'r') {
            turnRight();
          } else if (path_reverse.charAt(path_index) == 'l') {
            turnLeft();
          }
          path_index--;
        } else {
          turnLeft();
        }
      }
      break;
    case 15:
    case 7: //turn right
      delay(50);//repeat read sensor <130 ms
      lineSensor_val_previous = lineSensor_val;
      lineSensor_val = sensorReading();
      if ((lineSensor_val == 7) || (lineSensor_val == 15)) {
        Serial.print(F(" 7/15"));
        checkResult = straightCheck();
        if (checkResult > 0) {//can go straight
          Serial.println(path_reverse.charAt(path_index));
          if (path_reverse.charAt(path_index) == 's') {
            moveForward();
          } else if (path_reverse.charAt(path_index) == 'r') {
            turnRight();
          } else if (path_reverse.charAt(path_index) == 'l') {
            turnLeft();
          }
          path_index--;
        } else {
          turnRight();
        }
      }
      break;
    case 31://line detected all black
      //pauseMove();
      Serial.print(F(" 31"));
      checkResult = straightCheck();
      Serial.println(path_reverse.charAt(path_index));
      if (checkResult > 0) {//can go straight ,4 junction
        if (path_reverse.charAt(path_index) == 's') {
          moveForward();
        } else if (path_reverse.charAt(path_index) == 'r') {
          turnRight();
        } else if (path_reverse.charAt(path_index) == 'l') {
          turnLeft();
        }
        path_index--;
      } else if (checkResult == 0) {//can't go straight ,3 junction
        if (path_reverse.charAt(path_index) == 's') {
          moveForward();
        } else if (path_reverse.charAt(path_index) == 'r') {
          turnRight();
        } else if (path_reverse.charAt(path_index) == 'l') {
          turnLeft();
        }
        path_index--;
      }
      break;
    default:
      pauseMove();
      break;
  }
}
void pauseMove() {
  RightMotor.run(RELEASE);
  LeftMotor.run(RELEASE);
}
void moveForward() {
  RightMotor.run(FORWARD);
  LeftMotor.run(FORWARD);
}
void rotateRight() {
  RightMotor.run(BACKWARD);
  LeftMotor.run(FORWARD);
}
void rotateLeft() {
  RightMotor.run(FORWARD);
  LeftMotor.run(BACKWARD);
}
int straightCheck() {
  Serial.print(F(" straightCheck "));
  moveForward();
  delay(drivePastDelay);
  pauseMove();
  delay(pauseMoveDelay);
  lineSensor_val_previous = lineSensor_val;
  lineSensor_val = sensorReading();
  Serial.println(lineSensor_val);
  if ((lineSensor_val == 3) || (lineSensor_val == 2) || (lineSensor_val == 6) || (lineSensor_val == 4) || (lineSensor_val == 12) || (lineSensor_val == 8) || (lineSensor_val == 24) ) { //line detected by middle sensor
    //can go stress
    Serial.println(F(" can go stress"));
    return 1;
  } else if ((lineSensor_val == 31) || (lineSensor_val == 15) || (lineSensor_val == 30)) { //line detected all black
    //finish maze
    Serial.println(F(" finish maze"));
    return -1;
  }

  return 0;
}

void turnRight() {
  pauseMove();
  delay(pauseMoveDelay);
  rotateRight();
  if ((lineSensor_val == 1) || (lineSensor_val == 3) || (lineSensor_val == 2)) {
    delay(700);
  } else {
    delay(500);
  }
  pauseMove();
  delay(pauseMoveDelay);
  while (true) { //line detected by middle sensor
    rotateRight();
    lineSensor_val_previous = lineSensor_val;
    lineSensor_val = sensorReading();
    if ((lineSensor_val == 6) || (lineSensor_val == 4) || (lineSensor_val == 12)) {
      break;
    }
  }
}
void turnLeft() {
  pauseMove();
  delay(pauseMoveDelay);
  rotateLeft();
  if ((lineSensor_val == 16) || (lineSensor_val == 24) || (lineSensor_val == 8)) {
    delay(700);
  } else {
    delay(500);
  }
  pauseMove();
  delay(pauseMoveDelay);
  while (true) { //line detected by middle sensor
    rotateLeft();
    lineSensor_val_previous = lineSensor_val;
    lineSensor_val = sensorReading();
    if ((lineSensor_val == 6) || (lineSensor_val == 4) || (lineSensor_val == 12)) {
      break;
    }
  }
}
void turnAround()
{
  RightMotor.setSpeed(120);
  LeftMotor.setSpeed(120);
  turnLeft();
  path += 'B';
}
// Turns to the sent variable of
// 'L' (left), 'R' (right), 'S' (straight), or 'B' (back)
void turn(char dir)
{
  switch (dir)
  {
    // Turn left 90deg
    case 'L':

      break;

    // Turn right 90deg
    case 'R':

      break;

    // Turn right 180deg to go back
    case 'B':

      break;

    // Straight ahead
    case 'S':
      // do nothing
      break;
    default:
      break;
  }
} // end turn

void calculatePID()
{
  int rightSpeed, leftSpeed;

  sum_error += error;
  sum_error = constrain(sum_error, min_sum_error, max_sum_error);

  //   motorSpeed = Kp*error + Kd*(error - pre_error) + Ki*(sum_error);
  //   leftSpeed = baseSpeed + motorSpeed;
  //   rightSpeed = baseSpeed - motorSpeed;
  //
  //   leftSpeed = constrain(leftSpeed, -motorMaxSpeed, motorMaxSpeed);
  //   rightSpeed = constrain(rightSpeed, -motorMaxSpeed, motorMaxSpeed);
  //
  //   LeftMotor.setSpeed(leftSpeed);
  //   RightMotor.setSpeed(rightSpeed);

  //Serial.println(error);

  pre_error = error;
}
