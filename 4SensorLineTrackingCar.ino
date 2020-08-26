
#include <SimpleTimer.h> 

/*-------เซนเซอร์ร์หน้า------*/
int LS1 = 2;     
int LS2  = 12;   
int RS1 = 11;     
int RS2  = 8;      

int BL1 = A0;
int BR1 = A1;

//int LoadSensor1 = A7;

/*-------definning Outputs------*/
int LM1 =  6;       // left motor
int LM2 =  7;       // left motor
int RM1 =  4;       // right motor
int RM2 =  5;       // right motor

/*-------PWM ควบคุมความเร็วมอเตอร์-----*/
int PWM1 = 10;    // PWM left motor
int PWM2 = 3;    // PWM right motor
int LowSpeed =70;  // Speed PWM สามารถปรับความเร็วได้ถึง 255
int MediumSpeed = 100;

//int Outline;
//int timers1;

int DLS1;
int DLS2;
int DRS1;
int DRS2;
int BLS1;
int BRS1;

//int LoadCheck1;
//int StationStatus;
//int trackpoint;

int DriveStatus;
int TurnLogic;
SimpleTimer timer;

void setup()
{
  Serial.begin(9600);
  
  pinMode(LS1, INPUT);
  pinMode(RS1, INPUT);
  pinMode(LS2, INPUT);
  pinMode(RS2, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(BL1, INPUT);
  pinMode(BR1, INPUT);
  DriveStatus = 0;
  TurnLogic == 0;
 //timer.setInterval(10,Trackingline);      //  track เส้นทุกๆ 10 msec
 //timer.setInterval(1000, Monitorsensor);   //  monitor เส้นทุกๆ 1 second
 
}

void loop()
{
  Trackingline();
  Serial.println(TurnLogic);
  MotorAction();
  delay(50);
}

void Trackingline()
{
  DLS1 = digitalRead(LS1);
  DLS2 = digitalRead(LS2);
  DRS1 = digitalRead(RS1);
  DRS2 = digitalRead(RS2);

  /*switch (TurnLogic){
    case 1:
      if ( (DLS1 == 0) && (DLS2 == 1) && (DRS1 == 1) && (DRS2 == 0) )   // Forward
        {
          analogWrite(PWM1, LowSpeed);
          analogWrite(PWM2, LowSpeed);
          Forward();
        }

      else if( (DLS1 == 0) && (DLS2 == 0) && (DRS1 == 1) && (DRS2 == 0) )   // Forward
        {
          analogWrite(PWM1, LowSpeed);
          analogWrite(PWM2, LowSpeed);
          Forward();
        }

      else if ( (DLS1 == 0) && (DLS2 == 1) && (DRS1 == 0) && (DRS2 == 0) )   // Forward
        {
          analogWrite(PWM1, LowSpeed);
          analogWrite(PWM2, LowSpeed);
          Forward();
        }
  
      else if ( (DLS1 == 1) && (DLS2 == 0) && (DRS1== 0) && (DRS2 == 0) )    // Turn Left
        {
          analogWrite(PWM1, LowSpeed);
          analogWrite(PWM2, LowSpeed);
          TurnLeft();
        }

      else if ( (DLS1 == 1) && (DLS2 == 1) && (DRS1== 0) && (DRS2 == 0) )    // Turn Left
        {
          analogWrite(PWM1, LowSpeed);
          analogWrite(PWM2, LowSpeed);
          TurnLeft();
        }

      else if ( (DLS1 == 1) && (DLS2 == 1) && (DRS1== 1) && (DRS2 == 0) )    // Rotate Left
        {
          analogWrite(PWM1, MediumSpeed);
          analogWrite(PWM2, MediumSpeed);
          RotateLeft;
          TurnLogic=2;
          
          //delay(100);
          //RotateLeft();
          //delay(500);
         }
  
      else if ( (DLS1 == 0) && (DLS2 == 0) && (DRS1== 0) && (DRS2 == 1) )    // Turn Right
         {
          analogWrite(PWM1, LowSpeed);
          analogWrite(PWM2, LowSpeed);
          TurnRight();
          }

      else if ( (DLS1 == 0) && (DLS2 == 0) && (DRS1== 1) && (DRS2 == 1) )    // Turn Right
         {
          analogWrite(PWM1, LowSpeed);
          analogWrite(PWM2, LowSpeed);
          TurnRight();
         }

      else if ( (DLS1 == 0) && (DLS2 == 1) && (DRS1== 1) && (DRS2 == 1) )    // Rotate Right
         {
          analogWrite(PWM1, MediumSpeed);
          analogWrite(PWM2, MediumSpeed);
          RotateRight();
          TurnLogic=2;
          
          //delay(100);
          //RotateRight();
          //delay(500);
         }
  
      else // Stop Car
         {
          StopCar();
         }
    default:
      if ( (DLS1 == 0) && (DLS2 == 1) && (DRS1 == 1) && (DRS2 == 0) )   // Forward
        {
          analogWrite(PWM1, LowSpeed);
          analogWrite(PWM2, LowSpeed);
          Forward();
          TurnLogic=1;
        }
       else
        {
          TurnLogic=2;
        }
       

          
    }*/
      if ( (DLS1 == 0) && (DLS2 == 1) && (DRS1 == 1) && (DRS2 == 0) )   //  Forward
        { 
          DriveStatus = 1;
          TurnLogic == 0; 
        }
      else if ( (DLS1 == 0) && (DLS2 == 0) && (DRS1== 1) && (DRS2 == 1) )    // 
        {
          DriveStatus = 1;
          TurnLogic == 0;
        }
      else if ( (DLS1 == 1) && (DLS2 == 1) && (DRS1== 0) && (DRS2 == 0) )    // 
        {
          DriveStatus = 1;
          TurnLogic == 0;
        }
      else if ( (DLS1 == 0) && (DLS2 == 0) && (DRS1== 0) && (DRS2 == 1) )    // Rotate right 
        {
          DriveStatus = 2;
          TurnLogic = 1;
        }
      else if ( (DLS1 == 1) && (DLS2 == 0) && (DRS1== 0) && (DRS2 == 0) )    // Rotate Left 
        {
          DriveStatus = 2;
          TurnLogic = 2;
        }
      else
        {
          ;
        }
}

void MotorAction()
{
  if ( DriveStatus == 1 )
    { 
      if ( (DLS1 == 0) && (DLS2 == 1) && (DRS1 == 1) && (DRS2 == 0) )   //  Forward
        { 
          analogWrite(PWM1, LowSpeed);
          analogWrite(PWM2, LowSpeed);
          Forward();
        }
      else if ( (DLS1 == 0) && (DLS2 == 0) && (DRS1== 1) && (DRS2 == 1) )    //  TurnRight
        {
          analogWrite(PWM1, MediumSpeed);
          analogWrite(PWM2, MediumSpeed);
          TurnRight();
        }
      else if ( (DLS1 == 1) && (DLS2 == 1) && (DRS1== 0) && (DRS2 == 0) )    // TurnLeft
        {
          analogWrite(PWM1, MediumSpeed);
          analogWrite(PWM2, MediumSpeed);
          TurnLeft();
      }

    }
  else if ( DriveStatus == 2)
    {
      if ( (DLS1 == 0) && (DLS2 == 1) && (DRS1 == 1) && (DRS2 == 0) )   //  
       { 
          DriveStatus = 1;
       }
      else if ( TurnLogic == 1)   //  
       { 
          analogWrite(PWM1, MediumSpeed);
          analogWrite(PWM2, MediumSpeed);
          RotateRight();
       }
      else if ( TurnLogic == 2 )   //  
       { 
          analogWrite(PWM1, MediumSpeed);
          analogWrite(PWM2, MediumSpeed);
          RotateLeft();
       }
    }
  else
    {
      StopCar();
    }
}

/*-------สำหรับการ debug ------*/
void Monitorsensor()
{
//LoadCheck1 = analogRead(LoadSensor1);
//Serial.print("LoadCheck1: ");
Serial.println(TurnLogic);
BLS1 = analogRead(BL1);
BRS1 = analogRead(BR1);
Serial.print("BLS1: ");
Serial.println(BLS1);
Serial.print("BRS1: ");
Serial.println(BRS1);

//Serial.print("Outline: ");
//Serial.println(Outline);
//Serial.print("Sensor = " + LS1 + " " + LS2 + " " + RS1 + " " + RS2);
Serial.print(" ");
Serial.print("DLS1: ");
Serial.println(DLS1);
Serial.print("DLS2: ");
Serial.println(DLS2);
Serial.print("DRS1: ");
Serial.println(DRS1);
Serial.print("DRS2: ");
Serial.println(DRS2);
Serial.print(" ");
Serial.print(" ");
Serial.print(" ");
}

/*-------ฟังชั่นการวิ่ง------*/
void RotateRight()
{   
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);    
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, HIGH);
}

void RotateLeft()
{
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, HIGH);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
}

void TurnRight()
{   
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
}


void TurnLeft()
{
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
}


void Forward()
{
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW); 
}


void StopCar()
{
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW); 
}
