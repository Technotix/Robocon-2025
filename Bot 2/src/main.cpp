// Include Libraries
#include <Arduino.h>
#include <math.h>

#define minSpeed 10
#define maxSpeed 255

#define deadzone 100

#define CH1 22
#define CH2 23
#define CH4 25

#define M1_START 37
#define M1_SPEED 9
#define M1_DIR 39
#define M2_START 40
#define M2_SPEED 10
#define M2_DIR 42
#define M3_START 43
#define M3_SPEED 5
#define M3_DIR 45
#define M4_START 46
#define M4_SPEED 6
#define M4_DIR 48

int front_left = 0, front_right = 0, back_right = 0, back_left = 0;
int XSpeed = 0, YSpeed = 0, TSpeed = 0;

// Function Prototypes
void updateMotors(int XSpeed, int YSpeed, int TSpeed);

double fmap(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  Serial.begin(115200);

  pinMode(M1_START, OUTPUT);
  pinMode(M1_SPEED, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_START, OUTPUT);
  pinMode(M2_SPEED, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_START, OUTPUT);
  pinMode(M3_SPEED, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_START, OUTPUT);
  pinMode(M4_SPEED, OUTPUT);
  pinMode(M4_DIR, OUTPUT);

  Serial.print(F("\r\nFlySky Receiver Library Started"));

  digitalWrite(M1_START, LOW);
  analogWrite(M1_SPEED, 0); 
  digitalWrite(M2_START, LOW);
  analogWrite(M2_SPEED, 0); 
  digitalWrite(M3_START, LOW);
  analogWrite(M3_SPEED, 0); 
  digitalWrite(M4_START, LOW);
  analogWrite(M4_SPEED, 0); 
}

void loop()
{

  int ch1 = pulseIn(CH1, HIGH);
  int ch2 = pulseIn(CH2, HIGH);
  int ch4 = pulseIn(CH4, HIGH);

  Serial.print("Ch1: ");
  Serial.print(ch1);
  Serial.print(" Ch2: ");
  Serial.print(ch2);
  Serial.print(" Ch4: ");
  Serial.print(ch4);

  if ((ch1 > 900 && ch1 < 2000) && (ch2 > 900 && ch2 < 2000) && (ch4 > 900 && ch4 < 2000))
  {
    XSpeed = 0;
    YSpeed = 0;
    TSpeed = 0;

    if (ch1 >= 1552)
    {
      YSpeed = fmap(ch1, 1552, 1976, 0, -255);
    }
    else if (ch1 <= 1452)
    {
      YSpeed = fmap(ch1, 1452, 987, 0, 255);
    }
    else
    {
      YSpeed = 0;
    }

    if (ch2 >= 1552)
    {
      XSpeed = fmap(ch2, 1552, 1976, 0, -255);
    }
    else if (ch2 <= 1452)
    {
      XSpeed = fmap(ch2, 1452, 987, 0, 255);
    }
    else
    {
      XSpeed = 0;
    }

    if (ch4 >= 1558)
    {
      TSpeed = fmap(ch4, 1558, 1732, 0, 255);
    }
    else if (ch4 <= 1458)
    {
      TSpeed = fmap(ch4, 1458, 1183, 0, -255);
    }
    else
    {
      TSpeed = 0;
    }

    updateMotors(XSpeed, YSpeed, TSpeed * 0.8);

    Serial.print("  Motor 1: ");
    Serial.print(front_left);
    Serial.print("  Motor 2: ");
    Serial.print(front_right);
    Serial.print("  Motor 3: ");
    Serial.print(back_right);
    Serial.print("  Motor 4: ");
    Serial.println(back_left);
  }
  else
  {
    updateMotors(0, 0, 0);
    Serial.println("");
  }
}

// Function Definitions
void updateMotors(int XSpeed, int YSpeed, int TSpeed)
{
  front_left = constrain(-XSpeed - YSpeed - TSpeed, -255, 255);
  front_right = constrain(-XSpeed + YSpeed - TSpeed, -255, 255);
  back_right = constrain(XSpeed + YSpeed - TSpeed, -255, 255);
  back_left = constrain(XSpeed - YSpeed - TSpeed, -255, 255);

  if (front_left > 0)
  {
    digitalWrite(M1_START, HIGH);      // Enable motor
    digitalWrite(M1_DIR, HIGH);        // Forward direction
    analogWrite(M1_SPEED, front_left); // Set speed
  }
  else if (front_left < 0)
  {
    digitalWrite(M1_START, HIGH);       // Enable motor
    digitalWrite(M1_DIR, LOW);          // Reverse direction
    analogWrite(M1_SPEED, -front_left); // Set speed
  }
  else
  {
    digitalWrite(M1_START, LOW); // Disable motor (brake)
    analogWrite(M1_SPEED, 0);    // Stop motor
  }

  if (front_right > 0)
  {
    digitalWrite(M2_START, HIGH);       // Enable motor
    digitalWrite(M2_DIR, HIGH);         // Forward direction
    analogWrite(M2_SPEED, front_right); // Set speed
  }
  else if (front_right < 0)
  {
    digitalWrite(M2_START, HIGH);        // Enable motor
    digitalWrite(M2_DIR, LOW);           // Reverse direction
    analogWrite(M2_SPEED, -front_right); // Set speed
  }
  else
  {
    digitalWrite(M2_START, LOW); // Disable motor (brake)
    analogWrite(M2_SPEED, 0);    // Stop motor
  }

  if (back_right > 0)
  {
    digitalWrite(M3_START, HIGH);      // Enable motor
    digitalWrite(M3_DIR, HIGH);        // Forward direction
    analogWrite(M3_SPEED, back_right); // Set speed
  }
  else if (back_right < 0)
  {
    digitalWrite(M3_START, HIGH);       // Enable motor
    digitalWrite(M3_DIR, LOW);          // Reverse direction
    analogWrite(M3_SPEED, -back_right); // Set speed
  }
  else
  {
    digitalWrite(M3_START, LOW); // Disable motor (brake)
    analogWrite(M3_SPEED, 0);    // Stop motor
  }

  if (back_left > 0)
  {
    digitalWrite(M4_START, HIGH);     // Enable motor
    digitalWrite(M4_DIR, HIGH);       // Forward direction
    analogWrite(M4_SPEED, back_left); // Set speed
  }
  else if (back_left < 0)
  {
    digitalWrite(M4_START, HIGH);      // Enable motor
    digitalWrite(M4_DIR, LOW);         // Reverse direction
    analogWrite(M4_SPEED, -back_left); // Set speed
  }
  else
  {
    digitalWrite(M4_START, LOW); // Disable motor (brake)
    analogWrite(M4_SPEED, 0);    // Stop motor
  }
}