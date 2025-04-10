// Remove Comment to Enable Alternate XBOX Receiver Mode
#define XBOX_ALT_MODE

// Include Libraries
#include <Arduino.h>
#include <Cytron_SmartDriveDuo.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#ifdef XBOX_ALT_MODE
#include <XBOXUSB.h>
#else
#include <XBOXRECV.h>
#endif

// Constants
#define conid 0
#define deadzone 0

#define minSpeed 10
#define maxSpeed 75

#define mdds_1_2 35
#define mdds_3_4 37

// Global Variables
Cytron_SmartDriveDuo motor1motor2(SERIAL_SIMPLIFIED, mdds_1_2, 115200);
Cytron_SmartDriveDuo motor3motor4(SERIAL_SIMPLIFIED, mdds_3_4, 115200);

USB Usb;
#ifdef XBOX_ALT_MODE
XBOXUSB Xbox(&Usb);
#else
XBOXRECV Xbox(&Usb);
#endif

int front_left = 0, front_right = 0, back_right = 0, back_left = 0;

// Function Prototypes
void updateMotors(int XSpeed, int YSpeed, int TSpeed);
bool xboxConnCheck();

void setup()
{
  Wire.begin();
  Serial.begin(115200);
#if !defined(_MIPSEL_)
  while (!Serial)
    ;
#endif
  if (Usb.Init() == -1)
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));

  updateMotors(0, 0, 0);
}

void loop()
{
  Usb.Task();
  if (xboxConnCheck())
  {
#ifdef XBOX_ALT_MODE
    int leftHatY = Xbox.getAnalogHat(LeftHatY);
    int leftHatX = Xbox.getAnalogHat(LeftHatX);
    int rightHatX = Xbox.getAnalogHat(RightHatX);
#else
    int leftHatY = Xbox.getAnalogHat(LeftHatY, conid);
    int leftHatX = Xbox.getAnalogHat(LeftHatX, conid);
    int rightHatX = Xbox.getAnalogHat(RightHatX, conid);
#endif

    int XSpeed = 0, YSpeed = 0, TSpeed = 0;

    if (leftHatX > deadzone)
    {
      XSpeed = map(leftHatX, deadzone, 32767, minSpeed, maxSpeed);
    }
    else if (leftHatX < -deadzone)
    {
      XSpeed = map(leftHatX, -32768, -deadzone, -maxSpeed, -minSpeed);
    }
    if (leftHatY > deadzone)
    {
      YSpeed = map(leftHatY, deadzone, 32767, minSpeed, maxSpeed);
    }
    else if (leftHatY < -deadzone)
    {
      YSpeed = map(leftHatY, -32768, -deadzone, -maxSpeed, -minSpeed);
    }
    if (rightHatX > deadzone)
    {
      TSpeed = map(rightHatX, deadzone, 32767, minSpeed, maxSpeed);
    }
    else if (rightHatX < -deadzone)
    {
      TSpeed = map(rightHatX, -32768, -deadzone, -maxSpeed, -minSpeed);
    }

    updateMotors(XSpeed, YSpeed, TSpeed * 0.8);

    Serial.print("Motor 1: ");
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
  }
}

// Function Definitions
void updateMotors(int XSpeed, int YSpeed, int TSpeed)
{
  front_left = constrain(-XSpeed + YSpeed + TSpeed, -100, 100);
  front_right = constrain(-XSpeed - YSpeed + TSpeed, -100, 100);
  back_right = constrain(XSpeed + YSpeed + TSpeed, -100, 100);
  back_left = constrain(XSpeed - YSpeed + TSpeed, -100, 100);

  motor1motor2.control(front_left, front_right);
  motor3motor4.control(back_left, back_right);
}

bool xboxConnCheck()
{
#ifdef XBOX_ALT_MODE
  if (Xbox.Xbox360Connected)
  {
    return true;
  }
  return false;
#else
  if (Xbox.XboxReceiverConnected)
  {
    if (Xbox.Xbox360Connected[conid])
    {
      return true;
    }
  }
  return false;
#endif
}