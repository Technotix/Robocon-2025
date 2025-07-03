#define XBOX_ALT_MODE
#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Arduino.h>
#include <Cytron_SmartDriveDuo.h>
#include <Encoder.h>
#include <math.h>
#include <ODriveArduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#ifdef XBOX_ALT_MODE
#include <XBOXUSB.h>
#else
#include <XBOXRECV.h>
#endif

#define CONTROLLER_ID 0
#define JOYSTICK_DEADZONE 2000

#define MIN_SPEED 10
#define MAX_SPEED 50
#define TURN_MULTIPLIER 0.8

#define MDDS_FRONT_MOTORS 41
#define MDDS_REAR_MOTORS 40
#define DRIBBLE_MOTORS 31

#define ENCODER_1_A 2
#define ENCODER_1_B 3
#define ENCODER_2_A 18
#define ENCODER_2_B 19
#define ENCODER_3_A 20
#define ENCODER_3_B 21

#define linearPWM 4
#define linearDIR 39

#define ENCODER_PPR 600.0
#define WHEEL_DIAMETER_MM 58.0
#define WHEEL_BASE_CM 26.5

HardwareSerial &odrive_serial = Serial2;
ODriveArduino odrive(odrive_serial);

SoftwareSerial loggingSerial(23, 22);

Cytron_SmartDriveDuo frontMotors(SERIAL_SIMPLIFIED, MDDS_FRONT_MOTORS, 115200);
Cytron_SmartDriveDuo rearMotors(SERIAL_SIMPLIFIED, MDDS_REAR_MOTORS, 115200);
Cytron_SmartDriveDuo dribblingMotors(SERIAL_SIMPLIFIED, DRIBBLE_MOTORS, 115200);

Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
Encoder encoder2(ENCODER_2_A, ENCODER_2_B);
Encoder encoder3(ENCODER_3_A, ENCODER_3_B);

USB Usb;
#ifdef XBOX_ALT_MODE
XBOXUSB Xbox(&Usb);
#else
XBOXRECV Xbox(&Usb);
#endif

int motorFrontLeft = 0, motorFrontRight = 0, motorRearLeft = 0, motorRearRight = 0;

int prevMotorFrontLeft = 0, prevMotorFrontRight = 0, prevMotorRearLeft = 0, prevMotorRearRight = 0, requested_state;

const float wheelCircumference = (WHEEL_DIAMETER_MM * M_PI) / 10.0;
float position1 = 0, position2 = 0, position3 = 0;
float robotX = 0, robotY = 0, robotRotation = 0;

int odriveSpeeds[6] = {50, -50, 60, -60, 70, -70};
int odriveCurrent[2] = {0, 0};
bool odriveShooting = false;

float lastTime = 0;

void updateMotors(int xSpeed, int ySpeed, int turnSpeed);
bool isXboxConnected();
void updateEncoders();
void printStatus();
int mapJoystickInput(int input);
void runLinear(int direction);
void moveDribblingTo(int position);

void setup()
{
  Serial.begin(115200);
  loggingSerial.begin(115200);
  Wire.begin();

#if !defined(_MIPSEL_)
  while (!Serial)
  {
    delay(10);
  }
#endif

  if (Usb.Init() == -1)
  {
    Serial.println(F("USB Host Shield initialization failed"));
    while (1)
    {
      delay(1000);
    }
  }
  pinMode(24, OUTPUT);
  digitalWrite(24, LOW);

  pinMode(linearPWM, OUTPUT);
  pinMode(linearDIR, OUTPUT);
  runLinear(0);

  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive_serial.begin(115200);
  if (!odrive.run_state(0, requested_state, false))
    return;
  if (!odrive.run_state(1, requested_state, false))
    return;

  Serial.println(F("Xbox Wireless Receiver Library Started"));
  updateMotors(0, 0, 0);
  Serial.println(F("Setup complete"));
}

void loop()
{
  Usb.Task();

  if (isXboxConnected())
  {
#ifdef XBOX_ALT_MODE
    int leftHatY = Xbox.getAnalogHat(LeftHatY);
    int leftHatX = Xbox.getAnalogHat(LeftHatX);
    int rightHatX = Xbox.getAnalogHat(RightHatX);
#else
    int leftHatY = Xbox.getAnalogHat(LeftHatY, CONTROLLER_ID);
    int leftHatX = Xbox.getAnalogHat(LeftHatX, CONTROLLER_ID);
    int rightHatX = Xbox.getAnalogHat(RightHatX, CONTROLLER_ID);
#endif

    int xSpeed = mapJoystickInput(leftHatX);
    int ySpeed = mapJoystickInput(leftHatY);
    int turnSpeed = mapJoystickInput(rightHatX);

    if (Xbox.getButtonPress(UP))
    {
      runLinear(1);
    }
    else if (Xbox.getButtonPress(DOWN))
    {
      runLinear(-1);
    }
    else
    {
      runLinear(0);
    }

    if (Xbox.getButtonPress(LEFT))
    {
      moveDribblingTo(-100);
    }
    else if (Xbox.getButtonPress(RIGHT))
    {
      moveDribblingTo(100);
    }
    else
    {
      moveDribblingTo(0);
    }

    if (Xbox.getButtonClick(X))
    {
      if (odriveShooting)
      {
        odrive.SetVelocity(0, 0);
        odrive.SetVelocity(1, 0);
        odriveShooting = false;
      }
      else
      {
        odrive.SetVelocity(0, odriveSpeeds[0]);
        odrive.SetVelocity(1, odriveSpeeds[1]);
        odriveShooting = true;
      }
    }

    if (Xbox.getButtonClick(Y))
    {
      if (odriveShooting)
      {
        odrive.SetVelocity(0, 0);
        odrive.SetVelocity(1, 0);
        odriveShooting = false;
      }
      else
      {
        odrive.SetVelocity(0, odriveSpeeds[2]);
        odrive.SetVelocity(1, odriveSpeeds[3]);
        odriveShooting = true;
      }
    }

    if (Xbox.getButtonClick(B))
    {
      if (odriveShooting)
      {
        odrive.SetVelocity(0, 0);
        odrive.SetVelocity(1, 0);
        odriveShooting = false;
      }
      else
      {
        odrive.SetVelocity(0, odriveSpeeds[4]);
        odrive.SetVelocity(1, odriveSpeeds[5]);
        odriveShooting = true;
      }
    }

    if (Xbox.getButtonClick(A))
    {
      updateMotors(0, 0, 0);
      runLinear(0);
      moveDribblingTo(0);
      for (int i = 0; i <= 1; i++)
      {
        requested_state = AXIS_STATE_MOTOR_CALIBRATION;
        if (!odrive.run_state(i, requested_state, true))
          return;

        requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
        if (!odrive.run_state(i, requested_state, true, 25.0f))
          return;

        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        if (!odrive.run_state(i, requested_state, false))
          return;
      }
    }

    updateMotors(xSpeed, ySpeed, turnSpeed * TURN_MULTIPLIER);
    updateEncoders();
    printStatus();
  }
  else
  {
    updateMotors(0, 0, 0);
    runLinear(0);
    moveDribblingTo(0);
  }
}

void updateMotors(int xSpeed, int ySpeed, int turnSpeed)
{
  motorFrontLeft = constrain(xSpeed - ySpeed + turnSpeed, -100, 100);
  motorFrontRight = constrain(xSpeed + ySpeed + turnSpeed, -100, 100);
  motorRearLeft = constrain(-xSpeed - ySpeed + turnSpeed, -100, 100);
  motorRearRight = constrain(-xSpeed + ySpeed + turnSpeed, -100, 100);

  frontMotors.control(motorFrontLeft, motorFrontRight);
  rearMotors.control(motorRearRight, motorRearLeft);
}

bool isXboxConnected()
{
#ifdef XBOX_ALT_MODE
  return Xbox.Xbox360Connected;
#else
  return (Xbox.XboxReceiverConnected && Xbox.Xbox360Connected[CONTROLLER_ID]);
#endif
}

int mapJoystickInput(int input)
{
  if (abs(input) < JOYSTICK_DEADZONE)
  {
    return 0;
  }

  if (input > 0)
  {
    return map(input, JOYSTICK_DEADZONE, 32767, MIN_SPEED, MAX_SPEED);
  }
  else
  {
    return map(input, -32768, -JOYSTICK_DEADZONE, -MAX_SPEED, -MIN_SPEED);
  }
}

void updateEncoders()
{
  long currentEncoder1 = encoder1.read();
  long currentEncoder2 = encoder2.read();
  long currentEncoder3 = encoder3.read();

  position1 = (currentEncoder1 / ENCODER_PPR) * wheelCircumference;
  position2 = -(currentEncoder2 / ENCODER_PPR) * wheelCircumference;
  position3 = -(currentEncoder3 / ENCODER_PPR) * wheelCircumference;
}

void printStatus()
{
  if (millis() - lastTime >= 100)
  {
    loggingSerial.print("<");
    loggingSerial.print(motorFrontLeft);
    loggingSerial.print(",");
    loggingSerial.print(motorFrontRight);
    loggingSerial.print(",");
    loggingSerial.print(motorRearLeft);
    loggingSerial.print(",");
    loggingSerial.print(motorRearRight);
    loggingSerial.print(",");
    loggingSerial.print(position1, 2);
    loggingSerial.print(",");
    loggingSerial.print(position2, 2);
    loggingSerial.print(",");
    loggingSerial.print(position3, 2);
    loggingSerial.print(",");
    loggingSerial.print(odriveShooting ? "1" : "0");
    loggingSerial.println(">");
    lastTime = millis();
  }

  Serial.print("Motors [FL:");
  Serial.print(motorFrontLeft);
  Serial.print(" FR:");
  Serial.print(motorFrontRight);
  Serial.print(" RL:");
  Serial.print(motorRearLeft);
  Serial.print(" RR:");
  Serial.print(motorRearRight);
  Serial.print("] Positions [1:");
  Serial.print(position1, 2);
  Serial.print(" 2:");
  Serial.print(position2, 2);
  Serial.print(" 3:");
  Serial.print(position3, 2);
  Serial.println(" cm]");
}

void runLinear(int direction)
{
  if (direction < 0)
  {
    digitalWrite(linearDIR, HIGH);
    analogWrite(linearPWM, 255);
  }
  else if (direction > 0)
  {
    digitalWrite(linearDIR, LOW);
    analogWrite(linearPWM, 255);
  }
  else
  {
    digitalWrite(linearDIR, LOW);
    analogWrite(linearPWM, 0);
  }
}

void moveDribblingTo(int position)
{
  if (position < 0)
  {
    dribblingMotors.control(-100, 0);
  }
  else if (position > 0)
  {
    dribblingMotors.control(100, 0);
  }
  else
  {
    dribblingMotors.control(0, 0);
  }
}