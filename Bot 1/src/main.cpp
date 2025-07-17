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

// Controller Definitions
#define CONTROLLER_ID 0
#define JOYSTICK_DEADZONE 200

// Joystick Definitions
#define MIN_SPEED 10
#define MAX_SPEED 50
#define TURN_MULTIPLIER 0.4

// Cytron Definitions
#define MDDS_FRONT_MOTORS 26
#define MDDS_REAR_MOTORS 25
#define DP_MOTORS 24
#define DRIBBLE_PISTON 27
#define LINEAR_PWM 4
#define LINEAR_DIR 29

// IR Pin Definitions
#define DRIBBLING_IR_1 A5
#define DRIBBLING_IR_2 A2
#define DRIBBLING_INTAKE_IR A4
#define PULLEY_IR A3

#define DRIBBLING_INTER_IR_DISTANCE 8.3f

// Encoder Pin Definitions
#define ENCODER_1_A 2
#define ENCODER_1_B 3
#define ENCODER_2_A 18
#define ENCODER_2_B 19
#define ENCODER_3_A 20
#define ENCODER_3_B 21

// Kill Switch Pulley
#define KILL_SWITCH_PULLEY 49

// Encoder Measurement Definitions (mm)
#define ENCODER_PPR 2400.0
#define WHEEL_CIRCUMFERENCE 0.1884
#define DISTANCE_FROM_CENTER 0.1315

HardwareSerial &odrive_serial = Serial2;
SoftwareSerial loggingSerial(23, 22);

ODriveArduino odrive(odrive_serial);

Cytron_SmartDriveDuo frontMotors(SERIAL_SIMPLIFIED, MDDS_FRONT_MOTORS, 115200);
Cytron_SmartDriveDuo rearMotors(SERIAL_SIMPLIFIED, MDDS_REAR_MOTORS, 115200);
Cytron_SmartDriveDuo dpMotors(SERIAL_SIMPLIFIED, DP_MOTORS, 115200);
Cytron_SmartDriveDuo dribblingPiston(SERIAL_SIMPLIFIED, DRIBBLE_PISTON, 115200);

Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
Encoder encoder2(ENCODER_2_A, ENCODER_2_B);
Encoder encoder3(ENCODER_3_A, ENCODER_3_B);

USB Usb;
#ifdef XBOX_ALT_MODE
XBOXUSB Xbox(&Usb);
#else
XBOXRECV Xbox(&Usb);
#endif

// Motor Variables
int motorFrontLeft = 0, motorFrontRight = 0, motorRearLeft = 0, motorRearRight = 0, motorDribble = 0, motorPulley = 0, motorLinear = 0;
bool dribbling = false, flapsOpen = false, intaking = false, maxPosReached = false;

// Timing Variables
unsigned long intakeTime, IR_1_time, IR_2_time, lastOdometryTime, lastTime;
long delayTime;

// State Variables
int ballState = 0, intakeState = 0, odriveState = 0, requested_state;

// Encoder Variables
float position1 = 0, position2 = 0, position3 = 0, prevPosition1 = 0, prevPosition2 = 0, prevPosition3 = 0, Vw_actual = 0.0, Vx_actual = 0.0, Vy_actual = 0.0;
int currentEncoder1 = 0, currentEncoder2 = 0, currentEncoder3 = 0, prevEncoder1 = 0, prevEncoder2 = 0, prevEncoder3 = 0;
float robotX = 0.0, robotY = 0.0, robotTheta = 0.0;

// Odrive Variables
int odrivePresetSpeeds[6] = {-50, 50, -60, 60, -70, 70}, odriveMotorError[2] = {0, 0}, odriveEncoderError[2] = {0, 0}, odriveControllerError[2] = {0, 0};
float odriveVbusVoltage = 0.0, odriveCurrent[2] = {0.0, 0.0}, odriveVelocity[2] = {0.0, 0.0};
bool odriveShooting = false;
int lastOdriveSpeed = 0;

// Logging Variables
float ballSpeed;

// Function Prototypes
void updateWheelDrive(int xSpeed, int ySpeed, int turnSpeed);
void updateMotors();
void toggleFlaps();
void dribble();
void intake();
void odriveShoot(int speed1, int speed2);

bool isXboxConnected();
int mapJoystickInput(int input);

void updateEncoders();
float normalizeAngle(float angle);
void calculateOdometry();
void printStatus();

// Setup
void setup()
{
  // Serial Setup
  Serial.begin(115200);
  loggingSerial.begin(115200);
  Wire.begin();

  // IR Sensors Setup
  pinMode(DRIBBLING_IR_1, INPUT);
  pinMode(DRIBBLING_IR_2, INPUT);
  pinMode(DRIBBLING_INTAKE_IR, INPUT);
  pinMode(PULLEY_IR, INPUT);

  // Linear Actuator Setup
  pinMode(LINEAR_PWM, OUTPUT);
  pinMode(LINEAR_DIR, OUTPUT);

  // Kill Switch Setup
  pinMode(KILL_SWITCH_PULLEY, INPUT_PULLUP);

  // Stop All Motors
  updateWheelDrive(0, 0, 0);
  updateMotors();

  // XBOX Startup
#if !defined(_MIPSEL_)
  while (!Serial)
  {
    delay(10);
  }
#endif

  if (Usb.Init() == -1)
  {
    Serial.println(F("USB Host Shield Initialization Failed"));
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println(F("Xbox Wireless Receiver Library Started"));

  // Odrive Setup
  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive_serial.begin(115200);
  if (!odrive.run_state(0, requested_state, false))
    return;
  if (!odrive.run_state(1, requested_state, false))
    return;

  Serial.println(F("Setup complete"));
}

// Loop
void loop()
{
  Usb.Task();

  if (isXboxConnected())
  {
    // Joystick Controls
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

    // Linear Controls
    if (Xbox.getButtonPress(UP))
    {
      motorLinear = 255;
    }
    else if (Xbox.getButtonPress(DOWN))
    {
      motorLinear = -255;
    }
    else
    {
      motorLinear = 0;
    }

    // Dribble Motor Controls
    if (Xbox.getButtonPress(LEFT) && !intaking)
    {
      if (motorDribble > -50)
      {
        motorDribble -= 3;
      }
      else
      {
        motorDribble = -50;
      }
    }
    else if (Xbox.getButtonPress(RIGHT) && !intaking)
    {
      if (motorDribble < 50)
      {
        motorDribble += 3;
      }
      else
      {
        motorDribble = 50;
      }
    }
    else if (!intaking)
    {
      if (motorDribble > 0)
      {
        motorDribble -= 3;
      }
      else if (motorDribble < 0)
      {
        motorDribble += 3;
      }
      else
      {
        motorDribble = 0;
      }
    }

    // Pulley Controls
    if (Xbox.getButtonPress(RT) > 0)
    {
      motorPulley = map(Xbox.getButtonPress(RT), 0, 255, 10, 100);
    }
    else if (Xbox.getButtonPress(LT) > 0)
    {
      motorPulley = map(Xbox.getButtonPress(LT), 0, 255, 10, -100);
    }
    else if (!intaking && !odriveShooting)
    {
      motorPulley = 0;
    }

    // Dribbling Controls
    if ((Xbox.getButtonClick(L1) && !flapsOpen) || dribbling)
    {
      dribbling = true;
      dribble();
    }

    if (Xbox.getButtonClick(R1) && !dribbling)
    {
      toggleFlaps();
    }

    if (Xbox.getButtonClick(A) || intaking)
    {
      intaking = true;
      intake();
    }

    // ODrive Controls
    if (Xbox.getButtonClick(X) || (odriveShooting && lastOdriveSpeed == 1))
    {
      lastOdriveSpeed = 1;
      odriveShooting = true;
      odriveShoot(odrivePresetSpeeds[0], odrivePresetSpeeds[1]);
    }

    if (Xbox.getButtonClick(Y) || (odriveShooting && lastOdriveSpeed == 2))
    {
      lastOdriveSpeed = 2;
      odriveShooting = true;
      odriveShoot(odrivePresetSpeeds[2], odrivePresetSpeeds[3]);
    }

    if (Xbox.getButtonClick(B) || (odriveShooting && lastOdriveSpeed == 3))
    {
      lastOdriveSpeed = 3;
      odriveShooting = true;
      odriveShoot(odrivePresetSpeeds[4], odrivePresetSpeeds[5]);
    }

    updateWheelDrive(xSpeed, ySpeed, turnSpeed * TURN_MULTIPLIER);
    updateMotors();
    updateEncoders();
    printStatus();
  }
  else
  {
    updateWheelDrive(0, 0, 0);
    motorDribble = 0;
    motorPulley = 0;
    motorLinear = 0;
    updateMotors();
  }
}

void updateWheelDrive(int xSpeed, int ySpeed, int turnSpeed)
{
  motorFrontLeft = constrain(xSpeed - ySpeed + turnSpeed, -100, 100);
  motorFrontRight = constrain(xSpeed + ySpeed + turnSpeed, -100, 100);
  motorRearLeft = constrain(-xSpeed - ySpeed + turnSpeed, -100, 100);
  motorRearRight = constrain(-xSpeed + ySpeed + turnSpeed, -100, 100);

  frontMotors.control(motorFrontLeft, motorFrontRight);
  rearMotors.control(motorRearRight, motorRearLeft);
}

void updateMotors()
{
  digitalWrite(LINEAR_DIR, motorLinear < 0);
  analogWrite(LINEAR_PWM, abs(motorLinear));

  if (digitalRead(KILL_SWITCH_PULLEY) == LOW && motorPulley > 0)
  {
    motorPulley = 0;
  }

  dpMotors.control(motorDribble, motorPulley);
}

void toggleFlaps()
{
  if (!flapsOpen)
  {
    dribblingPiston.control(100, 0);
    flapsOpen = true;
  }
  else
  {
    dribblingPiston.control(0, 0);
    flapsOpen = false;
  }
}

void dribble()
{
  if (ballState == 0)
  {
    dribblingPiston.control(100, 100);
    ballState = 1;
    flapsOpen = true;
  }
  else if (ballState == 1 && !digitalRead(DRIBBLING_IR_2))
  {
    dribblingPiston.control(100, 0);
    ballState = 2;
  }
  else if (ballState == 2 && digitalRead(DRIBBLING_IR_2))
  {
    ballState = 3;
  }
  else if (ballState == 3 && !digitalRead(DRIBBLING_IR_2))
  {
    IR_2_time = millis();
    ballState = 4;
  }
  else if (ballState == 4 && !digitalRead(DRIBBLING_IR_1))
  {
    IR_1_time = millis();
    ballSpeed = (DRIBBLING_INTER_IR_DISTANCE / ((IR_1_time - IR_2_time) / 1000.0));
    // delayTime = map(ballSpeed, 200, 500, 100, -250);
    delayTime = 0;
    ballState = 5;
  }
  else if (ballState == 5 && (static_cast<long>(millis() - IR_1_time) >= delayTime))
  {
    dribblingPiston.control(0, 0);
    ballState = 0;
    flapsOpen = false;
    dribbling = false;
  }
}

void intake()
{
  if (intakeState == 0)
  {
    if (flapsOpen)
    {
      toggleFlaps();
    }

    if (motorDribble <= 50)
    {
      motorDribble += 2;
      updateMotors();
      return;
    }

    intakeState = 1;
  }
  else if (intakeState == 1)
  {
    if (!digitalRead(DRIBBLING_INTAKE_IR) || maxPosReached)
    {
      maxPosReached = true;
      if (motorDribble >= 0)
      {
        motorDribble -= 4;
        updateMotors();
        return;
      }
      maxPosReached = false;
      intakeState = 2;
    }
  }
  else if (intakeState == 2)
  {
    if (!flapsOpen)
    {
      toggleFlaps();
    }
    intakeTime = millis();
    intakeState = 3;
  }
  else if (intakeState == 3)
  {
    if (millis() - intakeTime >= 1500)
    {
      if (flapsOpen)
      {
        toggleFlaps();
      }
      intakeState = 4;
      intakeTime = millis();
    }
  }
  else if (intakeState == 4)
  {
    if (millis() - intakeTime <= 700)
    {
      if (motorDribble >= -50)
      {
        motorDribble -= 2;
        updateMotors();
        return;
      }
      return;
    }
    intakeState = 5;
  }
  else if (intakeState == 5)
  {
    if (motorDribble != 0)
    {
      if (motorDribble > 0)
      {
        motorDribble -= 2;
      }
      else
      {
        motorDribble += 2;
      }
      updateMotors();
      return;
    }
    if (motorPulley < 100)
    {
      motorPulley += 10;
      updateMotors();
      return;
    }
    intakeState = 6;
  }
  else if (intakeState == 6 && !digitalRead(PULLEY_IR))
  {
    if (motorPulley > 0)
    {
      motorPulley -= 50;
      updateMotors();
      return;
    }
    intakeState = 0;
    intaking = false;
  }
}

void odriveShoot(int speed1, int speed2)
{
  Serial.print("Odrive State: ");
  Serial.print(odriveState);
  Serial.print("  Odrive 0 Velocity: ");
  Serial.print(odriveVelocity[0]);
  Serial.print("  Odrive 1 Velocity: ");
  Serial.println(odriveVelocity[1]);

  if (odriveState == 0)
  {
    odrive.SetVelocity(0, speed1);
    odrive.SetVelocity(1, speed2);
    odriveState = 1;
  }
  else if (odriveState == 1 && (odriveVelocity[0] >= speed1 - 5) && (odriveVelocity[1] >= speed2 - 5))
  {
    if (motorPulley < 50)
    {
      motorPulley += 10;
      updateMotors();
      return;
    }
    odriveState = 2;
  }
  else if (odriveState == 2 && digitalRead(PULLEY_IR))
  {
    odrive.SetVelocity(0, 0);
    odrive.SetVelocity(1, 0);
    odriveState = 3;
  }
  else if (odriveState == 3)
  {
    if (motorPulley > -100)
    {
      motorPulley -= 20;
      updateMotors();
      return;
    }
    odriveState = 4;
  }
  else if (odriveState == 4)
  {
    if (motorPulley < 0)
    {
      motorPulley += 20;
      updateMotors();
      return;
    }
    odriveState = 0;
    odriveShooting = false;
    lastOdriveSpeed = 0;
  }
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
  currentEncoder1 = encoder1.read();
  currentEncoder2 = encoder2.read();
  currentEncoder3 = encoder3.read();

  prevPosition1 = position1;
  prevPosition2 = position2;
  prevPosition3 = position3;

  position1 = (currentEncoder1 / ENCODER_PPR) * WHEEL_CIRCUMFERENCE;  // X Axis Front
  position2 = -(currentEncoder2 / ENCODER_PPR) * WHEEL_CIRCUMFERENCE; // Y Axis
  position3 = -(currentEncoder3 / ENCODER_PPR) * WHEEL_CIRCUMFERENCE; // X Axis Back

  calculateOdometry();
}

float normalizeAngle(float angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}

void calculateOdometry()
{
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastOdometryTime) / 1000.0;

  if (deltaTime <= 0)
  {
    return;
  }

  float delta1 = position1 - prevPosition1;
  float delta2 = position2 - prevPosition2;
  float delta3 = position3 - prevPosition3;

  float delta_theta_raw = (delta1 - delta3) / (2.0 * DISTANCE_FROM_CENTER);
  Vw_actual = delta_theta_raw / deltaTime;

  float delta_x_robot = (delta1 + delta3) / 2.0;
  Vx_actual = delta_x_robot / deltaTime;

  float delta_y_robot = delta2;
  Vy_actual = delta_y_robot / deltaTime;

  robotTheta += delta_theta_raw;
  robotTheta = normalizeAngle(robotTheta);

  float delta_X_global = (delta_x_robot * cos(robotTheta)) - (delta_y_robot * sin(robotTheta));
  float delta_Y_global = (delta_x_robot * sin(robotTheta)) + (delta_y_robot * cos(robotTheta));

  robotX += delta_X_global;
  robotY += delta_Y_global;

  // Serial.print("Odometry: ");
  // Serial.print("Vw: ");
  // Serial.print(Vw_actual, 2); // Angular velocity in radians/second
  // Serial.print(" rad/s, Vx: ");
  // Serial.print(Vx_actual, 2); // Local X velocity in meters/second
  // Serial.print(" m/s, Vy: ");
  // Serial.print(Vy_actual, 2); // Local Y velocity in meters/second
  // Serial.print(" m/s, Xglobal: ");
  // Serial.print(robotX, 2); // Accumulated global X position in meters
  // Serial.print(" m, Yglobal: ");
  // Serial.print(robotY, 2); // Accumulated global Y position in meters
  // Serial.print(" m, ThetaGlobal: ");
  // Serial.print(robotTheta * 180.0 / M_PI, 2); // Accumulated global orientation in degrees
  // Serial.println("°");

  prevPosition1 = position1;
  prevPosition2 = position2;
  prevPosition3 = position3;
  lastOdometryTime = currentTime;
}

void printStatus()
{
  odrive_serial.print("r vbus_voltage\n");
  odriveVbusVoltage = odrive.readFloat();
  if (odriveVbusVoltage > 15)
  {
    odrive_serial.print("r axis0.encoder.vel_estimate\n");
    odriveVelocity[0] = odrive.readFloat();
    odrive_serial.print("r axis1.encoder.vel_estimate\n");
    odriveVelocity[1] = odrive.readFloat();
    odrive_serial.print("r axis0.motor.current_control.Iq_measured\n");
    odriveCurrent[0] = odrive.readFloat();
    odrive_serial.print("r axis1.motor.current_control.Iq_measured\n");
    odriveCurrent[1] = odrive.readFloat();
    odrive_serial.print("r axis0.motor.error\n");
    odriveMotorError[0] = odrive.readInt();
    odrive_serial.print("r axis1.motor.error\n");
    odriveMotorError[1] = odrive.readInt();
    odrive_serial.print("r axis0.encoder.error\n");
    odriveEncoderError[0] = odrive.readInt();
    odrive_serial.print("r axis1.encoder.error\n");
    odriveEncoderError[1] = odrive.readInt();
    odrive_serial.print("r axis0.controller.error\n");
    odriveControllerError[0] = odrive.readInt();
    odrive_serial.print("r axis1.controller.error\n");
    odriveControllerError[1] = odrive.readInt();
  }
  else
  {
    odriveVelocity[0] = 0.0;
    odriveVelocity[1] = 0.0;
    odriveCurrent[0] = 0.0;
    odriveCurrent[1] = 0.0;
    if (odriveMotorError[0] == 0 && odriveMotorError[1] == 0 &&
        odriveEncoderError[0] == 0 && odriveEncoderError[1] == 0 &&
        odriveControllerError[0] == 0 && odriveControllerError[1] == 0)
    {
      odriveMotorError[0] = -1;
      odriveMotorError[1] = -1;
      odriveEncoderError[0] = -1;
      odriveEncoderError[1] = -1;
      odriveControllerError[0] = -1;
      odriveControllerError[1] = -1;
    }
  }

  // if (millis() - lastTime >= 100)
  //{
  //   loggingSerial.print("<");
  //   loggingSerial.print(motorFrontLeft);
  //   loggingSerial.print(",");
  //   loggingSerial.print(motorFrontRight);
  //   loggingSerial.print(",");
  //   loggingSerial.print(motorRearLeft);
  //   loggingSerial.print(",");
  //   loggingSerial.print(motorRearRight);
  //   loggingSerial.print(",");
  //   loggingSerial.print(Vw_actual, 2);
  //   loggingSerial.print(",");
  //   loggingSerial.print(Vx_actual, 2);
  //   loggingSerial.print(",");
  //   loggingSerial.print(Vy_actual, 2);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveShooting ? "1" : "0");
  //   loggingSerial.print(",");
  //   loggingSerial.print(ballState);
  //   loggingSerial.print(",");
  //   loggingSerial.print(flapsOpen ? "1" : "0");
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveVbusVoltage, 1);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveVelocity[0], 1);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveVelocity[1], 1);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveCurrent[0], 1);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveCurrent[1], 1);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveMotorError[0]);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveMotorError[1]);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveEncoderError[0]);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveEncoderError[1]);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveControllerError[0]);
  //   loggingSerial.print(",");
  //   loggingSerial.print(odriveControllerError[1]);
  //   loggingSerial.println(">");
  //   lastTime = millis();
  // }

  // Serial.print("  Motors [FL:");
  // Serial.print(motorFrontLeft);
  // Serial.print(" FR:");
  // Serial.print(motorFrontRight);
  // Serial.print(" RL:");
  // Serial.print(motorRearLeft);
  // Serial.print(" RR:");
  // Serial.print(motorRearRight);
  // Serial.print("] Positions [1:");
  // Serial.print(position1, 2);
  // Serial.print(" 2:");
  // Serial.print(position2, 2);
  // Serial.print(" 3:");
  // Serial.print(position3, 2);
  // Serial.print("°]");
  // Serial.print(" Odrive Shooting: ");
  // Serial.print(odriveShooting);
  // Serial.print(" Ball State: ");
  // Serial.print(ballState);
  // Serial.print(" Flaps: ");
  // Serial.print(flapsOpen);
  // Serial.print(" Odrive Vbus Voltage: ");
  // Serial.print(odriveVbusVoltage, 1);
  // Serial.print(" V, Velocities [0:");
  // Serial.print(odriveVelocity[0], 1);
  // Serial.print(" 1:");
  // Serial.print(odriveVelocity[1], 1);
  // Serial.print("], Currents [0:");
  // Serial.print(odriveCurrent[0], 1);
  // Serial.print(" 1:");
  // Serial.print(odriveCurrent[1], 1);
  // Serial.println(" A");
}