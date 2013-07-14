#ifndef QUADCONTROLLER_H_
#define QUADCONTROLLER_H_

#include <Arduino.h>
#include <Servo.h>
#include "MPU6050Lib.h"
#include "PID_v1.h"

#define QUAD_DEBUG


class QuadController
{
public:
  // Constructor
  QuadController();

  boolean init(int pin1, int pin2, int pin3, int pin4, int mpuVinPin);

  // void yaw(float yawRate);
  void roll(int rollRate);
  void pitch(int pitchRate);

  void setSpeed(int speed);
  void setSpeed();
  void kill();
  void enable();
  void hover();

  void loop();

private:
  float m_a[4];
  int m_powerOffset[4];
  int m_speed;
  boolean m_enabled;
  boolean m_hover;
  MPU6050Lib m_mpu;
  Servo m_esc[4];
  PID m_pitchPid;
  PID m_rollPid;
  double m_pitchInput, m_pitchOutput, m_pitchSetpoint, m_rollInput, m_rollOutput, m_rollSetpoint;
  double Kp, Ki, Kd;

  void arm();
  boolean duePoll();

};

#endif