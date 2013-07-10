#include <Wire.h>
#include "I2Cdev.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <MPU6050Lib.h>

// The rate in Hz at which the MPU updates the sensor data and DMP output
#define MPU_UPDATE_RATE (10)

MPU6050Lib mpu;

long lastPollTime;   // This is the last time we checked the MPU
long pollInterval;   // How long between checks (so we don't thrash the I2C bus)

boolean duePoll()
{
    if ((millis() - lastPollTime) < pollInterval)
        return false;
    if (mpu.read())
    {
        lastPollTime = millis();
        return true;
    }
    return false;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Starting up...");
  Wire.begin();
  mpu.init(MPU_UPDATE_RATE);

  pollInterval = (1000 / MPU_UPDATE_RATE) - 1;
  lastPollTime = millis();
}

void loop() {
    if (duePoll())
    {
        mpu.printQuaternion(mpu.m_rawQuaternion);Serial.print("\n");
    } 
}