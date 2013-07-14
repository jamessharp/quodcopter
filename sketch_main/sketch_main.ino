#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpmap.h"
#include "dmpKey.h"
#include <MPU6050Lib.h>
#include <QuadController.h>
#include "aJSON.h"
#include "PID_v1.h"

MPU6050Lib mpu;
QuadController quad;

int ESC_PIN_1 = 13;
int ESC_PIN_2 = 12;
int ESC_PIN_3 = 11;
int ESC_PIN_4 = 10;
int MPU_VIN_PIN = 8;

aJsonStream serial_stream(&Serial);

long lastHeartbeat;
long heartbeatTolerance = 1000;  // If no heartbeat for this time then kill the motors

boolean dead = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Starting up...");
  Wire.begin();

  quad.init(ESC_PIN_1, ESC_PIN_2, ESC_PIN_3, ESC_PIN_4, MPU_VIN_PIN);
  Serial.println("QuadCopterLoaded");
}

void loop() {

  aJsonObject *msg;
  
  if (serial_stream.available()) 
  {
    // Skip any accidental whitespace
    serial_stream.skip();
  }

  if (serial_stream.available())
  {
    msg = aJson.parse(&serial_stream);
    processMessage(msg);
    aJson.deleteItem(msg);
  }

  checkHeartbeat();

  quad.loop();
}

void checkHeartbeat()
{
  if ((millis() - lastHeartbeat) > heartbeatTolerance)
  {
    // Arrrrrr kill the motors we're dead
    quad.kill();

    if (!dead)
    {
      Serial.println("Lost heartbeat!");  
    }

    dead = true;
  }
}

void heartbeat()
{
  lastHeartbeat = millis();

  if (dead)
  {
    Serial.println("Got heartbeat");
    quad.enable();
  }

  dead = false;
}

void processMessage(aJsonObject *msg)
{
  String cmdVal;
  aJsonObject *val = aJson.getObjectItem(msg, "val");
  aJsonObject *cmd = aJson.getObjectItem(msg, "cmd");

  if(!cmd)
  {
    Serial.println("no command!");
    // aJson.deleteItem(val);
    // aJson.deleteItem(cmd);
    return;
  }

  cmdVal = String(cmd->valuestring);

  if(cmdVal.equals("heartbeat"))
  {
    heartbeat();
  }
  else if (cmdVal.equals("force"))
  {
    if (val)
    {
      quad.setSpeed(val->valueint);
    }
  }
  else if (cmdVal.equals("hover"))
  {
    quad.hover();
  }

  // aJson.deleteItem(val);
  // aJson.deleteItem(cmd);
}