#include "QuadController.h"
#include "MPU6050Lib.h"

#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000
#define MPU_UPDATE_RATE (10)

#define VECX 1  // Swap x and y it seems
#define VECY 0
#define VECZ 2

long lastPollTime;   // This is the last time we checked the MPU
long pollInterval;   // How long between checks (so we don't thrash the I2C bus)

float zAxis[3] = {0, 0, 1};


// Pin config
//               2
//               |
//   3 --------------------- 4
//               |
//               1

QuadController::QuadController()
{
    m_a = {1, 1, 1, 1};
    m_powerOffset = {0, 0, 0, 0};
    m_enabled = false;
    Kp = 500;
    Ki = 0.05; 
    Kd = 0.25;
    m_pitchPid.init(&m_pitchInput, &m_pitchOutput, &m_pitchSetpoint, Kp, Ki, Kd, DIRECT);
    m_rollPid.init(&m_rollInput, &m_rollOutput, &m_rollSetpoint, Kp, Ki, Kd, DIRECT);
    m_rollSetpoint = 0;
    m_pitchSetpoint = 0;
    m_hover = false;
}

boolean QuadController::init(int pin1, int pin2, int pin3, int pin4, int mpuVinPin)
{
    m_esc[0].attach(pin1);
    m_esc[1].attach(pin2);
    m_esc[2].attach(pin3);
    m_esc[3].attach(pin4);

    arm();

    m_mpu.init(MPU_UPDATE_RATE, mpuVinPin);
    pollInterval = (1000 / MPU_UPDATE_RATE) - 1;
    lastPollTime = millis();

    m_pitchPid.SetMode(AUTOMATIC);
    m_rollPid.SetMode(AUTOMATIC);

    m_mpu.calibrate();

    return true;
}

void QuadController::pitch(int pitchRate)
{
    m_powerOffset[0] = pitchRate;
    m_powerOffset[1] = -1 * pitchRate;
    Serial.print("Setting pitch: "); Serial.println(pitchRate);
    setSpeed();
}

void QuadController::roll(int rollRate)
{
    m_powerOffset[2] = -1 * rollRate;
    m_powerOffset[3] = rollRate;
    Serial.print("Setting roll: "); Serial.println(rollRate);
    setSpeed();
}

void QuadController::setSpeed(int speed)
{
    m_speed = speed;
    setSpeed();
}

void QuadController::setSpeed()
{
    if (m_enabled)
    {
#ifdef QUAD_DEBUG
        Serial.print("Setting speed: "); Serial.println(m_speed);
#endif
        for (int ii = 0; ii < 4; ii++)
        {
            m_esc[ii].writeMicroseconds(m_a[ii] * m_speed + m_powerOffset[ii]);
        }
    }
}

void QuadController::kill()
{
    setSpeed(MIN_PULSE_WIDTH);
    m_enabled = false;
}

void QuadController::enable()
{
    m_enabled = true;    
}

void QuadController::arm()
{
    setSpeed(MIN_PULSE_WIDTH);
}

void QuadController::hover()
{

    m_hover = !m_hover;
}

boolean QuadController::duePoll()
{
    if ((millis() - lastPollTime) < pollInterval)
        return false;
    if (m_mpu.read())
    {
        lastPollTime = millis();
        return true;
    }
    return false;
}

void QuadController::loop()
{
    if (duePoll())
    {
        // m_mpu.printQuaternion(m_mpu.m_dmpQuaternion);Serial.print("\n");
        float rotatedZ[3];
        m_mpu.calcRotatedVector(zAxis, rotatedZ);

        m_pitchInput = (double)rotatedZ[VECX];
        m_rollInput = (double)rotatedZ[VECY];

        m_pitchPid.Compute();
        m_rollPid.Compute();

        if (m_hover)
        {
            pitch((int)m_pitchOutput);
            roll((int)m_rollOutput);
        }

        m_mpu.printVector(rotatedZ);Serial.print('\n');
    } 
}