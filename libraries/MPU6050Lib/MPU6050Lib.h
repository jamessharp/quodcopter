#ifndef _MPU9150LIB_H_
#define _MPU9150LIB_H_

#include <Arduino.h>

#define MPULIB_DEBUG

class MPU6050Lib
{
public:

    // Constructor
    MPU6050Lib();

    boolean init(int mpuRate);
    boolean init(int mpuRate, int vinPin);
    boolean read();

    // Calibrate
    boolean calibrate();

    void calcRotatedVector(float *inVector, float *outVector);

    //  these functions can be used to display quaternions and vectors
  
    void printQuaternion(long *quaternion);
    void printQuaternion(float *quaternion);
    void printVector(short *vec);
    void printVector(float *vec);
    void printVector(long *vec);
    void printAngles(float *vec);

    //  these variables are the values from the MPU

    long m_rawQuaternion[4];                                  // the quaternion output from the DMP
    short m_rawGyro[3];                                       // calibrated gyro output from the sensor
    short m_rawAccel[3];                                      // raw accel data

    //  these variables are processed results

    float m_dmpQuaternion[4];                                 // float and normalized version of the dmp quaternion
    float m_dmpEulerPose[3];                                  // Euler angles from the DMP quaternion
    short m_calAccel[3];                                      // calibrated and scaled accel data

    // these variables are the fused results

    float m_fusedEulerPose[3];                                // the fused Euler angles
    float m_fusedQuaternion[4];                               // the fused quaternion

};

#endif