#include "MPU6050Lib.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPUQuaternion.h"

static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    /*

       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}

static signed char gyro_orientation[9]= {1, 0, 0,
                                         0, 1, 0, 
                                         0, 0, 1};

MPU6050Lib::MPU6050Lib() {}

boolean MPU6050Lib::init(int mpuRate)
{
    struct int_param_s int_param;
    int result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;

#ifdef MPULIB_DEBUG
    Serial.println("started initializing MPU6050 chip");
#endif

    mpu_init_structures();

    int_param.cb = NULL;
    int_param.pin = 0;
    int_param.lp_exit = 0;
    int_param.active_low = 1;
    result = mpu_init(&int_param);

    if (result != 0)
    {
#ifdef MPULIB_DEBUG
        Serial.print("mpu_init failed with code: "); Serial.println(result);
#endif
        return false;
    }

    // Set up the sensors
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);

#ifdef MPULIB_DEBUG
    Serial.println("Loading firmware");
#endif

    if ((result = dmp_load_motion_driver_firmware()) != 0)
    {
#ifdef MPULIB_DEBUG
        Serial.print("Failed to load dmp firmware: "); Serial.println(result);
#endif
        return false;
    }

    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));

    // Enable features
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);

    dmp_set_fifo_rate(mpuRate);

    if (mpu_set_dmp_state(1) != 0)
    {
#ifdef MPULIB_DEBUG
        Serial.println("mpu_set_dmp_state failed");
#endif
        return false;
    }

    mpu_set_sample_rate(mpuRate);

#ifdef MPULIB_DEBUG
    Serial.println("finished initializing MPU6050 chip");
#endif 

    return true;
}

boolean MPU6050Lib::read()
{
    short intStatus;
    int result;
    short sensors;
    unsigned char more;
    unsigned long timestamp;

    mpu_get_int_status(&intStatus);                       // get the current MPU state
    if ((intStatus & (MPU_INT_STATUS_DMP | MPU_INT_STATUS_DMP_0))
            != (MPU_INT_STATUS_DMP | MPU_INT_STATUS_DMP_0))
    {
#ifdef MPULIB_DEBUG
        Serial.print("Unexpected internal status: "); Serial.println(intStatus);
#endif
        return false;
    }

    // Read the data from the fifo queue
    if ((result = dmp_read_fifo(m_rawGyro, m_rawAccel, m_rawQuaternion, &timestamp, &sensors, &more)) != 0) 
    {
#ifdef MPULIB_DEBUG
        Serial.print("Unexpected read from fifo: "); Serial.println(result);
#endif
        return false;
    }

     // got the raw data - now process
    
    m_dmpQuaternion[QUAT_W] = (float)m_rawQuaternion[QUAT_W];  // get float version of quaternion
    m_dmpQuaternion[QUAT_X] = (float)m_rawQuaternion[QUAT_X];
    m_dmpQuaternion[QUAT_Y] = (float)m_rawQuaternion[QUAT_Y];
    m_dmpQuaternion[QUAT_Z] = (float)m_rawQuaternion[QUAT_Z];
    MPUQuaternionNormalize(m_dmpQuaternion);                 // and normalize
    
    MPUQuaternionQuaternionToEuler(m_dmpQuaternion, m_dmpEulerPose);

    return true;

}

void MPU6050Lib::printQuaternion(long *quat)
{
    Serial.print("w: "); Serial.print(quat[QUAT_W]);  
    Serial.print(" x: "); Serial.print(quat[QUAT_X]);  
    Serial.print(" y: "); Serial.print(quat[QUAT_Y]);  
    Serial.print(" z: "); Serial.print(quat[QUAT_Z]);  
}

void MPU6050Lib::printQuaternion(float *quat)
{
    Serial.print("w: "); Serial.print(quat[QUAT_W]);  
    Serial.print(" x: "); Serial.print(quat[QUAT_X]);  
    Serial.print(" y: "); Serial.print(quat[QUAT_Y]);  
    Serial.print(" z: "); Serial.print(quat[QUAT_Z]);  
}

void MPU6050Lib::printVector(short *vec)
{
    Serial.print("x: "); Serial.print(vec[VEC3_X]);  
    Serial.print(" y: "); Serial.print(vec[VEC3_Y]);  
    Serial.print(" z: "); Serial.print(vec[VEC3_Z]);    
}

void MPU6050Lib::printVector(float *vec)
{
    Serial.print("x: "); Serial.print(vec[VEC3_X]);  
    Serial.print(" y: "); Serial.print(vec[VEC3_Y]);  
    Serial.print(" z: "); Serial.print(vec[VEC3_Z]);    
}

void MPU6050Lib::printAngles(float *vec)
{
    Serial.print("x: "); Serial.print(vec[VEC3_X] * RAD_TO_DEGREE);  
    Serial.print(" y: "); Serial.print(vec[VEC3_Y] * RAD_TO_DEGREE);  
    Serial.print(" z: "); Serial.print(vec[VEC3_Z] * RAD_TO_DEGREE);    
}
