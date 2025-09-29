#include "MahonyAHRS.h"
#include <math.h>
//#include "arm_math.h"

AHRS_t AHRS = {0};
QuaternionBuf_t QuaternionBuffer;

float twoKp = twoKpDef; // 2 * proportional gain (Kp)
float Gravity = 9.78;
uint8_t debugPlot = 0;

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                 // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

static float invSqrt(float x);
static float TuningKp(float *gyro, float *acc, float *kp);

	
/**
  * @brief          AHRS algorithm init
  * @param[1-3]     accelerometer measurement in m/s2
  */
void Quaternion_AHRS_InitIMU(float ax, float ay, float az, float ref_gNorm)
{
    float Pitch, Roll;

    Gravity = ref_gNorm;

    Pitch = atan2f(ay, az);
    Roll = -atan2f(ax, az);
    q0 = cosf(Pitch / 2) * cosf(Roll / 2);
    q1 = sinf(Pitch / 2) * cosf(Roll / 2);
    q2 = cosf(Pitch / 2) * sinf(Roll / 2);
    q3 = -sinf(Pitch / 2) * sinf(Roll / 2);
}

/**
  * @brief          AHRS algorithm update
  * @param[1-3]     gyro measurement in rad/s
  * @param[4-6]     accelerometer measurement in m/s2
  * @param[7-9]     magnetometer measurement in uT
  * @param[10]      time stamp in s
  */
void Quaternion_AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    static float recipNorm;
    static float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    static float hx, hy, bx, bz;
    static float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    static float halfex, halfey, halfez;
    static float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

					integralFBx += 0.1 * halfex * (1.0f / 1000);	// integral error scaled by Ki
			integralFBy += 0.1 * halfey * (1.0f / 1000);
			integralFBz += 0.1 * halfez * (1.0f / 1000);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
			
        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);

    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    AHRS.q[0] = q0;
    AHRS.q[1] = q1;
    AHRS.q[2] = q2;
    AHRS.q[3] = q3;
}

/**
  * @brief          IMU algorithm update
  * @param[1-3]     gyro measurement in rad/s
  * @param[4-6]     accelerometer measurement in m/s2
  * @param[7-9]     gravity vector in m/s2
  * @param[10]       time stamp in s
  */
void Quaternion_AHRS_UpdateIMU(float q[4],float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    static float recipNorm;
    static float halfvx, halfvy, halfvz;
    static float halfex, halfey, halfez;
    static float qa, qb, qc;
    static float kpScale;
    static float gyroRAW[3], gyro[3], gyro_[3], accel[3], accel_[3];
    static float lpf = 0.00002;

    gyroRAW[0] = gx;
    gyroRAW[1] = gy;
    gyroRAW[2] = gz;
    gyro_[0] = gx * lpf / (lpf + dt) + gyro[0] * dt / (lpf + dt);
    gyro_[1] = gy * lpf / (lpf + dt) + gyro[1] * dt / (lpf + dt);
    gyro_[2] = gz * lpf / (lpf + dt) + gyro[2] * dt / (lpf + dt);
    gyro[0] = gx * lpf / (lpf + dt) + gyro_[0] * dt / (lpf + dt);
    gyro[1] = gy * lpf / (lpf + dt) + gyro_[1] * dt / (lpf + dt);
    gyro[2] = gz * lpf / (lpf + dt) + gyro_[2] * dt / (lpf + dt);
    accel_[0] = ax * lpf / (lpf + dt) + accel[0] * dt / (lpf + dt);
    accel_[1] = ay * lpf / (lpf + dt) + accel[1] * dt / (lpf + dt);
    accel_[2] = az * lpf / (lpf + dt) + accel[2] * dt / (lpf + dt);
    accel[0] = ax * lpf / (lpf + dt) + accel_[0] * dt / (lpf + dt);
    accel[1] = ay * lpf / (lpf + dt) + accel_[1] * dt / (lpf + dt);
    accel[2] = az * lpf / (lpf + dt) + accel_[2] * dt / (lpf + dt);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((fabsf(ax) < 0.00001f) && (fabsf(ay) < 0.00001f) && (fabsf(az) < 0.00001f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        kpScale = TuningKp(gyro, accel, &twoKp);

        // Apply proportional feedback
        gx += twoKp * halfex * kpScale;
        gy += twoKp * halfey * kpScale;
        gz += twoKp * halfez * kpScale;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += 0.5f * dt * (qa * gyroRAW[2] + qb * gyroRAW[1] - qc * gyroRAW[0]);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    AHRS.q[0] = q0;
    AHRS.q[1] = q1;
    AHRS.q[2] = q2;
    AHRS.q[3] = q3;
		
		q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;		
}

static float TuningKp(float *gyro, float *acc, float *kp)
{
    float scale = 1;
    float accNorm;
    float gyroNorm;
    float normDifference;
    static float maxDifference = 0.5, differenceThreshold = 0.1, gyroNormThreshold = 0.084;

    accNorm = 1 / invSqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    gyroNorm = 1 / invSqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    normDifference = fabsf(accNorm - Gravity * 0 - 9.67f);
    if (normDifference > differenceThreshold || gyroNorm > gyroNormThreshold)
    {
        if (normDifference > maxDifference)
            scale = 0;
        else if (normDifference > differenceThreshold)
        {
            if (gyroNorm > gyroNormThreshold)
                scale = differenceThreshold / normDifference * gyroNormThreshold / gyroNorm;
            else
                scale = differenceThreshold / normDifference;
        }
        else
        {
            scale = gyroNormThreshold / gyroNorm;
        }
    }
    
    return scale;
}

/**
  * @brief        Convert quaternion to eular angle
  */
void Get_EulerAngle(float *q , float * yaw, float *pitch)
{
    static int16_t Pitch_Round_Count = 0;
    static int16_t Yaw_Round_Count = 0;
    static float Pitch_Angle_Last = 0;
    static float Yaw_Angle_Last = 0;
		static float Yaw_I_count=0;
	
		Yaw_I_count+=0.00025150764;
	
    AHRS.Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    AHRS.Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    AHRS.Roll = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.295779513f;

    // Yaw rount count
    if (AHRS.Yaw - Yaw_Angle_Last > 180.0f)
        Yaw_Round_Count--;
    else if (AHRS.Yaw - Yaw_Angle_Last < -180.0f)
        Yaw_Round_Count++;
    // Pitch rount count
    if (AHRS.Pitch - Pitch_Angle_Last > 180.0f)
        Pitch_Round_Count--;
    else if (AHRS.Pitch - Pitch_Angle_Last < -180.0f)
        Pitch_Round_Count++;
		
		

    AHRS.YawTotalAngle = 360.0f * Yaw_Round_Count + AHRS.Yaw-Yaw_I_count;
		
		
		if (AHRS.YawTotalAngle > 360) AHRS.YawTotalAngle -=360;
		if (AHRS.YawTotalAngle < 0) AHRS.YawTotalAngle +=360;
		
    AHRS.PitchTotalAngle = 360.0f * Pitch_Round_Count + AHRS.Pitch;

    Yaw_Angle_Last = AHRS.Yaw;
    Pitch_Angle_Last = AHRS.Pitch;
		
		*yaw = Yaw_Angle_Last;
		*pitch = AHRS.Pitch ;
		
	
		
}

/**
  * @brief        Convert quaternion to eular angle
  */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.295779513f;
}

/**
  * @brief        Convert eular angle to quaternion
  */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    q[0] = cosf(Pitch / 2) * cosf(Roll / 2) * cosf(Yaw / 2) + sinf(Pitch / 2) * sinf(Roll / 2) * sinf(Yaw / 2);
    q[1] = sinf(Pitch / 2) * cosf(Roll / 2) * cosf(Yaw / 2) - cosf(Pitch / 2) * sinf(Roll / 2) * sinf(Yaw / 2);
    q[2] = sinf(Pitch / 2) * cosf(Roll / 2) * sinf(Yaw / 2) + cosf(Pitch / 2) * sinf(Roll / 2) * cosf(Yaw / 2);
    q[3] = cosf(Pitch / 2) * cosf(Roll / 2) * sinf(Yaw / 2) - sinf(Pitch / 2) * sinf(Roll / 2) * cosf(Yaw / 2);
}

void InsertQuaternionFrame(QuaternionBuf_t *qBuf, float *q, float time_stamp)
{
    if (qBuf->LatestNum == Q_FRAME_LEN - 1)
        qBuf->LatestNum = 0;
    else
        qBuf->LatestNum++;

    qBuf->qFrame[qBuf->LatestNum].TimeStamp = time_stamp;
    for (uint16_t i = 0; i < 4; i++)
        qBuf->qFrame[qBuf->LatestNum].q[i] = q[i];
}

uint16_t FindTimeMatchFrame(QuaternionBuf_t *qBuf, float match_time_stamp)
{
    float min_time_error = fabsf(qBuf->qFrame[0].TimeStamp - match_time_stamp);
    uint16_t num = 0;
    for (uint16_t i = 0; i < Q_FRAME_LEN; i++)
    {
        if (fabsf(qBuf->qFrame[i].TimeStamp - match_time_stamp) < min_time_error)
        {
            min_time_error = fabsf(qBuf->qFrame[i].TimeStamp - match_time_stamp);
            num = i;
        }
    }
    return num;
}

/**
  * @brief          Transform 3dvector from BodyFrame to EarthFrame
  * @param[1]       vector in BodyFrame
  * @param[2]       vector in EarthFrame
  * @param[3]       quaternion
  */
void BodyFrameToEarthFrame(float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
  * @brief          Transform 3dvector from EarthFrame to BodyFrame
  * @param[1]       vector in EarthFrame
  * @param[2]       vector in BodyFrame
  * @param[3]       quaternion
  */
void EarthFrameToBodyFrame(float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

float invSqrt(float x) {
	return 1/sqrtf(x);
}
