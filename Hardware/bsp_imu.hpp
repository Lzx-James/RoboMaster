#ifndef _BSP_IMU_H
#define _BSP_IMU_H

#include "main.h"
#include "pid.h"

class IMU {
   public:
    IMU(void);
    void init(void);
    void update(void);

    void read_gyro(fp32 *gyro);
    void read_accel(fp32 *accel);
    void read_mag(fp32 *mag);
    void read_q4(fp32 *q);

    fp32 read_pitch(void);
    fp32 read_roll(void);
    fp32 read_yaw(void);
    fp32 read_temperate(void);

   private:
    void        AHRS_update(void);
    void        temperate_control(void);
    void        imu_pwm_set(uint16_t pwm);
    static void imu_thread_entry(void *parameter);
    rt_thread_t imu_thread;

    pid_type_t temp_pid;

    fp32 pitch, roll, yaw;
    fp32 pitch_, roll_, yaw_;

    fp32 accel_fliter_1[3], accel_fliter_2[3], accel_fliter_3[3];

    fp32 gyro[3], accel[3], mag[3], temperate;
    fp32 quat[4];
};

extern IMU imu;

#endif
