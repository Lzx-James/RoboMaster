#include "bsp_imu.hpp"
#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "math.h"
#include "tim.h"

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, { 0.0f, 0.0f, 1.0f }

fp32              gyro_offset[3];
fp32              accel_offset[3];
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
extern AHRS_t     AHRS;

fp32 gyro_scale_factor[3][3]  = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
IMU::IMU(void) {
    this->quat[0] = 1.0f;
    this->quat[1] = 0.0f;
    this->quat[2] = 0.0f;
    this->quat[3] = 0.0f;

    this->yaw   = 0;
    this->roll  = 0;
    this->pitch = 0;
}
void IMU::init(void) {
    fp32 imu_temp_PID[5] = {1600.0f, 0.2f, 0.0f, 4500, 4500};
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    pid_init(&temp_pid, PID_POSITION, imu_temp_PID);
    while (BMI088_init()) {
        rt_thread_mdelay(10);
    }
    BMI088_read(this->gyro, this->accel, &temperate);
    Quaternion_AHRS_InitIMU(accel[0], accel[1], accel[2], 9.78);

    imu_thread = rt_thread_create("imu", imu_thread_entry, (void *)this, 1024, 5, 5);
    rt_thread_startup(imu_thread);
}
void IMU::AHRS_update(void) {
    Quaternion_AHRS_UpdateIMU(quat, this->gyro[0], this->gyro[1], this->gyro[2], accel_fliter_3[0], accel_fliter_3[1],
                              accel_fliter_3[2], 1.0f / 1000.0f);
    Get_EulerAngle(quat, &yaw_, &pitch);
}

void IMU::imu_pwm_set(uint16_t pwm) {
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}
void IMU::update(void) {
    BMI088_read(this->gyro, this->accel, &temperate);
    temperate_control();

    for (uint8_t i = 0; i < 3; i++) {
        gyro[i] = (gyro[0] - 0.0031f) * (gyro_scale_factor[i][0]) + (gyro[1] + 0.00018125f) * gyro_scale_factor[i][1] +
                  (gyro[2] + 0.0019f) * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = (accel[0] + 0.0483f) * accel_scale_factor[i][0] + (accel[1] + 0.0483f) * accel_scale_factor[i][1] +
                   (accel[2] - 0.0305f) * accel_scale_factor[i][2] + accel_offset[i];
    }

    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];

    accel_fliter_3[0] =
        accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + accel[0] * fliter_num[2];

    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];

    accel_fliter_3[1] =
        accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + accel[1] * fliter_num[2];

    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];

    accel_fliter_3[2] =
        accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + accel[2] * fliter_num[2];

    this->AHRS_update();
}
fp32 IMU::read_pitch(void) {
    return this->pitch;
}
fp32 IMU::read_roll(void) {
    return this->roll;
}
fp32 IMU::read_yaw(void) {
    return this->yaw_;
}
fp32 IMU::read_temperate(void) {
    return this->temperate;
}
void IMU::read_accel(fp32 *accel) {
    accel[0] = this->accel[0];
    accel[1] = this->accel[1];
    accel[2] = this->accel[2];
}
void IMU::read_gyro(fp32 *gyro) {
    gyro[0] = this->gyro[0];              // Roll
    gyro[1] = this->gyro[1];              // Pitch
    gyro[2] = this->gyro[2] - 0.002224f;  // Yaw
}

void IMU::read_q4(fp32 *q) {
    q[0] = this->quat[0];
    q[1] = this->quat[1];
    q[2] = this->quat[2];
    q[3] = this->quat[3];
}
void IMU::temperate_control(void) {
    fp32 tempPWM;
    tempPWM = pid_calc(&temp_pid, temperate, 45.0f);
    if (tempPWM < 0) {
        tempPWM = 0;
    }
    imu_pwm_set((int)tempPWM);
}

void IMU::imu_thread_entry(void *parameter) {
    IMU *t_imu = (IMU *)(parameter);
    while (1) {
        t_imu->update();
        rt_thread_mdelay(5);
    }
}
