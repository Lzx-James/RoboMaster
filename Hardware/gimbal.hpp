#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <hardware_config.h>
#include <main.h>
#include <motor_config.h>
#include <pid.h>
#include <user_lib.h>

typedef enum {
    GIMBAL_STATE_STOP = 0,  //停止模式
    GIMBAL_STATE_NORMAL,    //普通模式
    GIMBAL_STATE_SPINNING,  //小陀螺模式
} gimbal_state_t;

typedef struct gimbal_behavior {
    volatile servo_degree_t camera_degree_;  //  摄像头电机 角度
    volatile servo_degree_t camera_speed_;   //  摄像头电机 速度

    volatile int16_t shoot_degree_;  //  枪管电机 角度
    volatile int16_t shoot_speed_;   //  枪管电机 速度

    volatile int16_t yaw_degree_;  // Yaw轴电机 角度
    volatile int16_t yaw_speed_;   // Yaw轴电机 速度
} gimbal_behavior_t;

class Gimbal {
   public:
    Gimbal(void);

    int  init(void);
    void set_state(gimbal_state_t state);
    void set_camera(servo_degree_t camera_degree);
    void set_shoot(int16_t shoot_degree);
    void set_yaw(int16_t yaw_speed);
    void update(uint8_t id, uint8_t data[8]);

    void set_current(servo_degree_t camera_motor, int16_t shoot_motor, int16_t yaw_motor);
    void pitch_limit(void);
    void motion_solution(void);
    void gimbal_pid_set(void);
    void handler(void);

    void manual_mode(int32_t pitch_delta, int16_t yaw_speed);
    void auto_mode(servo_degree_t camera_target, int32_t shoot_target, int16_t yaw_speed);
   
    servo_degree_t get_camera(void);
    int16_t        get_shoot(void);
    int16_t        get_yaw(void);

    void debug(void);

   private:
    servo_motor_t     camera_servo_;  // 摄像头伺服电机
    gm6020_t          shoot_gm6020_;  // 枪管 6020电机
    gm6020_t          yaw_gm6020_;    // yaw 6020电机
    gimbal_behavior_t behavior_;      // 期望速度角度缓存
    gimbal_state_t    state_;         // 云台状态

    pid_type_t camera_pid_speed_;  // 摄像头电机速度 PID
    pid_type_t shoot_pid_speed_;   // 枪管电机速度 PID
    pid_type_t yaw_pid_speed_;     // yaw轴电机速度 PID

    pid_type_t camera_pid_angle_;  // 摄像头电机角度 PID
    pid_type_t shoot_pid_angle_;   // 枪管电机角度 PID
    uint8_t    yaw_pid_angle_flag_;
    pid_type_t yaw_pid_angle_;  // yaw轴电机角度 PID

    static void timeout(void *parameter);
    rt_timer_t  timer_;  // 云台电机 PID 计算定时器
};

extern Gimbal gimbal;

#endif
