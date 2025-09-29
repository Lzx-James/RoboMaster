#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <hardware_config.h>
#include <main.h>
#include <motor_config.h>
#include <pid.h>
#include <user_lib.h>

typedef enum {
    CHASSIS_STATE_STOP = 0,  //停止模式
    CHASSIS_STATE_MOVE,      //移动模式
} chassis_state_t;

typedef struct {
    volatile int16_t x_speed_;        // X 速度
    volatile int16_t y_speed_;        // Y 速度
    volatile int16_t angular_speed_;  // 角速度
} chassis_behavior_t;

typedef struct {
    fp32    real_;
    fp32    max_;
    fp32    buffer_;
    uint8_t control_;
} chassis_power_t;

class Chassis {
   public:
    Chassis(void);

    int     init(void);
    uint8_t status_control(void);
    void    set_state(chassis_state_t state);
    void    set_speed(int16_t x_speed, int16_t y_speed, int16_t angular_speed);
    void    set_power(fp32 power_real, fp32 power_max, fp32 power_buffer);
    void    update(uint8_t id, uint8_t data[8]);

    void set_current(int16_t motor_1, int16_t motor_2, int16_t motor3, int16_t motor_4);
    void motion_solution(void);
    void ramp_set(void);
    void wheel_pid_set(void);
    void power_limit(void);
    void handler(void);

    void follow_mode(int16_t x_speed, int16_t y_speed, int16_t ref_angular, int16_t trgt_angular);
    void spinning_mode(
        int16_t x_speed, int16_t y_speed, int16_t rotate_speed, int16_t ref_angular, int16_t trgt_angular);

    void debug(void);

   private:
    m3508_t            m3508_[4];  // 3508电机
    chassis_behavior_t behavior_;  // 期望速度缓存
    chassis_state_t    state_;     // 底盘状态
    chassis_power_t    power_;     // 底盘功率

    pid_type_t current_pid_;  // 电流环 PID

    pid_type_t             speed_pid_[4];  // 底盘电机速度 PID
    ramp_function_source_t spd_ramp_[3];   // 输入量斜坡函数结构体

    pid_type_t follow_pid_;  // 跟随 PID

    static void timeout(void *parameter);
    rt_timer_t  timer_;  // 底盘电机 PID 计算定时器
};

extern Chassis chassis;

#endif
