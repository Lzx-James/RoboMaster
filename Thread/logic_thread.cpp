#include "logic_thread.hpp"

#include <math.h>
#include "tim.h"

#include "chassis.hpp"
#include "dr16.hpp"
#include "gimbal.hpp"
#include "shoot17.hpp"
#include "shoot42.hpp"

DR16    dr16;
Chassis chassis;
Gimbal  gimbal;
Shoot42 shoot42;
Shoot17 shoot17;

uint8_t spinning_flag = 0;  //小陀螺开启标志位

/**
 * @brief 底盘控制函数
 *
 */
void chassic_command() {
    static float spinning_speed = 80.0F;

    if (dr16.get_rc().left_sw_ == SW_DOWN) {
        chassis.set_state(CHASSIS_STATE_STOP);
        return;
    }
    if (dr16.get_rc().right_sw_ == SW_DOWN) {
        /* PC 控制模式 */
        int16_t pc_speed_x = 0, pc_speed_y = 0;

        if (dr16.pressed_down(KEY_SHIFT)) {
            pc_speed_y = (dr16.pressed_down(KEY_W) - dr16.pressed_down(KEY_S)) * 350;
            pc_speed_x = (dr16.pressed_down(KEY_A) - dr16.pressed_down(KEY_D)) * 350;
        } else {
            pc_speed_y = (dr16.pressed_down(KEY_W) - dr16.pressed_down(KEY_S)) * 150;
            pc_speed_x = (dr16.pressed_down(KEY_A) - dr16.pressed_down(KEY_D)) * 150;
        }
        if (dr16.clicked(KEY_E)) {
            /* E 键弹起时触发小陀螺模式 */
            spinning_flag = !spinning_flag;
        }
        if (spinning_flag) {
            /* 小陀螺模式 */
            chassis.spinning_mode(pc_speed_x, pc_speed_y, spinning_speed, gimbal.get_yaw_angular(), YAW_ZERO_DEGREE);
        } else {
            /* 正常模式 */
            chassis.follow_mode(pc_speed_x, pc_speed_y, gimbal.get_yaw_angular(), YAW_ZERO_DEGREE);
        }
    } else {
        /* 遥控器控制模式 */
        switch (dr16.get_rc().left_sw_) {
            case SW_UP:
                /* 小陀螺模式 */
                chassis.set_state(CHASSIS_STATE_MOVE);
                chassis.spinning_mode(dr16.get_rc().left_x_ - 1024, dr16.get_rc().left_y_ - 1024, spinning_speed,
                                      gimbal.get_yaw_angular(), YAW_ZERO_DEGREE);
                break;
            case SW_MID:
                /* 正常模式 */
                chassis.set_state(CHASSIS_STATE_MOVE);
                chassis.follow_mode(dr16.get_rc().left_x_ - 1024, dr16.get_rc().left_y_ - 1024,
                                    gimbal.get_yaw_angular(), YAW_ZERO_DEGREE);
                break;
            case SW_DOWN:
                /* 停止模式 */
                break;
        }
    }
}

void gimbal_command() {
    if (dr16.get_rc().left_sw_ == SW_DOWN) {
        gimbal.set_camera(0);
        gimbal.set_shoot(0);
        gimbal.set_yaw(0);
        return;
    }
    if (dr16.get_rc().right_sw_ == SW_DOWN) {
        /* PC 控制模式 */
        // TODO: minipc数据作为shoot轴数据
        int minipc = 0;
        gimbal.auto_mode((dr16.get_pc().y_) * 0.05, minipc, 1024 - dr16.get_pc().x_);
    } else {
        /* 遥控器控制模式 */
        switch (dr16.get_rc().left_sw_) {
            case SW_UP:
                /* 小陀螺模式 */
            case SW_MID:
                /* 正常模式 */
                gimbal.manual_mode((1024 - dr16.get_rc().right_y_) * 0.02, 1024 - dr16.get_rc().right_x_);
                break;
            case SW_DOWN:
                /* 停止模式 */
                break;
        }
    }
}

void shoot_command() {
    if (dr16.get_rc().left_sw_ == SW_DOWN) {
        shoot42.set_friction(FRICTION_42_STATE_SLOW_OUT, FRICTION_42_MIN_SPEED);
        shoot17.set_friction(FRICTION_17_STATE_SLOW_OUT, FRICTION_17_MIN_SPEED);
        shoot42.set_plunk(PLUNK_42_STATE_STOP);
        shoot17.set_plunk(PLUNK_17_STATE_STOP);
        return;
    }
    if (dr16.get_rc().right_sw_ == SW_DOWN) {
        /* PC 控制模式 */
#define COOLING_HEAT_MAX 100
        uint16_t cooling_heat_17 = 0, cooling_heat_42 = 0;
        shoot42.set_friction(FRICTION_42_STATE_SLOW_IN, FRICTION_42_MAX_SPEED);
        shoot17.set_friction(FRICTION_17_STATE_SLOW_IN, FRICTION_17_MAX_SPEED);
        if (dr16.get_pc().press_l_ && cooling_heat_42 <= COOLING_HEAT_MAX) {
            shoot42.set_plunk(PLUNK_42_STATE_WORK);
        } else {
            shoot42.set_plunk(PLUNK_42_STATE_STOP);
        }
        if (dr16.get_pc().press_l_ && cooling_heat_17 <= COOLING_HEAT_MAX) {
            shoot17.set_plunk(PLUNK_17_STATE_WORK);
        } else {
            shoot17.set_plunk(PLUNK_17_STATE_STOP);
        }
    } else {
        switch (dr16.get_rc().right_sw_) {
            case SW_UP:
                /* 启动摩擦轮 */
                shoot42.set_friction(FRICTION_42_STATE_SLOW_IN, FRICTION_42_MAX_SPEED);
                shoot17.set_friction(FRICTION_17_STATE_SLOW_IN, FRICTION_17_MAX_SPEED);
                if (dr16.get_rc().wheel_ >= 1600) {
                    shoot42.set_plunk(PLUNK_42_STATE_WORK);
                } else if (dr16.get_rc().wheel_ <= 600) {
                    shoot17.set_plunk(PLUNK_17_STATE_WORK);
                } else {
                    shoot42.set_plunk(PLUNK_42_STATE_STOP);
                    shoot17.set_plunk(PLUNK_17_STATE_STOP);
                }
                break;
            case SW_MID:
                /* 停止摩擦轮 */
                shoot42.set_friction(FRICTION_42_STATE_SLOW_OUT, FRICTION_42_MIN_SPEED);
                shoot17.set_friction(FRICTION_17_STATE_SLOW_OUT, FRICTION_17_MIN_SPEED);
                shoot42.set_plunk(PLUNK_42_STATE_STOP);
                shoot17.set_plunk(PLUNK_17_STATE_STOP);
                break;
            case SW_DOWN:
                /* PC控制模式 */
                break;
        }
    }
}

/**
 * @brief   主逻辑线程
 *
 * @param   parameter //入口参数
 *
 */
void logic_thread_entry(void *parameter) {
    /* 初始化 */
    dr16.init();
    gimbal.init();
    shoot42.init();
    shoot17.init();

    chassis.set_state(CHASSIS_STATE_MOVE);
	
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);	
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 580);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 580);
    while (1) {
        gimbal_command();
        chassic_command();
        //shoot_command();

        rt_thread_mdelay(5);
    }

#if 0
    //            /* 死区 */
    //            if (gimbal_data.yaw.degree - YAW_ZERO_DEGREE >= 150) {
    //                angular_speed = -pid_calc(&pid_follow_gimbal, gimbal_data.yaw.degree - 200, YAW_ZERO_DEGREE);
    //            } else if (YAW_ZERO_DEGREE - gimbal_data.yaw.degree >= 150) {
    //                angular_speed = -pid_calc(&pid_follow_gimbal, gimbal_data.yaw.degree + 200, YAW_ZERO_DEGREE);
    //            } else {
    //                angular_speed = 0;
    //            }
#endif
}
