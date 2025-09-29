#include "gimbal.hpp"
#include "bsp_imu.hpp"
#include "can.h"
#include "math.h"
#include "pid.h"
#include "rtthread.h"

/**
 * @brief Yaw轴 PID
 *
 */
fp32 gimbal_speed_ratio = 0.01F;  // 云台电机速度倍率

fp32 camera_pid_speed_param[5] = {7.0F, 0.0F, 5.0F, 20000, 15000};  // 摄像头电机 PID 参数
// fp32 shoot_pid_speed_param[5]  = {30.0F, 0.05F, 0.0F, 20000, 8000};       // 枪管电机 PID 参数
fp32 shoot_pid_speed_param[5] = {65.0F, 0.11F, 0.0F, 20000.0F, 8000.0F};  // 枪管电机 PID 参数
fp32 yaw_pid_speed_param[5]   = {13000.0F, 175.0F, 0.0F, 30000, 20000};    // yaw轴电机 PID 参数

fp32 camera_pid_angle_param[5] = {7.0F, 0.00F, 0.5F, 5000, 0};  //角度环PID
fp32 shoot_pid_angle_param[5]  = {2.0F, 0.00F, 1.5F, 200, 0};
// fp32 yaw_pid_angle_param[5]    = {20.0F, 0.02F, 0.0F, 5000, 1000};
fp32 yaw_pid_angle_param[5] = {8.0F, 0.0F, 1.0F, 5000, 0};

Gimbal::Gimbal(void) {
    behavior_.camera_degree_ = 0;
    behavior_.camera_speed_  = 0;
    behavior_.shoot_degree_  = 0;
    behavior_.shoot_speed_   = 0;
    behavior_.yaw_degree_    = 0;
    behavior_.yaw_speed_     = 0;

    state_ = GIMBAL_STATE_NORMAL;
    /* 摄像头电机 PID 初始化 */
    pid_init(&(camera_pid_speed_), PID_POSITION, camera_pid_speed_param);
    pid_init(&camera_pid_angle_, PID_POSITION, camera_pid_angle_param);
    /* 枪管电机 初始PID 初始化 */
    pid_init(&(shoot_pid_speed_), PID_POSITION, shoot_pid_speed_param);
    pid_init(&shoot_pid_angle_, PID_POSITION, shoot_pid_angle_param);
    /* Yaw轴电机 初始PID 初始化 */
    yaw_pid_angle_flag_ = 0;
    pid_init(&(yaw_pid_speed_), PID_POSITION, yaw_pid_speed_param);
    pid_init(&yaw_pid_angle_, PID_POSITION, yaw_pid_angle_param);
}
/**
 * @brief 云台初始化函数
 *
 */
int Gimbal::init(void) {
    /* 初始化定时器 */
    timer_ = rt_timer_create("gimbal", timeout, (void *)this, 5, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);

    set_camera(0);
    set_shoot(0);
    set_yaw(0);

    /* 启动定时器 */
    rt_timer_start(timer_);

    rt_thread_mdelay(1000);  //等待抬头

    /*抬头结束，重新配置pid*/
    pid_init(&shoot_pid_speed_, PID_POSITION, shoot_pid_speed_param);

    return 1;
}
/**
 * @brief 云台控制定时器超时函数
 *
 * @param parameter 云台对象
 */
void Gimbal::timeout(void *parameter) {
    Gimbal *t_gimbal = (Gimbal *)parameter;
    t_gimbal->handler();
    t_gimbal->debug();
}

/**
 * @brief 云台周期控制函数
 *
 */
void Gimbal::handler(void) {
    motion_solution();
    pitch_limit(); /* Pitch 限位 */
    gimbal_pid_set();

    set_current(camera_servo_.expect_degree_, shoot_gm6020_.set_current_, yaw_gm6020_.set_current_);
}

/**
 * @brief 云台角度姿态解算
 *
 */
void Gimbal::motion_solution(void) {
    camera_servo_.expect_degree_ = CAMERA_ZERO_DEGREE + behavior_.camera_degree_;
    shoot_gm6020_.expect_degree_ = SHOOT_ZERO_DEGREE + behavior_.shoot_degree_;
    yaw_gm6020_.expect_degree_   = behavior_.yaw_degree_;
    yaw_gm6020_.expect_speed_    = behavior_.yaw_speed_;
}

/**
 * @brief Pitch 轴角度限位
 *
 */
void Gimbal::pitch_limit(void) {
    /* 速度限位 */
    uint16_t pitch_degree_min;

    if (state_ == GIMBAL_STATE_SPINNING) {
        pitch_degree_min = SHOOT_ZERO_DEGREE - 340;
    } else if (state_ == GIMBAL_STATE_NORMAL) {
        pitch_degree_min = SHOOT_ZERO_DEGREE - 300;
    }

    if (camera_servo_.expect_degree_ <= CAMERA_ZERO_DEGREE - 1500) {
        camera_servo_.expect_degree_ = CAMERA_ZERO_DEGREE - 1500;
    } else if (camera_servo_.expect_degree_ >= CAMERA_ZERO_DEGREE + 1500) {
        camera_servo_.expect_degree_ = CAMERA_ZERO_DEGREE + 1500;
    }

    if (shoot_gm6020_.expect_degree_ <= pitch_degree_min) {
        shoot_gm6020_.expect_degree_ = pitch_degree_min;
    } else if (shoot_gm6020_.expect_degree_ >= SHOOT_ZERO_DEGREE + 790) {
        shoot_gm6020_.expect_degree_ = SHOOT_ZERO_DEGREE + 790;
    }
}

/**
 * @brief 云台电机 PID
 *
 */
void Gimbal::gimbal_pid_set(void) {
    fp32 gyro[3];
    imu.read_gyro(gyro);

    shoot_gm6020_.expect_speed_ = pid_calc(&shoot_pid_angle_, shoot_gm6020_.degree_, shoot_gm6020_.expect_degree_);
    shoot_gm6020_.set_current_  = pid_calc(&shoot_pid_speed_, shoot_gm6020_.speed_, shoot_gm6020_.expect_speed_);
    if (yaw_pid_angle_flag_ == 1) {
        yaw_gm6020_.expect_speed_ = pid_calc(&yaw_pid_angle_, yaw_gm6020_.degree_, yaw_gm6020_.expect_degree_);
    }
    //云台倍率在此处传递，防止浮点数精度缺失
    yaw_gm6020_.set_current_ = pid_calc(&yaw_pid_speed_, gyro[2], gimbal_speed_ratio * yaw_gm6020_.expect_speed_);
}

/**
 * @brief 状态设定消息队列
 *
 * @param state
 */
void Gimbal::set_state(gimbal_state_t state) {
    state_ = state;
}

/**
 * @brief 摄像头电机角度数据访问
 *
 * @param camera_degree 摄像头电机角度
 */
void Gimbal::set_camera(servo_degree_t camera_degree) {
    behavior_.camera_degree_ = camera_degree;
}

/**
 * @brief 枪管电机角度数据访问
 *
 * @param shoot_degree 枪管电机角度
 */
void Gimbal::set_shoot(int16_t shoot_degree) {
    behavior_.shoot_degree_ = shoot_degree;
}
/**
 * @brief Yaw轴电机角度数据访问
 *
 * @param yaw_speed Yaw轴电机角度
 */
void Gimbal::set_yaw(int16_t yaw_speed) {
    behavior_.yaw_speed_ = yaw_speed;
}

/**
 * @brief 云台控制接口-调试模式
 *
 * @param pitch_delta Pitch轴角度增量
 * @param yaw_speed Yaw轴角度增量
 */
void Gimbal::manual_mode(int32_t pitch_delta, int16_t yaw_speed) {
    static float camera_angular = 0, shoot_angular = 0;

    pitch_delta *= 0.1;
    camera_angular += (pitch_delta * 1.6384);
    if (camera_angular >= 1500)
        camera_angular = 1500;
    else if (camera_angular <= -600)
        camera_angular = -600;

    shoot_angular += (pitch_delta * 0.8192);
    if (shoot_angular >= 750)
        shoot_angular = 750;
    else if (shoot_angular <= -300)
        shoot_angular = -300;

    set_camera((-1) * (int32_t)camera_angular);
    set_shoot((int32_t)shoot_angular);
    set_yaw(yaw_speed);
}

/**
 * @brief 云台控制接口-控制模式
 *
 * @param camera_delta 摄像头电机角度增量
 * @param shoot_delta 枪管电机角度增量
 * @param yaw_speed Yaw轴电机角度增量
 */
void Gimbal::auto_mode(servo_degree_t camera_target, int32_t shoot_target, int16_t yaw_speed) {
    static servo_degree_t camera_angular = 0;
    static int16_t        shoot_angular  = 0;

    camera_angular += camera_target;
    if (camera_angular >= 1500)
        camera_angular = 1500;
    else if (camera_angular <= -780)
        camera_angular = -780;

    shoot_angular += shoot_target;
    if (shoot_angular >= 750)
        shoot_angular = 750;
    else if (shoot_angular <= -780 / 2)
        shoot_angular = -(780 / 2);

    yaw_pid_angle_flag_ = 0;

    set_camera((-1) * (int32_t)camera_angular);
    set_shoot((int32_t)shoot_angular);
    set_yaw(yaw_speed);
}

/**
 * @brief 读取摄像头电机角度
 *
 * @return servo_degree_t 摄像头电机角度
 */
servo_degree_t Gimbal::get_camera(void) {
    // TODO 485电机实际角度无反馈
    camera_servo_.degree_ = camera_servo_.expect_degree_;
    return camera_servo_.degree_;
}

/**
 * @brief 读取发射机构电机角度
 *
 * @return int16_t 发射机构电机角度
 */
int16_t Gimbal::get_shoot(void) {
    return shoot_gm6020_.degree_;
}

/**
 * @brief 读取Yaw轴电机角度
 *
 * @return int16_t Yaw轴电机角度
 */
int16_t Gimbal::get_yaw(void) {
    return yaw_gm6020_.degree_;
}
/**
 * @brief 更新云台电机数据
 *
 * @param ID 电机ID
 * @param Data 电机原始数据
 */
void Gimbal::update(uint8_t id, uint8_t data[8]) {
    if (id == 1) {
        /* Yaw轴 电机数据 */
        // *用原始速度即可，因为控制频率上去之后速度大小和电机反馈的差不多
        yaw_gm6020_.degree_     = (data[0] << 8) | data[1];
        yaw_gm6020_.speed_      = (data[2] << 8) | data[3];
        yaw_gm6020_.current_    = (data[4] << 8) | data[5];
        yaw_gm6020_.temprature_ = data[6];
    }
    if (id == 2) {
        /* 枪管 电机数据 */
        shoot_gm6020_.degree_     = (data[0] << 8) | data[1];
        shoot_gm6020_.speed_      = (data[2] << 8) | data[3];
        shoot_gm6020_.current_    = (data[4] << 8) | data[5];
        shoot_gm6020_.temprature_ = data[6];
    }
}

/**
 * @brief 设置云台电机的电压
 *
 * @param Yaw GM6020的电压，范围[-30000, 30000]
 * @param Shoot GM6020的电压，范围[-30000, 30000]
 * @param Camera 伺服电机的角度，范围[-1500, 1500]
 */
void Gimbal::set_current(servo_degree_t camera_motor, int16_t shoot_motor, int16_t yaw_motor) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8];
    uint32_t            tx_mailbox;

    /* Yaw轴电机设置电流 */

    /* 配置消息头 */
    tx_header.StdId              = 0x1FFU;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 计算发送数据 */
    tx_data[0] = yaw_motor >> 8;
    tx_data[1] = yaw_motor;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);

    /* 枪管电机设置电流 */

    /* 计算发送数据 */
    tx_data[0] = shoot_motor >> 8;
    tx_data[1] = shoot_motor;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox);

    /* 摄像头电机设置角度 */

    /* 配置消息头 */
    tx_header.StdId              = 0x001U;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = 4;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 计算发送数据 */
    tx_data[0] = camera_motor;
    tx_data[1] = camera_motor >> 8;
    tx_data[2] = camera_motor >> 16;
    tx_data[3] = camera_motor >> 24;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox);
}

/**
 * @brief 调试函数
 *
 */
void Gimbal::debug(void) {
    // fp32 gyro[3];
    // imu.read_gyro(gyro);
    // if (!yaw_gm6020_.speed_) {
    //     rt_kprintf("0\n");
    // } else {
    //     rt_kprintf("%f\n", gyro[2] / yaw_gm6020_.speed_);
    // }
    // rt_kprintf("%f, %f, %f\n", shoot_pid_angle_.Pout, shoot_pid_angle_.Dout, (fp32)yaw_gm6020_.expect_speed_);
}
