#include "chassis.hpp"
#include "can.h"
#include "math.h"

fp32    speed_pid_param[5]      = {10.0f, 0.7f, 0.0f, 8000.0f, 8000.0f};  // PID 参数
fp32    follow_pid_param[5]     = {0.07, 0.0f, 0.0f, 300.0f, 300.0f};     // PID 参数
fp32    wheel_speed_ratio       = 17.0f;                                  // 麦轮速度倍率
int16_t normalization_max_speed = 0;                                      // 归一化最大速度

fp32 current_param_pid_[5] = {0.8f, 0.0f, 0.0f, 6000.0f, 4000.0f};  // PID 参数

// fp32 power_pid_param[5] = {0.005F, 0.0F, 0.0F, 0.999F, 0.99F};  // 转速比率方式
fp32 power_pid_param[5] = {0.01F, 0.002F, 0.1F, 0.999F, 0.99F};  // 电流方式

/**
 * @brief 对象初始化
 *
 */
Chassis::Chassis(void) {
    /* 初始化成员变量 */
    state_ = CHASSIS_STATE_STOP;

    behavior_.x_speed_       = 0;
    behavior_.y_speed_       = 0;
    behavior_.angular_speed_ = 0;

    power_.real_    = 0.0F;
    power_.max_     = 50.0F;
    power_.buffer_  = 60.0F;
    power_.control_ = 0;

    /* 初始化 PID */
    for (uint8_t i = 0; i < 4; i++) {
        pid_init(speed_pid_ + i, PID_POSITION, speed_pid_param);
    }

    /* 初始化 斜坡函数 */
    for (uint8_t i = 0; i < 3; i++) {
        ramp_init(spd_ramp_ + i);
        spd_ramp_[i].out = 0;
    }
    /* 初始化 PID */
    pid_init(&current_pid_, PID_POSITION, current_param_pid_);
    pid_init(&follow_pid_, PID_POSITION, follow_pid_param);
}

/**
 * @brief 底盘周期初始化函数
 *
 * @param
 */
int Chassis::init(void) {
    /* 初始化底盘定时器 */
    timer_ = rt_timer_create("chassis", timeout, (void *)this, 10, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    set_state(CHASSIS_STATE_MOVE);
    rt_timer_start(timer_);

    return 1;
}
/**
 * @brief 底盘PID处理超时函数
 *
 * @param parameter 入口参数列表
 */
void Chassis::timeout(void *parameter) {
    Chassis *t_chassis = (Chassis *)parameter;
    t_chassis->handler();
    // t_chassis->debug();
}
/**
 * @brief 底盘周期控制函数
 *
 * @param
 */
void Chassis::handler() {
    if (status_control())
        return;
    ramp_set();
    motion_solution();
    wheel_pid_set();
    // power_limit();

    /* 输出电机电流 */
    set_current(m3508_[0].set_current_, m3508_[1].set_current_, m3508_[2].set_current_, m3508_[3].set_current_);
}
/**
 * @brief 状态控制
 *
 * @return uint8_t 如果stop返回1，move返回0
 */
uint8_t Chassis::status_control(void) {
    if (state_ == CHASSIS_STATE_STOP) {
        set_current(0, 0, 0, 0);
        return 1;
    } else
        return 0;
}

/**
 * @brief 调用斜坡函数
 *
 */
void Chassis::ramp_set(void) {
    ramp_calc(spd_ramp_, behavior_.x_speed_, CHASSIS_RAMP_STEP);
    ramp_calc(spd_ramp_ + 1, behavior_.y_speed_, CHASSIS_RAMP_STEP);

    behavior_.x_speed_ = spd_ramp_[0].out;
    behavior_.y_speed_ = spd_ramp_[1].out;
}

/**
 * @brief 期望速度分解
 *
 */
void Chassis::motion_solution(void) {
    /* 麦轮速度解算 */
    m3508_[0].expect_speed_ =
        (behavior_.x_speed_ + behavior_.y_speed_ + behavior_.angular_speed_ * CHASSIS_ROTATE_RATIO) *
        wheel_speed_ratio;  // 左前
    m3508_[1].expect_speed_ =
        (-behavior_.x_speed_ + behavior_.y_speed_ + behavior_.angular_speed_ * CHASSIS_ROTATE_RATIO) *
        wheel_speed_ratio;  // 左后
    m3508_[2].expect_speed_ =
        (-behavior_.x_speed_ - behavior_.y_speed_ + behavior_.angular_speed_ * CHASSIS_ROTATE_RATIO) *
        wheel_speed_ratio;  // 右后
    m3508_[3].expect_speed_ =
        (behavior_.x_speed_ - behavior_.y_speed_ + behavior_.angular_speed_ * CHASSIS_ROTATE_RATIO) *
        wheel_speed_ratio;  // 右前

    /* 底盘速度归一化 */
    normalization_max_speed = 0;
    for (size_t i = 0; i < 4; i++) {
        if (normalization_max_speed <= fabs((fp32)m3508_[i].expect_speed_)) {
            normalization_max_speed = fabs((fp32)m3508_[i].expect_speed_);
        }
    }
    if (normalization_max_speed > NORMALIZATION_MAX_SPEED) {
        m3508_[0].expect_speed_ =
            (int16_t)(1.0F * m3508_[0].expect_speed_ / normalization_max_speed * NORMALIZATION_MAX_SPEED);
        m3508_[1].expect_speed_ =
            (int16_t)(1.0F * m3508_[1].expect_speed_ / normalization_max_speed * NORMALIZATION_MAX_SPEED);
        m3508_[2].expect_speed_ =
            (int16_t)(1.0F * m3508_[2].expect_speed_ / normalization_max_speed * NORMALIZATION_MAX_SPEED);
        m3508_[3].expect_speed_ =
            (int16_t)(1.0F * m3508_[3].expect_speed_ / normalization_max_speed * NORMALIZATION_MAX_SPEED);
    }
}

/**
 * @brief PID控制麦轮
 *
 */
void Chassis::wheel_pid_set(void) {
    /* 速度环 PID 计算 */
    for (uint8_t i = 0; i < 4; i++) {
        m3508_[i].set_current_ = pid_calc(speed_pid_ + i, m3508_[i].speed_, m3508_[i].expect_speed_);
    }
}

void Chassis::power_limit(void) {
    fp32 totalCurrent = 0;

    totalCurrent = fabs((fp32)m3508_[0].set_current_) + fabs((fp32)m3508_[1].set_current_) +
                   fabs((fp32)m3508_[2].set_current_) + fabs((fp32)m3508_[3].set_current_);
    if ((power_.buffer_ - power_.real_ * 0.4 < 0.33f * power_.max_) && (power_.control_ == 0))
        power_.control_ = 1;

    if (power_.control_) {
        for (uint8_t i = 0; i < 4; i++) {
            fp32 set_current__;
            // m3508_[i].set_current_
            set_current__ = (0.1f + 0.15 * power_.buffer_) * m3508_[i].set_current_ / (1 + totalCurrent);
           // m3508_out[i]  = pid_calc(&current_pid_, m3508_[i].current_, set_current__);
        }
        if (power_.buffer_ >= 0.85f * power_.max_)
            power_.control_ = 0;
    } else {
        for (uint8_t i = 0; i < 4; i++) {
            // m3508_out[i] = pid_calc(&current_pid_, m3508_[i].current_, m3508_[i].set_current_);
//            m3508_out[i] = m3508_[i].set_current_;
        }
    }
}

void Chassis::follow_mode(int16_t x_speed, int16_t y_speed, int16_t ref_angular, int16_t trgt_angular) {
    int16_t mech_degree, theta_speed;

    BOUNDARY_PROCESS(mech_degree, ref_angular - trgt_angular, 0, 8192);
    theta_speed = pid_calc(&follow_pid_, mech_degree, 0);
    // if (mech_degree < 500 &&mech_degree > -500) theta_speed = 0;

    set_speed(x_speed, y_speed, theta_speed);
}

void Chassis::spinning_mode(
    int16_t x_speed, int16_t y_speed, int16_t rotate_speed, int16_t ref_angular, int16_t trgt_angular) {
    int16_t mech_degree;
    fp32    theta_rad;
    int16_t x_speed_mapped, y_speed_mapped;
    double  PI = acos(-1.0F);

    pid_clear(&follow_pid_);

    BOUNDARY_PROCESS(mech_degree, trgt_angular - ref_angular, 0, 8192);
    theta_rad = PI / 4096.0F * mech_degree;

    /* 速度坐标变换 */
    y_speed_mapped = (int16_t)(cosf(theta_rad) * y_speed - sinf(theta_rad) * x_speed);
    x_speed_mapped = (int16_t)(sinf(theta_rad) * y_speed + cosf(theta_rad) * x_speed);

    set_speed(x_speed_mapped, y_speed_mapped, rotate_speed);
}

/**
 * @brief 状态设定消息队列
 *
 * @param state
 */
void Chassis::set_state(chassis_state_t state) {
    state_ = state;
}

/**
 * @brief 速度设定函消息队列
 *
 * @param x_speed
 * @param y_speed
 * @param angular_speed
 */
void Chassis::set_speed(int16_t x_speed, int16_t y_speed, int16_t angular_speed) {
    behavior_.x_speed_       = x_speed;
    behavior_.y_speed_       = y_speed;
    behavior_.angular_speed_ = angular_speed;
}

/**
 * @brief 功率设定函数
 *
 * @param power_real
 * @param power_max
 */
void Chassis::set_power(fp32 power_real, fp32 power_max, fp32 power_buffer) {
    power_.real_   = power_real;
    power_.max_    = power_max;
    power_.buffer_ = power_buffer;
}
/**
 * @brief 更新底盘单电机数据
 *
 * @param id 电机ID
 * @param data 电机原始报文数据
 */
void Chassis::update(uint8_t id, uint8_t data[8]) {
    /* 结构体指针 */
    m3508_t *pData;

    pData = m3508_ + (id - 1);

    /* 处理数据 */
    pData->degree_     = (data[0] << 8) | data[1];
    pData->speed_      = (data[2] << 8) | data[3];
    pData->current_    = (data[4] << 8) | data[5];
    pData->temprature_ = data[6];
}

/**
 * @brief 设置底盘电机电流
 *
 * @param motor_1 第一个3508的电流，范围[-16384, 16384]
 * @param motor_2 第二个3508的电流，范围[-16384, 16384]
 * @param motor_3 第三个3508的电流，范围[-16384, 16384]
 * @param motor_4 第四个3508的电流，范围[-16384, 16384]
 */
void Chassis::set_current(int16_t motor_1, int16_t motor_2, int16_t motor_3, int16_t motor_4) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8];
    uint32_t            tx_mailbox;

    /* 配置消息头 */
    tx_header.StdId              = 0x200U;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 计算发送数据 */
    tx_data[0] = motor_1 >> 8;
    tx_data[1] = motor_1;
    tx_data[2] = motor_2 >> 8;
    tx_data[3] = motor_2;
    tx_data[4] = motor_3 >> 8;
    tx_data[5] = motor_3;
    tx_data[6] = motor_4 >> 8;
    tx_data[7] = motor_4;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

void Chassis::debug(void) {
    // rt_kprintf("%d, %d, %d\n", behavior_.x_speed_, behavior_.y_speed_, behavior_.angular_speed_);
    // rt_kprintf("%d, %d\n", this->behavior_.angular_speed_, this->state_);
   // rt_kprintf("%f, %d, %d\n", power_.buffer_, 10000 * power_.control_, m3508_out[0]);
}
