#include "dr16.hpp"

/**
 * @brief 对象初始化
 *
 */
DR16::DR16(void) {
    reset();
}

/**
 * @brief 初始化遥控器以及遥控器事件
 *
 */
void DR16::init(void) {
    DMALIB_UART_Receive_IT(&DBUS_USART, data, DR16_RECEIVE_SIZE);

    event     = rt_event_create("dr16", RT_IPC_FLAG_PRIO);
    rc_thread = rt_thread_create("rc", rc_thread_entry, (void *)this, 1024, 4, 10);

    rt_thread_startup(rc_thread);
}
/**
 * @brief 初始化遥控器
 *
 */
void DR16::reset(void) {
    rc_.left_x_ = rc_.left_y_ = rc_.right_x_ = rc_.right_y_ = rc_.wheel_ = 1024;
    rc_.left_sw_                                                         = 2;
    rc_.right_sw_                                                        = 3;
    pc_.x_ = pc_.y_ = pc_.z_ = pc_.press_l_ = pc_.press_r_ = pc_.pre_v_ = pc_.v_ = 0;
}

/**
 * @brief 解码遥控器数据
 *
 */
void DR16::decode(void) {
    rc_.right_x_ = ((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF;
    rc_.right_y_ = (((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF;
    rc_.left_x_  = (((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) | ((int16_t)data[4] << 10)) & 0x07FF;
    rc_.left_y_  = (((int16_t)data[4] >> 1) | ((int16_t)data[5] << 7)) & 0x07FF;
    rc_.wheel_   = ((int16_t)data[16] | ((int16_t)data[17] << 8)) & 0x07FF;

    rc_.left_sw_  = ((data[5] >> 4) & 0x000C) >> 2;
    rc_.right_sw_ = ((data[5] >> 4) & 0x0003);
    pc_.x_        = ((int16_t)data[6]) | ((int16_t)data[7] << 8);
    pc_.y_        = ((int16_t)data[8]) | ((int16_t)data[9] << 8);
    pc_.z_        = ((int16_t)data[10]) | ((int16_t)data[11] << 8);
    pc_.press_l_  = data[12];
    pc_.press_r_  = data[13];
    pc_.pre_v_    = pc_.v_;
    pc_.v_        = ((int16_t)data[14]);

    rt_event_send(event, DR16_EVENT_ONLINE);
}

/**
 * @brief 在线事件接收
 *
 * @return rt_err_t
 */
rt_err_t DR16::event_recv(void) {
    return rt_event_recv(event, DR16_EVENT_ONLINE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 50, RT_NULL);
}

/**
 * @brief 遥控线程
 *
 * @param parameter
 */
void DR16::rc_thread_entry(void *parameter) {
    rt_err_t ret;
    DR16    *t_dr16 = (DR16 *)parameter;

    while (1) {
        ret = t_dr16->event_recv();

        /* 遥控器断开连接 */
        if (ret != RT_EOK) {
            t_dr16->reset();
        }
        // t_dr16->debug();
    }
}

/**
 * @brief 访问遥控器数据
 *
 * @return struct __RC
 */
struct __RC DR16::get_rc(void) {
    return rc_;
}
/**
 * @brief 访问PC数据
 *
 * @return struct __PC
 */
struct __PC DR16::get_pc(void) {
    return pc_;
}

/**
 * @brief 按键是否按下
 *
 * @param key 要查询的按键，可用或运算判断多个按键
 * @return uint16_t 是否按下
 */
uint8_t DR16::pressed_down(uint16_t key) {
    return (pc_.v_ & key) ? 1 : 0;
}

/**
 * @brief 按键是否点击（按下后弹起则有效）
 *
 * @param key 要查询的按键
 * @return uint16_t 是否点击
 */
uint8_t DR16::clicked(uint16_t key) {
    return ((pc_.pre_v_ & key) && !(pc_.v_ & key)) ? 1 : 0;
}

/**
 * @brief 调试函数
 * 
 */
void DR16::debug(void) {
    rt_kprintf("%d,%d,%d,%d\n ", rc_.left_sw_, rc_.right_sw_, rc_.wheel_, rc_.left_x_);
}
