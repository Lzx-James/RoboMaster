#include "minipc.hpp"
#include "dr16.hpp"
#include "math.h"
#include "rtthread.h"
#include "stdlib.h"
#include "usart.h"

/**
 * @brief Construct a new Minipc:: Minipc object
 *
 */
Minipc::Minipc(void) {
    // 接收数据缓存
    recv_behavior_.shoot_angular_ = 0;
    recv_behavior_.yaw_angular_   = 0;

    // 发送数据缓存

    uint8_t *head_pointer          = (uint8_t *)(&send_behavior_);
    *(head_pointer + 0)            = 0xAA;
    *(head_pointer + 1)            = 0xBB;
    *(head_pointer + 2)            = 0xCC;
    *(head_pointer + 3)            = 0xDD;
    send_behavior_.camera_angular_ = 0;
    send_behavior_.shoot_angular_  = 0;
    send_behavior_.yaw_angular_    = 0;
}

/**
 * @brief Minipc初始化函数
 *
 */
void Minipc::init(void) {
    huart_ = &MINIPC_UART;
    DMALIB_UART_Receive_IT(&MINIPC_UART, (uint8_t *)(&(this->recv_behavior_)), sizeof(minipc_recv_behavior_t));
    event = rt_event_create("minipc", RT_IPC_FLAG_PRIO);
}

/**
 * @brief Minipc串口离线检测事件
 *
 */
void Minipc::recv_event(void) {
    rt_interrupt_enter();

    HAL_UART_Transmit(&MINIPC_UART, (uint8_t *)(&send_behavior_), sizeof(minipc_send_behavior_t), 20);
    rt_event_send(event, MINIPC_EVENT_ONLINE);

    rt_interrupt_leave();
}

/**
 * @brief Minipc数据发送
 *
 * @param camera 摄像头电机角度
 * @param shoot 枪管电机角度
 * @param yaw Yaw轴电机角度
 */
void Minipc::send_cmd(fp32 camera, fp32 shoot, fp32 yaw) {
    uint8_t *head_pointer = (uint8_t *)(&send_behavior_);
    *(head_pointer + 0)   = 0xAA;
    *(head_pointer + 1)   = 0xBB;
    *(head_pointer + 2)   = 0xCC;
    *(head_pointer + 3)   = 0xDD;

    send_behavior_.camera_angular_ = camera;
    send_behavior_.shoot_angular_  = shoot;
    send_behavior_.yaw_angular_    = yaw;
}

/**
 * @brief Minipc数据访问
 *
 * @return minipc_recv_behavior_t Minipc接收数据结构体
 */
minipc_recv_behavior_t Minipc::get_cmd(void) {
    //Minipc 读一次数据清零 pitch轴，（yaw做速度环不清零），在此函数中实现，缓冲区机制
    minipc_recv_behavior_t recv_temp = recv_behavior_;
    recv_behavior_.shoot_angular_    = 0;
    // recv_behavior_.yaw_angular_      = 0;
    return recv_temp;
}

/**
 * @brief Minipc事件处理
 *
 * @return rt_err_t Minipc在线事件的值
 */
rt_err_t Minipc::event_process(void) {
    return rt_event_recv(event, MINIPC_EVENT_ONLINE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 50, RT_NULL);
}

/**
 * @brief 调试函数
 *
 */
void Minipc::debug(void) {
    // HAL_UART_Transmit(&MINIPC_UART, (uint8_t *)&recv_behavior_, sizeof(minipc_recv_behavior_t), 0xFF);
}
