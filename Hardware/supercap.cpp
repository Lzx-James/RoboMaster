#include "supercap.hpp"

#include "can.h"
#include "referee.hpp"
#include "rtthread.h"

/**
 * @brief 超级电容初始化
 *
 * @return int
 */
int Supercap::start(void) {
    /* 初始化超级电容定时器 */
    timer_ = rt_timer_create("supercap", timeout, (void *)this, 200, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    /* 启动超级电容定时器 */
    rt_timer_start(timer_);

    return 0;
}

/**
 * @brief 更新超级电容数据
 *
 * @param data 原始数据
 */
void Supercap::update(uint8_t Data[8]) {
    uint16_t *data = (uint16_t *)Data;
    input_voltage  = data[0] / 100.0F;
    cap_voltage    = data[1] / 100.0F;
    input_current  = data[2] / 100.0F;
    target_power   = data[3] / 100.0F;
}

/**
 * @brief 设置目标充电功率
 *
 * @param target_power
 */
void Supercap::set_power(uint16_t target_power) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8];
    uint32_t            tx_mailbox;

    /* 配置消息头 */
    tx_header.StdId              = 0x210U;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 计算发送数据 */
    tx_data[0] = target_power >> 8;
    tx_data[1] = target_power;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

/**
 * @brief 超级电容超时函数
 *
 * @param parameter
 */
void Supercap::timeout(void *parameter) {
    Supercap *t_supercap = (Supercap *)parameter;
    /* 设置超级电容功率 */
    if (game_robot_status.chassis_power_limit != 80) {
        t_supercap->set_power(50 * 100);
    } else {
        t_supercap->set_power(80 * 100);
    }
}
