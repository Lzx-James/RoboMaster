#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include "main.h"

class Supercap {
   public:
    int  start(void);
    void update(uint8_t Data[8]);
    void set_power(uint16_t target_power);

   private:
    volatile float input_voltage;  // 输入电压
    volatile float cap_voltage;    // 电容电压
    volatile float input_current;  // 输入电流
    volatile float target_power;   // 目标功率

    static void timeout(void *parameter);
    rt_timer_t  timer_;  // 超级电容定时器
};

#endif
