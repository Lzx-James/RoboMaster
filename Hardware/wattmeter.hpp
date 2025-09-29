#ifndef __WATTMETER_H
#define __WATTMETER_H

#include "main.h"

class Wattmeter {
   public:
    Wattmeter(void);
    void update_data(uint8_t *data);
    int  get_power(void);

   private:
    float voltage_;  //  电压
    float current_;  // 电流
    float power_;    // 功率
};

extern Wattmeter wattmeter;

#endif
