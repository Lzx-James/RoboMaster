#include "wattmeter.hpp"

Wattmeter wattmeter;

/**
 * @brief 瓦特计初始化
 *
 * @param
 */
Wattmeter::Wattmeter(void) {
}

/**
 * @brief 瓦特计更新数据函数
 *
 * @param data
 */
void Wattmeter::update_data(uint8_t *data) {
    uint16_t *data_u16 = (uint16_t *)data;
    this->voltage_     = data_u16[0] / 100.0F;
    this->current_     = data_u16[1] / 100.0F;
    this->power_       = this->voltage_ * this->current_;
}

int Wattmeter::get_power(void) {
    return (int)this->power_;
}
