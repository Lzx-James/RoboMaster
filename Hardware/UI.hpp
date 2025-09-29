#ifndef __UI_H
#define __UI_H

#include "main.h"

/******** 图形名称定义 *******/
#define GRAPIC_TARGET_X           0x01  /* 准心十字线 横线 */
#define GRAPIC_TARGET_Y           0x02  /* 准心十字线 竖线 */
#define GRAPIC_CHARACTER_SUPERCAP 0x03  /* 超级电容 */
#define GRAPIC_CHARACTER_CHASSIS  0x04  /* 底盘 */
#define GRAPIC_CHARACTER_STUCK    0x05  /* 卡弹 */

class UI {
   public:
    void Target(int operation_tpye, int x, int y);
    void CapVoltage(int operation_tpye, float supercap_voltage);
    void ChassisMode(int operation_tpye, uint8_t is_chassis_spin);
    void Stuck(int operation_tpye, uint8_t is_stuck);
    void DeleteAllGrapic(void);
};

extern UI ui;

#endif // __UI_H
