#ifndef __MINIPC_H
#define __MINIPC_H

#include <dmalib.h>
#include <hardware_config.h>
#include <main.h>
#include <usart.h>

#define MINIPC_EVENT_ONLINE 0x02  // minipc在线事件

typedef struct {
    fp32 head;
    fp32 shoot_angular_;
    fp32 yaw_angular_;
} minipc_recv_behavior_t;

typedef struct {
    fp32 head;
    fp32 camera_angular_;
    fp32 shoot_angular_;
    fp32 yaw_angular_;
} minipc_send_behavior_t;

class Minipc {
   public:
    Minipc(void);

    void                   init(void);
    void                   recv_event(void);
    rt_err_t               event_process(void);
    void                   send_cmd(fp32 camera, fp32 shoot, fp32 yaw);
    minipc_recv_behavior_t get_cmd(void);
    void                   debug(void);

   private:
    UART_HandleTypeDef *huart_;  // Minipc串口

    minipc_recv_behavior_t recv_behavior_;
    minipc_send_behavior_t send_behavior_;

    rt_event_t event;  // Minipc在线事件
};

extern Minipc minipc;

#endif
