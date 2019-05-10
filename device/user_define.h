#ifndef _USER_DEFINE_H
#define _USER_DEFINE_H

#include "stdint.h"
#define byte uint8_t

#define UART_IMU_GPIO_PIN_NUMBER        30
#define TIMESTAMP_PRINT_PIN_NUMBER      28

typedef struct {
    byte imu_frame_bufer[5][32];
    byte imu_fifo_front;
    byte imu_fifo_rear;
    byte imu_fifo_size;
}T_sUartImuFrameFifo_TypeDef;

extern int put_imu_data_fifo(byte *p_uart_imu_frame, byte imu_frame_len);
extern T_sUartImuFrameFifo_TypeDef T_sUartImuFrameFifo;
#endif
