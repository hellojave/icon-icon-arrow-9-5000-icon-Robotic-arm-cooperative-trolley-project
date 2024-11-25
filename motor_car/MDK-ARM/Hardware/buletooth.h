#ifndef __BULEBOOTH_H
#define __BULEBOOTH_H
#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "motor.h"
#define Forward   'A'
#define Backward  'B'
#define Rightward 'C'
#define Leftward  'D'
#define Stop       'E'
#define SRightward 'F'
#define SLeftward  'G'
#define B_Rightward 'H'
#define B_Leftward  'I'
#define SoftReset  'J'
typedef struct
{
  uint8_t RxByte;
	uint8_t movement_move;
	bool Serial_Flag;
  	
}BlueTooth;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

bool Serial_GetRxFlag(void);
void clear_PID(void);
void chassis_movememt(void)	;
void Stm32_SoftReset(void);
#endif

