#include "stm32f1xx_hal.h"
#include "pid.h"
#include <stdbool.h>
#ifndef __MOTOR_H_
#define __MOTOR_H_
typedef struct
{
	float real_angle_speed;
	float set_angle_speed;
	int angle;
	pid_t pid;
	int line_speed;
    bool deretion;
	uint8_t current_chassis_condition;
	uint8_t last_chassis_condition;
	int32_t CaptureNumber;
}chassis_Motor;


uint16_t  pwm_subdivision(int speed);
float Speed_tran_Pwm(float i );
void chassis_motor1_speed(int16_t speed);
void chassis_motor2_speed(int16_t speed);
void chassis_motor3_speed(int16_t speed);
void chassis_motor4_speed(int16_t speed);
void Chassis_Forward(void);           //ֱ��
void Chassis_Backward(void);          //����
void Chassis_Rightward(void);       //����
void Chassis_SRightward(void);       //��ǰת
void Chassis_Leftward(void);        //����
void Chassis_SLeftward(void);       //��ǰת
void Chassis_B_Rightward(void);     //�Һ�ת
void Chassis_B_Leftward(void);      //���ת
void Chassis_Stop(void);



#endif
