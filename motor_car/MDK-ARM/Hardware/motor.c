#include "stm32f1xx_hal.h"
#include "tim.h"
#include <stdlib.h>
#include "PID.h"
extern INIT_STATUS init_status;
uint8_t chassis__flag_1=0;
uint16_t Duty;
int i_chassis=0;
int i_stop=0;
chassis_Motor chassis_motor[4];
uint16_t  pwm_subdivision(int speed)     //PWM限幅
{
 
	if(speed>999)
	{
		speed=999;
	}
	 
	 return speed;
}

float Speed_tran_Pwm(float i )
	
{
	return (float)(1.73* i);
}


void chassis_motor1_speed(int16_t speed)
{
	  if(speed>0)
		{
    	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,pwm_subdivision(abs(speed)));
		HAL_GPIO_WritePin(motor1_gpio1_GPIO_Port,motor1_gpio1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor1_gpio2_GPIO_Port,motor1_gpio2_Pin,GPIO_PIN_SET);
		}
		else
		{
	    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,pwm_subdivision(abs(speed)));
		HAL_GPIO_WritePin(motor1_gpio1_GPIO_Port,motor1_gpio1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor1_gpio2_GPIO_Port,motor1_gpio2_Pin,GPIO_PIN_RESET);
		
		}
		
}
void chassis_motor2_speed(int16_t speed)
{
	  if(speed>0)
		{
				__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,pwm_subdivision(abs(speed)));
				HAL_GPIO_WritePin(motor2_gpio1_GPIO_Port,motor2_gpio1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(motor2_gpio2_GPIO_Port,motor2_gpio2_Pin,GPIO_PIN_SET);
		}
		else
		{
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,pwm_subdivision(abs(speed)));
		HAL_GPIO_WritePin(motor2_gpio1_GPIO_Port,motor2_gpio1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor2_gpio2_GPIO_Port,motor2_gpio2_Pin,GPIO_PIN_RESET);
		
		}
		
}
void chassis_motor3_speed(int16_t speed)
{
	  if(speed>0)
		{
        __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,pwm_subdivision(abs(speed)));
		HAL_GPIO_WritePin(motor3_gpio1_GPIO_Port,motor3_gpio1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor3_gpio2_GPIO_Port,motor3_gpio2_Pin,GPIO_PIN_SET);
		}
		else
		{
			    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,pwm_subdivision(abs(speed)));
		HAL_GPIO_WritePin(motor3_gpio1_GPIO_Port,motor3_gpio1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor3_gpio2_GPIO_Port,motor3_gpio2_Pin,GPIO_PIN_RESET);
		
		}
		
}
void chassis_motor4_speed(int16_t speed)
{
	  if(speed>0)
		{ 
           __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,pwm_subdivision(abs(speed)));     
		      HAL_GPIO_WritePin(motor4_gpio1_GPIO_Port,motor4_gpio1_Pin,GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(motor4_gpio2_GPIO_Port,motor4_gpio2_Pin,GPIO_PIN_SET);
		}
		else
		{
			    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,pwm_subdivision(abs(speed)));
		      HAL_GPIO_WritePin(motor4_gpio1_GPIO_Port,motor4_gpio1_Pin,GPIO_PIN_SET);
		      HAL_GPIO_WritePin(motor4_gpio2_GPIO_Port,motor4_gpio2_Pin,GPIO_PIN_RESET);
		
		}
		
}
void Chassis_Forward(void)  //400
{ 
	if(chassis__flag_1==0)
	{
		for(i_chassis=0;i_chassis<8000;i_chassis++)            
	 {
			chassis_motor[0].set_angle_speed=-0.05*i_chassis;
			chassis_motor[1].set_angle_speed=-0.05*i_chassis;
			chassis_motor[2].set_angle_speed=-0.05*i_chassis;
			chassis_motor[3].set_angle_speed=-0.05*i_chassis;
      chassis_motor1_speed(Speed_tran_Pwm(chassis_motor[0].set_angle_speed));
	  chassis_motor2_speed(Speed_tran_Pwm(chassis_motor[1].set_angle_speed));
	  chassis_motor3_speed(Speed_tran_Pwm(chassis_motor[2].set_angle_speed));
	  chassis_motor4_speed(Speed_tran_Pwm(chassis_motor[3].set_angle_speed));
	 }
	chassis__flag_1=1;
	}
	  chassis_motor1_speed(pid_calc(&chassis_motor[0].pid,Speed_tran_Pwm(chassis_motor[0].real_angle_speed),Speed_tran_Pwm(chassis_motor[0].set_angle_speed)));
	  chassis_motor2_speed(pid_calc(&chassis_motor[1].pid,Speed_tran_Pwm(chassis_motor[1].real_angle_speed),Speed_tran_Pwm(chassis_motor[1].set_angle_speed)));
	  chassis_motor3_speed(pid_calc(&chassis_motor[2].pid,Speed_tran_Pwm(chassis_motor[2].real_angle_speed),Speed_tran_Pwm(chassis_motor[2].set_angle_speed)));
	  chassis_motor4_speed(pid_calc(&chassis_motor[3].pid,Speed_tran_Pwm(chassis_motor[3].real_angle_speed),Speed_tran_Pwm(chassis_motor[3].set_angle_speed)));

	
}
void Chassis_Backward(void)                  //370
{  
	if(chassis__flag_1==0)
	{
		for(i_chassis=0;i_chassis<8000;i_chassis++)            
	 {
			chassis_motor[0].set_angle_speed=0.0463*i_chassis;
			chassis_motor[1].set_angle_speed=0.0463*i_chassis;
			chassis_motor[2].set_angle_speed=0.0463*i_chassis;
			chassis_motor[3].set_angle_speed=0.0463*i_chassis;
      chassis_motor1_speed(Speed_tran_Pwm(chassis_motor[0].set_angle_speed));
	  chassis_motor2_speed(Speed_tran_Pwm(chassis_motor[1].set_angle_speed));
	  chassis_motor3_speed(Speed_tran_Pwm(chassis_motor[2].set_angle_speed));
	  chassis_motor4_speed(Speed_tran_Pwm(chassis_motor[3].set_angle_speed));
	 }
	chassis__flag_1=1;
	}

	  chassis_motor1_speed(pid_calc(&chassis_motor[0].pid,Speed_tran_Pwm(chassis_motor[0].real_angle_speed),Speed_tran_Pwm(chassis_motor[0].set_angle_speed)));
	  chassis_motor2_speed(pid_calc(&chassis_motor[1].pid,Speed_tran_Pwm(chassis_motor[1].real_angle_speed),Speed_tran_Pwm(chassis_motor[1].set_angle_speed)));
	  chassis_motor3_speed(pid_calc(&chassis_motor[2].pid,Speed_tran_Pwm(chassis_motor[2].real_angle_speed),Speed_tran_Pwm(chassis_motor[2].set_angle_speed)));
	  chassis_motor4_speed(pid_calc(&chassis_motor[3].pid,Speed_tran_Pwm(chassis_motor[3].real_angle_speed),Speed_tran_Pwm(chassis_motor[3].set_angle_speed)));
	 
}
void Chassis_Rightward(void)  //右旋350
{

		    chassis_motor[0].set_angle_speed=-350;
			chassis_motor[1].set_angle_speed=350;
			chassis_motor[2].set_angle_speed=-350;
			chassis_motor[3].set_angle_speed=350;		
	  chassis_motor1_speed(pid_calc(&chassis_motor[0].pid,Speed_tran_Pwm(chassis_motor[0].real_angle_speed),Speed_tran_Pwm(chassis_motor[0].set_angle_speed)));
	  chassis_motor2_speed(pid_calc(&chassis_motor[1].pid,Speed_tran_Pwm(chassis_motor[1].real_angle_speed),Speed_tran_Pwm(chassis_motor[1].set_angle_speed)));
	  chassis_motor3_speed(pid_calc(&chassis_motor[2].pid,Speed_tran_Pwm(chassis_motor[2].real_angle_speed),Speed_tran_Pwm(chassis_motor[2].set_angle_speed)));
	  chassis_motor4_speed(pid_calc(&chassis_motor[3].pid,Speed_tran_Pwm(chassis_motor[3].real_angle_speed),Speed_tran_Pwm(chassis_motor[3].set_angle_speed)));
		
}
void Chassis_SRightward(void)  //前右转
{
	if(chassis__flag_1==0)
	{
		for(i_chassis=0;i_chassis<8000;i_chassis++)            
	 {
			chassis_motor[0].set_angle_speed=-0.0463*i_chassis;             //370
			chassis_motor[1].set_angle_speed=-0.0187*i_chassis;              //150
			chassis_motor[2].set_angle_speed=-0.0463*i_chassis;
			chassis_motor[3].set_angle_speed=-0.0187*i_chassis;
      chassis_motor1_speed(Speed_tran_Pwm(chassis_motor[0].set_angle_speed));
	  chassis_motor2_speed(Speed_tran_Pwm(chassis_motor[1].set_angle_speed));
	  chassis_motor3_speed(Speed_tran_Pwm(chassis_motor[2].set_angle_speed));
	  chassis_motor4_speed(Speed_tran_Pwm(chassis_motor[3].set_angle_speed));
	 }
	chassis__flag_1=1;
	}	
	  chassis_motor1_speed(pid_calc(&chassis_motor[0].pid,Speed_tran_Pwm(chassis_motor[0].real_angle_speed),Speed_tran_Pwm(chassis_motor[0].set_angle_speed)));
	  chassis_motor2_speed(pid_calc(&chassis_motor[1].pid,Speed_tran_Pwm(chassis_motor[1].real_angle_speed),Speed_tran_Pwm(chassis_motor[1].set_angle_speed)));
	  chassis_motor3_speed(pid_calc(&chassis_motor[2].pid,Speed_tran_Pwm(chassis_motor[2].real_angle_speed),Speed_tran_Pwm(chassis_motor[2].set_angle_speed)));
	  chassis_motor4_speed(pid_calc(&chassis_motor[3].pid,Speed_tran_Pwm(chassis_motor[3].real_angle_speed),Speed_tran_Pwm(chassis_motor[3].set_angle_speed)));
		
}
void Chassis_Leftward(void)        //左旋350
{          

	        chassis_motor[0].set_angle_speed=350;
			chassis_motor[1].set_angle_speed=-350;
			chassis_motor[2].set_angle_speed=350;
			chassis_motor[3].set_angle_speed=-350;
	  chassis_motor1_speed(pid_calc(&chassis_motor[0].pid,Speed_tran_Pwm(chassis_motor[0].real_angle_speed),Speed_tran_Pwm(chassis_motor[0].set_angle_speed)));
	  chassis_motor2_speed(pid_calc(&chassis_motor[1].pid,Speed_tran_Pwm(chassis_motor[1].real_angle_speed),Speed_tran_Pwm(chassis_motor[1].set_angle_speed)));
	  chassis_motor3_speed(pid_calc(&chassis_motor[2].pid,Speed_tran_Pwm(chassis_motor[2].real_angle_speed),Speed_tran_Pwm(chassis_motor[2].set_angle_speed)));
	  chassis_motor4_speed(pid_calc(&chassis_motor[3].pid,Speed_tran_Pwm(chassis_motor[3].real_angle_speed),Speed_tran_Pwm(chassis_motor[3].set_angle_speed)));
}

void Chassis_SLeftward(void)       //前左转
{    
    if(chassis__flag_1==0)
	{
		for(i_chassis=0;i_chassis<8000;i_chassis++)            
	 {
			chassis_motor[0].set_angle_speed=-0.0187*i_chassis;    //150
			chassis_motor[1].set_angle_speed=-0.0463*i_chassis;     // 370
			chassis_motor[2].set_angle_speed=-0.0187*i_chassis;
			chassis_motor[3].set_angle_speed=-0.0463*i_chassis;
      chassis_motor1_speed(Speed_tran_Pwm(chassis_motor[0].set_angle_speed));
	  chassis_motor2_speed(Speed_tran_Pwm(chassis_motor[1].set_angle_speed));
	  chassis_motor3_speed(Speed_tran_Pwm(chassis_motor[2].set_angle_speed));
	  chassis_motor4_speed(Speed_tran_Pwm(chassis_motor[3].set_angle_speed));
	 }
	chassis__flag_1=1;
	}	
	  chassis_motor1_speed(pid_calc(&chassis_motor[0].pid,Speed_tran_Pwm(chassis_motor[0].real_angle_speed),Speed_tran_Pwm(chassis_motor[0].set_angle_speed)));
	  chassis_motor2_speed(pid_calc(&chassis_motor[1].pid,Speed_tran_Pwm(chassis_motor[1].real_angle_speed),Speed_tran_Pwm(chassis_motor[1].set_angle_speed)));
	  chassis_motor3_speed(pid_calc(&chassis_motor[2].pid,Speed_tran_Pwm(chassis_motor[2].real_angle_speed),Speed_tran_Pwm(chassis_motor[2].set_angle_speed)));
	  chassis_motor4_speed(pid_calc(&chassis_motor[3].pid,Speed_tran_Pwm(chassis_motor[3].real_angle_speed),Speed_tran_Pwm(chassis_motor[3].set_angle_speed)));
}


void Chassis_B_Rightward(void)  //后右转
{
	if(chassis__flag_1==0)
	{
		for(i_chassis=0;i_chassis<8000;i_chassis++)            
	 {
			chassis_motor[0].set_angle_speed=0.0463*i_chassis;             //370
			chassis_motor[1].set_angle_speed=0.0187*i_chassis;              //150
			chassis_motor[2].set_angle_speed=0.0463*i_chassis;
			chassis_motor[3].set_angle_speed=0.0187*i_chassis;
      chassis_motor1_speed(Speed_tran_Pwm(chassis_motor[0].set_angle_speed));
	  chassis_motor2_speed(Speed_tran_Pwm(chassis_motor[1].set_angle_speed));
	  chassis_motor3_speed(Speed_tran_Pwm(chassis_motor[2].set_angle_speed));
	  chassis_motor4_speed(Speed_tran_Pwm(chassis_motor[3].set_angle_speed));
	 }
	chassis__flag_1=1;
	}	
	  chassis_motor1_speed(pid_calc(&chassis_motor[0].pid,Speed_tran_Pwm(chassis_motor[0].real_angle_speed),Speed_tran_Pwm(chassis_motor[0].set_angle_speed)));
	  chassis_motor2_speed(pid_calc(&chassis_motor[1].pid,Speed_tran_Pwm(chassis_motor[1].real_angle_speed),Speed_tran_Pwm(chassis_motor[1].set_angle_speed)));
	  chassis_motor3_speed(pid_calc(&chassis_motor[2].pid,Speed_tran_Pwm(chassis_motor[2].real_angle_speed),Speed_tran_Pwm(chassis_motor[2].set_angle_speed)));
	  chassis_motor4_speed(pid_calc(&chassis_motor[3].pid,Speed_tran_Pwm(chassis_motor[3].real_angle_speed),Speed_tran_Pwm(chassis_motor[3].set_angle_speed)));
		
}

void Chassis_B_Leftward(void)       //后左转
{    
    if(chassis__flag_1==0)
	{
		for(i_chassis=0;i_chassis<8000;i_chassis++)            
	 {
			chassis_motor[0].set_angle_speed=0.0187*i_chassis;    //150
			chassis_motor[1].set_angle_speed=0.0463*i_chassis;     // 370
			chassis_motor[2].set_angle_speed=0.0187*i_chassis;
			chassis_motor[3].set_angle_speed=0.0463*i_chassis;
      chassis_motor1_speed(Speed_tran_Pwm(chassis_motor[0].set_angle_speed));
	  chassis_motor2_speed(Speed_tran_Pwm(chassis_motor[1].set_angle_speed));
	  chassis_motor3_speed(Speed_tran_Pwm(chassis_motor[2].set_angle_speed));
	  chassis_motor4_speed(Speed_tran_Pwm(chassis_motor[3].set_angle_speed));
	 }
	chassis__flag_1=1;
	}	
	  chassis_motor1_speed(pid_calc(&chassis_motor[0].pid,Speed_tran_Pwm(chassis_motor[0].real_angle_speed),Speed_tran_Pwm(chassis_motor[0].set_angle_speed)));
	  chassis_motor2_speed(pid_calc(&chassis_motor[1].pid,Speed_tran_Pwm(chassis_motor[1].real_angle_speed),Speed_tran_Pwm(chassis_motor[1].set_angle_speed)));
	  chassis_motor3_speed(pid_calc(&chassis_motor[2].pid,Speed_tran_Pwm(chassis_motor[2].real_angle_speed),Speed_tran_Pwm(chassis_motor[2].set_angle_speed)));
	  chassis_motor4_speed(pid_calc(&chassis_motor[3].pid,Speed_tran_Pwm(chassis_motor[3].real_angle_speed),Speed_tran_Pwm(chassis_motor[3].set_angle_speed)));
}

void Chassis_Stop(void)
{
		 chassis_motor1_speed(0);
	     chassis_motor2_speed(0);
		 chassis_motor3_speed(0);
		 chassis_motor4_speed(0);
}

