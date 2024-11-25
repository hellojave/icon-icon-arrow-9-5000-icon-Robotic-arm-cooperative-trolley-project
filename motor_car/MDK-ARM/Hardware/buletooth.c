#include "buletooth.h"
#include "motor.h"
BlueTooth bluetooth;
extern chassis_Motor chassis_motor[4];
extern  uint8_t chassis__flag_1;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart==&huart4)
	{
	   bluetooth.Serial_Flag=1;
	
		 HAL_UART_Receive_IT(&huart4,&bluetooth.RxByte,1);
	}

}
bool Serial_GetRxFlag(void)
{
  if(bluetooth.Serial_Flag==1)
	{
	    bluetooth.Serial_Flag=0;
			return 1;
	}

  return 0;
}
void chassis_movememt(void)	
{
		bluetooth.movement_move=bluetooth.RxByte;
		switch(bluetooth.movement_move)
		{
		
		  case Forward:
				Chassis_Forward();
				break;
			case Backward:
				Chassis_Backward();
				break;
			case Rightward:
				Chassis_Rightward();
				break;
			case Leftward:
				Chassis_Leftward();
				break;
			case SRightward:
				Chassis_SRightward();
			    break;
			case SLeftward:
				Chassis_SLeftward();
				break;	
           case B_Rightward:
				Chassis_B_Rightward() ; 
				break;
           case B_Leftward:
				Chassis_B_Leftward() ;
				break;
			case Stop:
				 clear_PID();
				Chassis_Stop();
			     break;
			case SoftReset:
				Stm32_SoftReset();
			     break;
			default:
				 clear_PID();
				Chassis_Stop();
				break;
	}
}
void clear_PID(void)
{
	int i=0;
	for(i=0;i<4;i++)
	{   chassis_motor[i].pid.error[0]=0;
		chassis_motor[i].pid.error[1]=0;
		chassis_motor[i].pid.error[2]=0;
	    chassis_motor[i].pid.pout=0;
	    chassis_motor[i].pid.iout=0;
	    chassis_motor[i].pid.dout=0;
	    chassis_motor[i].pid. out=0;    
	 }
	 chassis__flag_1=0;
}
void Stm32_SoftReset(void)
 {
   __set_FAULTMASK(1);//禁止所有的可屏蔽中断
   NVIC_SystemReset();//软件复位
 }
 
 
 
 