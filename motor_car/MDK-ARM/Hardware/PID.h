#ifndef __PID_H
#define __PID_H
typedef enum
{
	INIT,
	DONE,
}INIT_STATUS;

enum
{
	NOW_ERR = 0,
	LAST_ERR,
	LLAST_ERR,
};

typedef struct pid
{
	float set;
	float get;
	float error[3];
  
	float kp;
	float ki;
	float kd;
	
	float pout;
	float iout;
	float dout;
	float out; 
	
	
	int32_t maxout;
	int32_t integral_limit;
	float output_deadband;//����
	
	void (*f_pid_init)(struct pid *pid_t,
					   float p,
					   float i,
					   float d,
					   int32_t max_out,
					   int32_t integral_limit);
	void (*f_pid_reset)(struct pid *pid_t,
						float p,
						float i,
						float d);
}pid_t;


float pid_calc(pid_t *pid,float get,float set);
float fuzzy_pid_calc(pid_t *pid, float get, float set);

void PID_Struct_Init(pid_t *pid,
					 float p,
					 float i,
					 float d,
					 int32_t max_out,
					 int32_t integral_limit,
					 INIT_STATUS init_status);
#endif									 
