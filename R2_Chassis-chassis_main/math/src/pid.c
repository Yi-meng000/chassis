#include "pid.h"
/**
 * @brief 
 * 
 * @param pid 
 * @param kp 
 * @param ki 
 * @param kd 
 * @param set 
 * @return int32_t 返回pid输出量
 */
void PID_Init(PIDType *pid, float kp, float ki, float kd, uint8_t mode)
{
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;

    pid->SetVal = 0;
    pid->CurVal = 0;
    
    pid->mode = mode;
    pid->err[0] = 0;
    pid->err[1] = 0;
    pid->err[2] = 0;
    pid->output = 0;
}
float PID_Caculate(PIDType *pid,float Input,float Target)
{

    pid->err[0] = Target - Input;
    switch (pid->mode)
    {
		case PIDINC:
        pid->output = pid->KP * (pid->err[0] - pid->err[1]) + pid->KI * pid->err[0] + pid->KD * (pid->err[0] - 2 * pid->err[1] + pid->err[2]);
        pid->err[2] = pid->err[1];
        pid->err[1] = pid->err[0]; // 顺序不能乱
        break;
    case PIDPOS:
        pid->err[2] = 0.5f * pid->err[0] + 0.5f * pid->err[2];
        if (ABS(pid->err[2]) > 100)
            pid->err[2] = GetSign(pid->err[2]) * 100.f;
        pid->output = pid->KP * pid->err[0] + pid->KI * pid->err[2] + pid->KD * (pid->err[0] - pid->err[1]);
        pid->err[1] = pid->err[0];
        break;
    default:
        break;
    }

    return pid->output;
}
void vector2fPIDInit(Vector2fPID *pid,float *param,uint8_t mode)
{
    pid->kp = param[0];
    pid->ki = param[1];
    pid->kd = param[2];

    pid->output.x = 0;
    pid->output.y = 0;
    
    for(int i = 0 ; i < 3 ; i++)
    {
      pid->err[i].x = 0;
      pid->err[i].y = 0;
    }
    pid->mode = mode;
}
vector2d vector2fPIDOperation(Vector2fPID *pid)
{
  pid->err[0] = Vector_Minus(pid->target , pid->input);
  switch (pid->mode)
  {
  case  PIDPOS:
    pid->err[2] = Vector_Add(Vector_MultiplyNum(pid->err[2], 0.5f),Vector_MultiplyNum(pid->err[0],0.5));
    
    if(ABS(pid->err[2].x) > 1000)
      pid->err[2].x = GetSign(pid->err[2].x) * 1000;
    if(ABS(pid->err[2].y) > 1000)
      pid->err[2].y = GetSign(pid->err[2].y) * 1000;
    pid->output = Vector_MultiplyNum(pid->err[0] ,pid->kp);
    pid->output = Vector_Add(pid->output,Vector_MultiplyNum(pid->err[2],pid->ki));
    pid->output = Vector_Add(pid->output,Vector_MultiplyNum(Vector_Minus(pid->err[1], pid->err[0]),pid->kd));

    pid->err[1] = pid->err[0];
      /* code */
    break;
  case PIDINC:
		pid->output = Vector_MultiplyNum(Vector_Minus(pid->err[1], pid->err[0]), pid->kp);
		pid->output = Vector_Add(pid->output, Vector_MultiplyNum(pid->err[0], pid->ki));
		pid->output = Vector_Add(pid->output, Vector_MultiplyNum(Vector_Minus(Vector_MultiplyNum(pid->err[1], 2), Vector_Add(pid->err[0], pid->err[2])), pid->kd));

		pid->err[2] = pid->err[1];
		pid->err[1] = pid->err[0];
    break;
  default:
    break;
  }
  return pid->output;
}
