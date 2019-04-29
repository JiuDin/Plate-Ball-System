

#ifndef  _SERVOCONTROL_H
#define  _SERVOCONTROL_H

typedef struct
{ 
  int Ek;                       //当前误差
  int Ek1;                      //前一次误差 e(k-1)
  int Ek2;                      //再前一次误差 e(k-2)
  int LocSum;                   //累计积分位置
  int PIDLoc;
}PID_LocTypeDef;

typedef struct
{
  int KP;
  float KI;
  int KD;

}PID_Parameter;

typedef struct
{
int now_x;
int now_y;
int last_x;
int last_y;
}Ball_Position;


void Servo_aim_X(int set_x, PID_LocTypeDef *PID);
void Servo_aim_Y(int set_y, PID_LocTypeDef *PID);
void Servo_ctl_X(int set_x, PID_LocTypeDef *PID);
void Servo_ctl_Y(int set_y, PID_LocTypeDef *PID);
void jude(uint16 max_speed);
void  derection(uint16 max_speed);
void Control_aim(int aim,int times,int mode_after);
void Control_transition(int transition,int aim,int mode_after);
void Servo_Out(void) ;
void Servo_Plan(void);
#endif


