#include "include.h"
#include "ServoControl.h"

#define I_integral 100


#define X_max  350
#define Y_max  350
extern int ZERO_X;//零位  
extern int ZERO_Y;//零位  

extern uint16 outcount;
extern int mode;
extern uint16 comein_count;//到达目标点判定
extern int Position[25][2];
extern long int Time_ms;//MS计数

Ball_Position Ball;
PID_LocTypeDef PID_X;
PID_LocTypeDef PID_Y;

PID_Parameter X_Parameter={14,0,35};//0.25  40
PID_Parameter Y_Parameter={15,0,35};




 int PID_x=0;    //PID的输出，pwm的输入
 int PID_y=0;  
 int PID_lastx=0;
 int PID_lasty=0;
 int Out_finalX=0;
 int Out_finalY=0;
 

/************************************************
函数名称 ： PID_Loc
功    能 ： PID位置(Location)计算
参    数 ： SetValue ------ 设置值(期望值)
            ActualValue --- 实际值(反馈值)
            PID ----------- PID数据结构
返 回 值 ： PIDLoc -------- PID位置
作    者 ： strongerHuang
*************************************************/




void Servo_ctl_X(int Set_X, PID_LocTypeDef *PID)
{
  
  PID->Ek = Set_X - Ball.now_x ;
  if(PID->Ek>=-1&&PID->Ek<=1)
    PID->Ek=0;
  PID->LocSum += PID->Ek;                         //累计误差
// if( PID->Ek<=3&&PID->Ek>=-3)
// {
//   X_Parameter.KI=0.5;
// }
// else
//    X_Parameter.KI=0;
//    
  if(  PID->LocSum>I_integral)     //积分限幅
        PID->LocSum=I_integral;
  if(  PID->LocSum<-I_integral)
        PID->LocSum=-I_integral;
  PID->PIDLoc = X_Parameter.KP * PID->Ek + (int)(X_Parameter.KI * PID->LocSum) + X_Parameter.KD * (Ball.now_x  - Ball.last_x );
  PID->Ek1 = PID->Ek; 
  
  Ball.last_x =Ball.now_x ;
  
  PID_lastx= PID_x;
    
  PID_x=PID->PIDLoc;
  
  if(PID_x>X_max)
    PID_x=X_max;
    else if(PID_x<-X_max)
      PID_x=-X_max;
 
}

void Servo_ctl_Y(int Set_Y, PID_LocTypeDef *PID)
{
                               //位置
 
  PID->Ek = Set_Y - Ball.now_y;
  if(PID->Ek>=-1&&PID->Ek<=1)
    PID->Ek=0;
  PID->LocSum += PID->Ek;                         //累计误差
//  if( PID->Ek<=3&&PID->Ek>=-3)
// {
//   Y_Parameter.KI=0.5;
// }
// else
//   Y_Parameter.KI=0;
   if(  PID->LocSum>I_integral)
        PID->LocSum=I_integral;
  if(  PID->LocSum<-I_integral)
        PID->LocSum=-I_integral;
  
  PID->PIDLoc = Y_Parameter.KP * PID->Ek + (int)(Y_Parameter.KI * PID->LocSum) + Y_Parameter.KD * (Ball.now_y  - Ball.last_y );
  PID->Ek1 = PID->Ek; 
  
  Ball.last_y=Ball.now_y;
  
  PID_lasty=PID_y;
  
  PID_y=PID->PIDLoc;
  
  if(PID_y>Y_max)
    PID_y=Y_max;
    else if(PID_y<-Y_max)
      PID_y=-Y_max;

}

//运动计划
//用于给定过渡点及判定是否到达目标点
void Servo_Plan(void)
{
  /************模式1************************/
      
  if(mode==1)
  {    
    if(Time_ms<6000)
        {
          Servo_ctl_X(Position[1][0],&PID_X);
          Servo_ctl_Y(Position[1][1],&PID_Y);
        } 
        else 
        {
          Time_ms=0;
        }
  }
  
  /**************模式2***************************/
  if(mode==2||mode==22)
  {
  if(mode==2)  //模式2.1
  {
   //  Control_transition(10,5,22);
     comein_count=0;
    X_Parameter.KD=35;
    Y_Parameter.KD=35;
       ZERO_X=1370;//零位  1460  1100-1900 低至高
       ZERO_Y=1650;
     Servo_ctl_X(Position[10][0],&PID_X);
     Servo_ctl_Y(Position[10][1],&PID_Y);
    if((Ball.now_x-Position[5][0]<=2&&Ball.now_x-Position[5][0]>=-5)&&\
        (Ball.now_y-Position[5][1]<=5&&Ball.now_y-Position[5][1]>=-5))   //到达5点附近区域
    {
       mode=22;
 
    }

  
  }
  
  if(mode==22) //模式2.2
  {     X_Parameter.KP=25;
        Y_Parameter.KP=25;
    Servo_ctl_X(Position[5][0],&PID_X);                    //
    Servo_ctl_Y(Position[5][1],&PID_Y);
        
  if((Ball.now_x-Position[5][0]<=2&&Ball.now_x-Position[5][0]>=-2)&&\
        (Ball.now_y-Position[5][1]<=2&&Ball.now_y-Position[5][1]>=-2))   //到达5点附近,开始计数
        {
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[5][0],&PID_X);                    //
        Servo_ctl_Y(Position[5][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达5点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[5][0],&PID_X);                    //
          Servo_ctl_Y(Position[5][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<3000)  //在5点停3S
          {
            Servo_ctl_X(Position[5][0],&PID_X);                    //
            Servo_ctl_Y(Position[5][1],&PID_Y);
          }
          else 
          {
           //空空
          }
        }
    
    
    
   }
  
  }


 /**************模式3**********************/ 
 
  if(mode==3||mode==32||mode==33||mode==34)
  {
   if(mode==3)
   {    comein_count=0;
       ZERO_X=1360;
       ZERO_Y=1670;
       Servo_ctl_X(Position[11][0],&PID_X);
       Servo_ctl_Y(Position[11][1],&PID_Y);
    if((Ball.now_x-Position[4][0]<=3&&Ball.now_x-Position[4][0]>=-3)&&\
        (Ball.now_y-Position[4][1]<=3&&Ball.now_y-Position[4][1]>=0))   //到达4点附近区域
      {
       mode=32;
      }   
   }
  
  
  if(mode==32) //模式3.2
  {   
    
     ZERO_X=1370;//零位  1460  1100-1900 低至高
       ZERO_Y=1640;
        X_Parameter.KP=16;
        Y_Parameter.KP=16;
        Servo_ctl_X(Position[4][0],&PID_X);                    //
        Servo_ctl_Y(Position[4][1],&PID_Y);
        
  if((Ball.now_x-Position[4][0]<=3&&Ball.now_x-Position[4][0]>=-3)&&\
        (Ball.now_y-Position[4][1]<=3&&Ball.now_y-Position[4][1]>=-3))   //到达4点附近,开始计数
        {
        comein_count++; 
         X_Parameter.KP=20;
        Y_Parameter.KP=20;
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[4][0],&PID_X);                    //
        Servo_ctl_Y(Position[4][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达4点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[4][0],&PID_X);                    //
          Servo_ctl_Y(Position[4][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<3000)  //在4点停3S
          {
            Servo_ctl_X(Position[4][0],&PID_X);                    //
            Servo_ctl_Y(Position[4][1],&PID_Y);
          }
          else 
          {
           mode=33;
          }
        }
    }
  
  
    if(mode==33)
   {    ZERO_X=1370;
         ZERO_Y=1650;
          X_Parameter.KP=14;
        Y_Parameter.KP=15;
      Servo_ctl_X(Position[12][0]-1,&PID_X);
     Servo_ctl_Y(Position[12][1],&PID_Y);
    if((Ball.now_x-Position[5][0]<=1&&Ball.now_x-Position[5][0]>=-3)&&\
        (Ball.now_y-Position[5][1]<=3&&Ball.now_y-Position[5][1]>=-3))   //到达5点附近区域
      {
       mode=34;
      } 
    
   }

  if(mode==34) //模式3.2
  {
    
        Servo_ctl_X(Position[5][0],&PID_X);                    //
        Servo_ctl_Y(Position[5][1],&PID_Y);
        
  if((Ball.now_x-Position[5][0]<=2&&Ball.now_x-Position[5][0]>=-2)&&\
        (Ball.now_y-Position[5][1]<=2&&Ball.now_y-Position[5][1]>=-2))   //到达4点附近,开始计数
        {
        comein_count++; 
         X_Parameter.KP=18;
        Y_Parameter.KP=18;
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[5][0],&PID_X);                    //
        Servo_ctl_Y(Position[5][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达4点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[5][0],&PID_X);                    //
          Servo_ctl_Y(Position[5][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<3000)  //在5点停3S
          {
            Servo_ctl_X(Position[5][0],&PID_X);                    //
            Servo_ctl_Y(Position[5][1],&PID_Y);
          }
          else 
          {
          //
          }
        }
    }
  
  }
   /******************模式4********************/

  if(mode==4||mode==41||mode==42||mode==43||mode==44)//调节9点的x y 或着5-9中间点 控制其最后的停止点
  {
  if(mode==4)  //模式2.1
  { 
     comein_count=0;
     ZERO_X=1400;
     ZERO_Y=1670; 
     
     Servo_ctl_X(Position[10][0]-3,&PID_X);
     Servo_ctl_Y(Position[10][1]+3,&PID_Y);
    if((Ball.now_x-Position[5][0]<=6&&Ball.now_x-Position[5][0]>=-6)&&\
        (Ball.now_y-Position[5][1]<=6&&Ball.now_y-Position[5][1]>=-6))   //到达5点附近区域
    {
       mode=42;
 
    }
     if((Ball.now_x-Position[15][0]<=4&&Ball.now_x-Position[15][0]>=-4)&&\
        (Ball.now_y-Position[15][1]<=4&&Ball.now_y-Position[15][1]>=-4))   //到达5点附近区域
    {
       mode=42;
     
    }
     if((Ball.now_x-Position[9][0]<=5&&Ball.now_x-Position[9][0]>=-5)&&\
        (Ball.now_y-Position[9][1]<=5&&Ball.now_y-Position[9][1]>=-5))   //到达5点附近区域
    {
       mode=44;
     
    }
  
  }
  
  if(mode==42) //模式2.2
   {
    mode=43;
    ZERO_Y=1590;
     }
   if(mode==43)  //模式2.1
  {
     Servo_ctl_X(Position[15][0]+6,&PID_X);
     Servo_ctl_Y(Position[15][1]-6,&PID_Y);
    if((Ball.now_x-Position[9][0]<=0&&Ball.now_x-Position[9][0]>=-4)&&\
        (Ball.now_y-Position[9][1]<=4&&Ball.now_y-Position[9][1]>=-4))   //到达5点附近区域
    {
       mode=44;
    
    }

  
  }
  
  if(mode==44) //模式4.4
  {
       ZERO_Y=1630;
      
        Servo_ctl_X(Position[9][0],&PID_X);                    //
        Servo_ctl_Y(Position[9][1],&PID_Y);
  if((Ball.now_x-Position[9][0]<=2&&Ball.now_x-Position[9][0]>=-2)&&\
        (Ball.now_y-Position[9][1]<=2&&Ball.now_y-Position[9][1]>=-2))   //到达9点附近,开始计数
        {
        X_Parameter.KP=15;
        Y_Parameter.KP=15;
        comein_count++; 
        
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[9][0],&PID_X);                    //
        Servo_ctl_Y(Position[9][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达9点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[9][0],&PID_X);                    //
          Servo_ctl_Y(Position[9][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<3000)  //在9点停3S
          {
            Servo_ctl_X(Position[9][0],&PID_X);                    //
            Servo_ctl_Y(Position[9][1],&PID_Y);
          }
          else 
          {
            X_Parameter.KI=0;
        Y_Parameter.KI=0;
          }
        }
    }
  
  }
  
  /********************模式5**********************/
  
  if(mode==5||mode==51||mode==52||mode==53||mode==54||mode==55)
  {
  if(mode==5)
  {
     ZERO_X=1370;
     ZERO_Y=1640;  
     comein_count=0;
      Servo_ctl_X(Position[17][0],&PID_X);
       Servo_ctl_Y(Position[17][1],&PID_Y);
   if((Ball.now_x-Position[2][0]<=0&&Ball.now_x-Position[2][0]>=-2)&&(Ball.now_y-Position[2][1]<=3&&Ball.now_y-Position[2][1]>=-3)) 
      //到达4点附近区域       <=0极佳
      {
       mode=51;
      }   
  }
  if(mode==51)
  {     X_Parameter.KP=16;
        Y_Parameter.KP=16;
        Servo_ctl_X(Position[2][0],&PID_X);                    //
        Servo_ctl_Y(Position[2][1],&PID_Y);
        
  if((Ball.now_x-Position[2][0]<=3&&Ball.now_x-Position[2][0]>=-3)&&\
        (Ball.now_y-Position[2][1]<=3&&Ball.now_y-Position[2][1]>=-3))   //到达2点附近,开始计数
        {X_Parameter.KP=16;
        Y_Parameter.KP=16;
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[2][0],&PID_X);                    //
        Servo_ctl_Y(Position[2][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达4点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[2][0],&PID_X);                    //
          Servo_ctl_Y(Position[2][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<100)  //在4点停3S
          {
            Servo_ctl_X(Position[2][0],&PID_X);                    //
            Servo_ctl_Y(Position[2][1],&PID_Y);
          }
          else 
          {
           mode=52;
          }
        }
  }
  if(mode==52)
  { 
    X_Parameter.KP=15;
        Y_Parameter.KP=15;
      Servo_ctl_X(Position[24][0],&PID_X);
       Servo_ctl_Y(Position[24][1],&PID_Y);
    if((Ball.now_x-Position[6][0]<=2&&Ball.now_x-Position[6][0]>=-5)&&\
        (Ball.now_y-Position[6][1]<=5&&Ball.now_y-Position[6][1]>=-5))   //到达4点附近区域
      {
       mode=53;
       comein_count=0;
      }   
  }
  if(mode==53)
  { 
       Servo_ctl_X(Position[6][0],&PID_X);                    //
        Servo_ctl_Y(Position[6][1],&PID_Y);
        
  if((Ball.now_x-Position[6][0]<=3&&Ball.now_x-Position[6][0]>=-3)&&\
        (Ball.now_y-Position[6][1]<=3&&Ball.now_y-Position[6][1]>=-3))   //到达2点附近,开始计数
        { X_Parameter.KP=16;
        Y_Parameter.KP=16;
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[6][0],&PID_X);                    //
        Servo_ctl_Y(Position[6][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达4点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[6][0],&PID_X);                    //
          Servo_ctl_Y(Position[6][1],&PID_Y);
        }
        else
        {
 
          if(Time_ms<100)  //在4点停3S
          {
            Servo_ctl_X(Position[6][0],&PID_X);                    //
            Servo_ctl_Y(Position[6][1],&PID_Y);
          }
          else 
          {
           mode=54;

          }
        }
  }
  if(mode==54)
  { 
      ZERO_X=1370;
      ZERO_Y=1650;
       X_Parameter.KP=14;
        Y_Parameter.KP=15;
       Servo_ctl_X(Position[20][0],&PID_X);
       Servo_ctl_Y(Position[20][1],&PID_Y);
    if((Ball.now_x-Position[9][0]<=3&&Ball.now_x-Position[9][0]>=-3)&&\
        (Ball.now_y-Position[9][1]<=3&&Ball.now_y-Position[9][1]>=-1))   //到达4点附近区域
      {
       mode=55;
       comein_count==0;

      }   
  }
  if(mode==55)
  {   ZERO_X=1370;
     
        Servo_ctl_X(Position[9][0],&PID_X);                    //
        Servo_ctl_Y(Position[9][1],&PID_Y);
  if((Ball.now_x-Position[9][0]<=2&&Ball.now_x-Position[9][0]>=-2)&&\
        (Ball.now_y-Position[9][1]<=2&&Ball.now_y-Position[9][1]>=-2))   //到达9点附近,开始计数
        {
        comein_count++; 
         X_Parameter.KP=16;
        Y_Parameter.KP=16;
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[9][0],&PID_X);                    //
        Servo_ctl_Y(Position[9][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达9点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[9][0],&PID_X);                    //
          Servo_ctl_Y(Position[9][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<3000)  //在9点停3S
          {
            Servo_ctl_X(Position[9][0],&PID_X);                    //
            Servo_ctl_Y(Position[9][1],&PID_Y);
          }
          else 
          {
           //空空
          }
        }
  }
  
 
  }
  
  
  /*********************模式6***************************/
  if(mode==6||mode>=61||mode<=69||mode>=80||mode<=89)
  {
  if(mode==6)
  {
     ZERO_X=1400;
     ZERO_Y=1640;  
     comein_count=0;
      Servo_ctl_X(Position[2][0],&PID_X);
       Servo_ctl_Y(Position[2][1],&PID_Y);
   if((Ball.now_x-Position[3][0]<=3&&Ball.now_x-Position[3][0]>=-3)&&(Ball.now_y-Position[3][1]<=3&&Ball.now_y-Position[3][1]>=-3)) 
      //到达4点附近区域       <=0极佳
      {
       mode=61;
      }   
  }
   if(mode==61)
  { 
      Servo_ctl_X(Position[3][0],&PID_X);                    //
        Servo_ctl_Y(Position[3][1],&PID_Y);
        
  if((Ball.now_x-Position[3][0]<=3&&Ball.now_x-Position[3][0]>=-3)&&\
        (Ball.now_y-Position[3][1]<=3&&Ball.now_y-Position[3][1]>=-3))   //到达2点附近,开始计数
        {X_Parameter.KP=16;
        Y_Parameter.KP=16;
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[3][0],&PID_X);                    //
        Servo_ctl_Y(Position[3][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达4点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[3][0],&PID_X);                    //
          Servo_ctl_Y(Position[3][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<200)  //在4点停3S
          {
            Servo_ctl_X(Position[3][0],&PID_X);                    //
            Servo_ctl_Y(Position[3][1],&PID_Y);
          }
          else 
          {
           mode=62;
          }
        }
  }
  
  if(mode==62)
  {
        X_Parameter.KP=14;
        Y_Parameter.KP=15;
   Servo_ctl_X(Position[9][0],&PID_X);
       Servo_ctl_Y(Position[9][1],&PID_Y);
   if((Ball.now_x-Position[6][0]<=0&&Ball.now_x-Position[6][0]>=-3)&&(Ball.now_y-Position[6][1]<=3&&Ball.now_y-Position[6][1]>=-3)) 
      //到达4点附近区域       <=0极佳
      {
       mode=63;
      }   
  }
  if(mode==63)
  {
        X_Parameter.KP=16;
        Y_Parameter.KP=16;
        Servo_ctl_X(Position[9][0],&PID_X);                    //
        Servo_ctl_Y(Position[9][1],&PID_Y);
        
  if((Ball.now_x-Position[9][0]<=2&&Ball.now_x-Position[9][0]>=-2)&&\
        (Ball.now_y-Position[9][1]<=2&&Ball.now_y-Position[9][1]>=-2))   //到达2点附近,开始计数
        {
          X_Parameter.KP=16;
        Y_Parameter.KP=16;
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[9][0],&PID_X);                    //
        Servo_ctl_Y(Position[9][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达4点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[9][0],&PID_X);                    //
          Servo_ctl_Y(Position[9][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<200)  //在4点停3S
          {
            Servo_ctl_X(Position[9][0],&PID_X);                    //
            Servo_ctl_Y(Position[9][1],&PID_Y);
          }
          else 
          {
           mode=63;
          }
        }
  }
  
  if(mode==64)
  {X_Parameter.KP=14;
        Y_Parameter.KP=15;
     ZERO_X=1370;
     ZERO_Y=1640;
     Servo_ctl_X(Position[19][0],&PID_X);
       Servo_ctl_Y(Position[19][1],&PID_Y);
   if((Ball.now_x-Position[6][0]<=3&&Ball.now_x-Position[6][0]>=-3)&&(Ball.now_y-Position[6][1]<=3&&Ball.now_y-Position[6][1]>=-1)) 
      //到达4点附近区域       <=0极佳
      {
       mode=65;
      }     
  }
  if(mode==65)
  {
  
        Servo_ctl_X(Position[6][0],&PID_X);                    //
        Servo_ctl_Y(Position[6][1],&PID_Y);
        
  if((Ball.now_x-Position[6][0]<=2&&Ball.now_x-Position[6][0]>=-2)&&\
        (Ball.now_y-Position[6][1]<=2&&Ball.now_y-Position[6][1]>=-2))   //到达2点附近,开始计数
        {X_Parameter.KP=16;
        Y_Parameter.KP=16;
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[6][0],&PID_X);                    //
        Servo_ctl_Y(Position[6][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达4点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[6][0],&PID_X);                    //
          Servo_ctl_Y(Position[6][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<500)  //在4点停3S
          {
            Servo_ctl_X(Position[6][0],&PID_X);                    //
            Servo_ctl_Y(Position[6][1],&PID_Y);
          }
          else 
          {
           mode=66;
          }
        }
    
  }
  
    if(mode==66)
  {
        X_Parameter.KP=14;
        Y_Parameter.KP=15;
       Servo_ctl_X(Position[20][0],&PID_X);
       Servo_ctl_Y(Position[20][1],&PID_Y);
   if((Ball.now_x-Position[9][0]<=3&&Ball.now_x-Position[9][0]>=-3)&&(Ball.now_y-Position[9][1]<=2&&Ball.now_y-Position[9][1]>=0)) 
      //到达4点附近区域       <=0极佳
      {
       mode=67;
      }     
  }
  if(mode==67)
  {
        ZERO_X=1380;
        X_Parameter.KP=16;
        Y_Parameter.KP=16;
        Servo_ctl_X(Position[9][0],&PID_X);                    //
        Servo_ctl_Y(Position[9][1],&PID_Y);
        
  if((Ball.now_x-Position[9][0]<=2&&Ball.now_x-Position[9][0]>=-2)&&\
        (Ball.now_y-Position[9][1]<=2&&Ball.now_y-Position[9][1]>=-2))   //到达2点附近,开始计数
        {
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[9][0],&PID_X);                    //
        Servo_ctl_Y(Position[9][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达4点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[9][0],&PID_X);                    //
          Servo_ctl_Y(Position[9][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<200)  //在4点停3S
          {
            Servo_ctl_X(Position[9][0],&PID_X);                    //
            Servo_ctl_Y(Position[9][1],&PID_Y);
          }
          else 
          {
           mode=68;
          }
        }
    
  }
  
  
     if(mode==68)
  {
    
       Servo_ctl_X(Position[21][0],&PID_X);
       Servo_ctl_Y(Position[21][1],&PID_Y);
   if((Ball.now_x-Position[8][0]<=3&&Ball.now_x-Position[8][0]>=0)&&(Ball.now_y-Position[8][1]<=3&&Ball.now_y-Position[8][1]>=-3)) 
      //到达4点附近区域       <=0极佳
      {
       mode=69;
      }     
  }
  if(mode==69)
  {
        ZERO_X=1370;
        
        Servo_ctl_X(Position[8][0],&PID_X);                    //
        Servo_ctl_Y(Position[8][1],&PID_Y);
        
  if((Ball.now_x-Position[8][0]<=2&&Ball.now_x-Position[8][0]>=-2)&&\
        (Ball.now_y-Position[8][1]<=2&&Ball.now_y-Position[8][1]>=-2))   //到达2点附近,开始计数
        {
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[8][0],&PID_X);                    //
        Servo_ctl_Y(Position[8][1],&PID_Y);
      }
      else if(comein_count==300)         //认为已到达4点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[8][0],&PID_X);                    //
          Servo_ctl_Y(Position[8][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<200)  //在4点停3S
          {
            Servo_ctl_X(Position[8][0],&PID_X);                    //
            Servo_ctl_Y(Position[8][1],&PID_Y);
          }
          else 
          {
           mode=80;
          }
        }
    
  }
  if(mode==80)
  {
  
  }
  
  
  
  
  
  
  
  
  }
  
  
}


void Control_transition(int transition,int aim,int mode_after)
{
      Servo_ctl_X(Position[transition][0],&PID_X);
      Servo_ctl_Y(Position[transition][1],&PID_Y);
    if((Ball.now_x-Position[aim][0]<=3&&Ball.now_x-Position[aim][0]>=-3)&&\
        (Ball.now_y-Position[aim][1]<=3&&Ball.now_y-Position[aim][1]>=-3))   //到达目标点点附近区域
      {
        comein_count=0;
       mode=mode_after;
      }
     if((Ball.now_x-Position[transition][0]<=3&&Ball.now_x-Position[transition][0]>=-3)&&\
        (Ball.now_y-Position[transition][1]<=3&&Ball.now_y-Position[transition][1]>=-3))   //到达1过渡点点附近区域
     {
        comein_count++; 
     }
    if(comein_count<200)
      {
        Servo_ctl_X(Position[transition][0],&PID_X);                    //
        Servo_ctl_Y(Position[transition][1],&PID_Y);
      }
      else if(comein_count>=200)         //认为已到达过渡点 
        {
          comein_count=0;
          mode=mode_after;
        }
}

void Control_aim(int aim,int times,int mode_after)
{
        Servo_ctl_X(Position[aim][0],&PID_X);                    //
        Servo_ctl_Y(Position[aim][1],&PID_Y);
        
  if((Ball.now_x-Position[aim][0]<=2&&Ball.now_x-Position[aim][0]>=-2)&&\
        (Ball.now_y-Position[aim][1]<=2&&Ball.now_y-Position[aim][1]>=-2))   //到达4点附近,开始计数
        {
        comein_count++; 
        }
      if(comein_count<200)
      {
         Servo_ctl_Y(Position[aim][0],&PID_X);                    //
         Servo_ctl_Y(Position[aim][1],&PID_Y);
      }
      else if(comein_count==200)         //认为已到达4点 
        {
          Time_ms=0;
          Servo_ctl_X(Position[aim][0],&PID_X);                    //
         Servo_ctl_Y(Position[aim][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<times)  //在4点停3S
          {
              Servo_ctl_X(Position[aim][0],&PID_X);                    //
              Servo_ctl_Y(Position[aim][1],&PID_Y);
          }
          else 
          {
           mode=mode_after;
          }
        }
}

void Servo_Out(void)  //舵机输出
{
  int fValue_Y,fValue_X; 
  fValue_X = PID_x - PID_lastx; 
  fValue_Y = PID_y - PID_lasty; 
  Out_finalX = fValue_X * (outcount+1)/2+PID_lastx;//分段输出，逐步累加，赞
  Out_finalY = fValue_Y * (outcount+1)/2+PID_lasty;
   
  
 
   ftm_pwm_duty(FTM2 , FTM_CH0,(unsigned int)(ZERO_X-(int)Out_finalX));
   ftm_pwm_duty(FTM2 , FTM_CH1,(unsigned int)(ZERO_Y+(int)Out_finalY));
  
//  ftm_pwm_duty(FTM2 , FTM_CH0,ZERO_X-PID_x);
//  ftm_pwm_duty(FTM2 , FTM_CH1,ZERO_Y+PID_y);
 
}




