#include "include.h"
#include "ServoControl.h"

#define I_integral 100


#define X_max  350
#define Y_max  350
extern int ZERO_X;//��λ  
extern int ZERO_Y;//��λ  

extern uint16 outcount;
extern int mode;
extern uint16 comein_count;//����Ŀ����ж�
extern int Position[25][2];
extern long int Time_ms;//MS����

Ball_Position Ball;
PID_LocTypeDef PID_X;
PID_LocTypeDef PID_Y;

PID_Parameter X_Parameter={14,0,35};//0.25  40
PID_Parameter Y_Parameter={15,0,35};




 int PID_x=0;    //PID�������pwm������
 int PID_y=0;  
 int PID_lastx=0;
 int PID_lasty=0;
 int Out_finalX=0;
 int Out_finalY=0;
 

/************************************************
�������� �� PID_Loc
��    �� �� PIDλ��(Location)����
��    �� �� SetValue ------ ����ֵ(����ֵ)
            ActualValue --- ʵ��ֵ(����ֵ)
            PID ----------- PID���ݽṹ
�� �� ֵ �� PIDLoc -------- PIDλ��
��    �� �� strongerHuang
*************************************************/




void Servo_ctl_X(int Set_X, PID_LocTypeDef *PID)
{
  
  PID->Ek = Set_X - Ball.now_x ;
  if(PID->Ek>=-1&&PID->Ek<=1)
    PID->Ek=0;
  PID->LocSum += PID->Ek;                         //�ۼ����
// if( PID->Ek<=3&&PID->Ek>=-3)
// {
//   X_Parameter.KI=0.5;
// }
// else
//    X_Parameter.KI=0;
//    
  if(  PID->LocSum>I_integral)     //�����޷�
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
                               //λ��
 
  PID->Ek = Set_Y - Ball.now_y;
  if(PID->Ek>=-1&&PID->Ek<=1)
    PID->Ek=0;
  PID->LocSum += PID->Ek;                         //�ۼ����
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

//�˶��ƻ�
//���ڸ������ɵ㼰�ж��Ƿ񵽴�Ŀ���
void Servo_Plan(void)
{
  /************ģʽ1************************/
      
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
  
  /**************ģʽ2***************************/
  if(mode==2||mode==22)
  {
  if(mode==2)  //ģʽ2.1
  {
   //  Control_transition(10,5,22);
     comein_count=0;
    X_Parameter.KD=35;
    Y_Parameter.KD=35;
       ZERO_X=1370;//��λ  1460  1100-1900 ������
       ZERO_Y=1650;
     Servo_ctl_X(Position[10][0],&PID_X);
     Servo_ctl_Y(Position[10][1],&PID_Y);
    if((Ball.now_x-Position[5][0]<=2&&Ball.now_x-Position[5][0]>=-5)&&\
        (Ball.now_y-Position[5][1]<=5&&Ball.now_y-Position[5][1]>=-5))   //����5�㸽������
    {
       mode=22;
 
    }

  
  }
  
  if(mode==22) //ģʽ2.2
  {     X_Parameter.KP=25;
        Y_Parameter.KP=25;
    Servo_ctl_X(Position[5][0],&PID_X);                    //
    Servo_ctl_Y(Position[5][1],&PID_Y);
        
  if((Ball.now_x-Position[5][0]<=2&&Ball.now_x-Position[5][0]>=-2)&&\
        (Ball.now_y-Position[5][1]<=2&&Ball.now_y-Position[5][1]>=-2))   //����5�㸽��,��ʼ����
        {
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[5][0],&PID_X);                    //
        Servo_ctl_Y(Position[5][1],&PID_Y);
      }
      else if(comein_count==300)         //��Ϊ�ѵ���5�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[5][0],&PID_X);                    //
          Servo_ctl_Y(Position[5][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<3000)  //��5��ͣ3S
          {
            Servo_ctl_X(Position[5][0],&PID_X);                    //
            Servo_ctl_Y(Position[5][1],&PID_Y);
          }
          else 
          {
           //�տ�
          }
        }
    
    
    
   }
  
  }


 /**************ģʽ3**********************/ 
 
  if(mode==3||mode==32||mode==33||mode==34)
  {
   if(mode==3)
   {    comein_count=0;
       ZERO_X=1360;
       ZERO_Y=1670;
       Servo_ctl_X(Position[11][0],&PID_X);
       Servo_ctl_Y(Position[11][1],&PID_Y);
    if((Ball.now_x-Position[4][0]<=3&&Ball.now_x-Position[4][0]>=-3)&&\
        (Ball.now_y-Position[4][1]<=3&&Ball.now_y-Position[4][1]>=0))   //����4�㸽������
      {
       mode=32;
      }   
   }
  
  
  if(mode==32) //ģʽ3.2
  {   
    
     ZERO_X=1370;//��λ  1460  1100-1900 ������
       ZERO_Y=1640;
        X_Parameter.KP=16;
        Y_Parameter.KP=16;
        Servo_ctl_X(Position[4][0],&PID_X);                    //
        Servo_ctl_Y(Position[4][1],&PID_Y);
        
  if((Ball.now_x-Position[4][0]<=3&&Ball.now_x-Position[4][0]>=-3)&&\
        (Ball.now_y-Position[4][1]<=3&&Ball.now_y-Position[4][1]>=-3))   //����4�㸽��,��ʼ����
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
      else if(comein_count==300)         //��Ϊ�ѵ���4�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[4][0],&PID_X);                    //
          Servo_ctl_Y(Position[4][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<3000)  //��4��ͣ3S
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
        (Ball.now_y-Position[5][1]<=3&&Ball.now_y-Position[5][1]>=-3))   //����5�㸽������
      {
       mode=34;
      } 
    
   }

  if(mode==34) //ģʽ3.2
  {
    
        Servo_ctl_X(Position[5][0],&PID_X);                    //
        Servo_ctl_Y(Position[5][1],&PID_Y);
        
  if((Ball.now_x-Position[5][0]<=2&&Ball.now_x-Position[5][0]>=-2)&&\
        (Ball.now_y-Position[5][1]<=2&&Ball.now_y-Position[5][1]>=-2))   //����4�㸽��,��ʼ����
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
      else if(comein_count==300)         //��Ϊ�ѵ���4�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[5][0],&PID_X);                    //
          Servo_ctl_Y(Position[5][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<3000)  //��5��ͣ3S
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
   /******************ģʽ4********************/

  if(mode==4||mode==41||mode==42||mode==43||mode==44)//����9���x y ����5-9�м�� ����������ֹͣ��
  {
  if(mode==4)  //ģʽ2.1
  { 
     comein_count=0;
     ZERO_X=1400;
     ZERO_Y=1670; 
     
     Servo_ctl_X(Position[10][0]-3,&PID_X);
     Servo_ctl_Y(Position[10][1]+3,&PID_Y);
    if((Ball.now_x-Position[5][0]<=6&&Ball.now_x-Position[5][0]>=-6)&&\
        (Ball.now_y-Position[5][1]<=6&&Ball.now_y-Position[5][1]>=-6))   //����5�㸽������
    {
       mode=42;
 
    }
     if((Ball.now_x-Position[15][0]<=4&&Ball.now_x-Position[15][0]>=-4)&&\
        (Ball.now_y-Position[15][1]<=4&&Ball.now_y-Position[15][1]>=-4))   //����5�㸽������
    {
       mode=42;
     
    }
     if((Ball.now_x-Position[9][0]<=5&&Ball.now_x-Position[9][0]>=-5)&&\
        (Ball.now_y-Position[9][1]<=5&&Ball.now_y-Position[9][1]>=-5))   //����5�㸽������
    {
       mode=44;
     
    }
  
  }
  
  if(mode==42) //ģʽ2.2
   {
    mode=43;
    ZERO_Y=1590;
     }
   if(mode==43)  //ģʽ2.1
  {
     Servo_ctl_X(Position[15][0]+6,&PID_X);
     Servo_ctl_Y(Position[15][1]-6,&PID_Y);
    if((Ball.now_x-Position[9][0]<=0&&Ball.now_x-Position[9][0]>=-4)&&\
        (Ball.now_y-Position[9][1]<=4&&Ball.now_y-Position[9][1]>=-4))   //����5�㸽������
    {
       mode=44;
    
    }

  
  }
  
  if(mode==44) //ģʽ4.4
  {
       ZERO_Y=1630;
      
        Servo_ctl_X(Position[9][0],&PID_X);                    //
        Servo_ctl_Y(Position[9][1],&PID_Y);
  if((Ball.now_x-Position[9][0]<=2&&Ball.now_x-Position[9][0]>=-2)&&\
        (Ball.now_y-Position[9][1]<=2&&Ball.now_y-Position[9][1]>=-2))   //����9�㸽��,��ʼ����
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
      else if(comein_count==300)         //��Ϊ�ѵ���9�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[9][0],&PID_X);                    //
          Servo_ctl_Y(Position[9][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<3000)  //��9��ͣ3S
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
  
  /********************ģʽ5**********************/
  
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
      //����4�㸽������       <=0����
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
        (Ball.now_y-Position[2][1]<=3&&Ball.now_y-Position[2][1]>=-3))   //����2�㸽��,��ʼ����
        {X_Parameter.KP=16;
        Y_Parameter.KP=16;
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[2][0],&PID_X);                    //
        Servo_ctl_Y(Position[2][1],&PID_Y);
      }
      else if(comein_count==300)         //��Ϊ�ѵ���4�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[2][0],&PID_X);                    //
          Servo_ctl_Y(Position[2][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<100)  //��4��ͣ3S
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
        (Ball.now_y-Position[6][1]<=5&&Ball.now_y-Position[6][1]>=-5))   //����4�㸽������
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
        (Ball.now_y-Position[6][1]<=3&&Ball.now_y-Position[6][1]>=-3))   //����2�㸽��,��ʼ����
        { X_Parameter.KP=16;
        Y_Parameter.KP=16;
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[6][0],&PID_X);                    //
        Servo_ctl_Y(Position[6][1],&PID_Y);
      }
      else if(comein_count==300)         //��Ϊ�ѵ���4�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[6][0],&PID_X);                    //
          Servo_ctl_Y(Position[6][1],&PID_Y);
        }
        else
        {
 
          if(Time_ms<100)  //��4��ͣ3S
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
        (Ball.now_y-Position[9][1]<=3&&Ball.now_y-Position[9][1]>=-1))   //����4�㸽������
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
        (Ball.now_y-Position[9][1]<=2&&Ball.now_y-Position[9][1]>=-2))   //����9�㸽��,��ʼ����
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
      else if(comein_count==300)         //��Ϊ�ѵ���9�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[9][0],&PID_X);                    //
          Servo_ctl_Y(Position[9][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<3000)  //��9��ͣ3S
          {
            Servo_ctl_X(Position[9][0],&PID_X);                    //
            Servo_ctl_Y(Position[9][1],&PID_Y);
          }
          else 
          {
           //�տ�
          }
        }
  }
  
 
  }
  
  
  /*********************ģʽ6***************************/
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
      //����4�㸽������       <=0����
      {
       mode=61;
      }   
  }
   if(mode==61)
  { 
      Servo_ctl_X(Position[3][0],&PID_X);                    //
        Servo_ctl_Y(Position[3][1],&PID_Y);
        
  if((Ball.now_x-Position[3][0]<=3&&Ball.now_x-Position[3][0]>=-3)&&\
        (Ball.now_y-Position[3][1]<=3&&Ball.now_y-Position[3][1]>=-3))   //����2�㸽��,��ʼ����
        {X_Parameter.KP=16;
        Y_Parameter.KP=16;
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[3][0],&PID_X);                    //
        Servo_ctl_Y(Position[3][1],&PID_Y);
      }
      else if(comein_count==300)         //��Ϊ�ѵ���4�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[3][0],&PID_X);                    //
          Servo_ctl_Y(Position[3][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<200)  //��4��ͣ3S
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
      //����4�㸽������       <=0����
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
        (Ball.now_y-Position[9][1]<=2&&Ball.now_y-Position[9][1]>=-2))   //����2�㸽��,��ʼ����
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
      else if(comein_count==300)         //��Ϊ�ѵ���4�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[9][0],&PID_X);                    //
          Servo_ctl_Y(Position[9][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<200)  //��4��ͣ3S
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
      //����4�㸽������       <=0����
      {
       mode=65;
      }     
  }
  if(mode==65)
  {
  
        Servo_ctl_X(Position[6][0],&PID_X);                    //
        Servo_ctl_Y(Position[6][1],&PID_Y);
        
  if((Ball.now_x-Position[6][0]<=2&&Ball.now_x-Position[6][0]>=-2)&&\
        (Ball.now_y-Position[6][1]<=2&&Ball.now_y-Position[6][1]>=-2))   //����2�㸽��,��ʼ����
        {X_Parameter.KP=16;
        Y_Parameter.KP=16;
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[6][0],&PID_X);                    //
        Servo_ctl_Y(Position[6][1],&PID_Y);
      }
      else if(comein_count==300)         //��Ϊ�ѵ���4�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[6][0],&PID_X);                    //
          Servo_ctl_Y(Position[6][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<500)  //��4��ͣ3S
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
      //����4�㸽������       <=0����
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
        (Ball.now_y-Position[9][1]<=2&&Ball.now_y-Position[9][1]>=-2))   //����2�㸽��,��ʼ����
        {
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[9][0],&PID_X);                    //
        Servo_ctl_Y(Position[9][1],&PID_Y);
      }
      else if(comein_count==300)         //��Ϊ�ѵ���4�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[9][0],&PID_X);                    //
          Servo_ctl_Y(Position[9][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<200)  //��4��ͣ3S
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
      //����4�㸽������       <=0����
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
        (Ball.now_y-Position[8][1]<=2&&Ball.now_y-Position[8][1]>=-2))   //����2�㸽��,��ʼ����
        {
        comein_count++; 
        }
      if(comein_count<300)
      {
        Servo_ctl_X(Position[8][0],&PID_X);                    //
        Servo_ctl_Y(Position[8][1],&PID_Y);
      }
      else if(comein_count==300)         //��Ϊ�ѵ���4�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[8][0],&PID_X);                    //
          Servo_ctl_Y(Position[8][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<200)  //��4��ͣ3S
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
        (Ball.now_y-Position[aim][1]<=3&&Ball.now_y-Position[aim][1]>=-3))   //����Ŀ���㸽������
      {
        comein_count=0;
       mode=mode_after;
      }
     if((Ball.now_x-Position[transition][0]<=3&&Ball.now_x-Position[transition][0]>=-3)&&\
        (Ball.now_y-Position[transition][1]<=3&&Ball.now_y-Position[transition][1]>=-3))   //����1���ɵ�㸽������
     {
        comein_count++; 
     }
    if(comein_count<200)
      {
        Servo_ctl_X(Position[transition][0],&PID_X);                    //
        Servo_ctl_Y(Position[transition][1],&PID_Y);
      }
      else if(comein_count>=200)         //��Ϊ�ѵ�����ɵ� 
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
        (Ball.now_y-Position[aim][1]<=2&&Ball.now_y-Position[aim][1]>=-2))   //����4�㸽��,��ʼ����
        {
        comein_count++; 
        }
      if(comein_count<200)
      {
         Servo_ctl_Y(Position[aim][0],&PID_X);                    //
         Servo_ctl_Y(Position[aim][1],&PID_Y);
      }
      else if(comein_count==200)         //��Ϊ�ѵ���4�� 
        {
          Time_ms=0;
          Servo_ctl_X(Position[aim][0],&PID_X);                    //
         Servo_ctl_Y(Position[aim][1],&PID_Y);
        }
        else
        {
          comein_count==0;
          if(Time_ms<times)  //��4��ͣ3S
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

void Servo_Out(void)  //������
{
  int fValue_Y,fValue_X; 
  fValue_X = PID_x - PID_lastx; 
  fValue_Y = PID_y - PID_lasty; 
  Out_finalX = fValue_X * (outcount+1)/2+PID_lastx;//�ֶ���������ۼӣ���
  Out_finalY = fValue_Y * (outcount+1)/2+PID_lasty;
   
  
 
   ftm_pwm_duty(FTM2 , FTM_CH0,(unsigned int)(ZERO_X-(int)Out_finalX));
   ftm_pwm_duty(FTM2 , FTM_CH1,(unsigned int)(ZERO_Y+(int)Out_finalY));
  
//  ftm_pwm_duty(FTM2 , FTM_CH0,ZERO_X-PID_x);
//  ftm_pwm_duty(FTM2 , FTM_CH1,ZERO_Y+PID_y);
 
}




