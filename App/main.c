/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28 12
 */

#include "common.h"
#include "include.h"
#include "MotorControl.h"
#include "ImageProcessing.h"
#include "ServoControl.h"
#include "MK60_uart.h" 
#define image 0
#define data  1
uint8           imgbuff[CAMERA_SIZE];                       //定义存储接收图像的数组
uint8           img[CAMERA_H*CAMERA_W];


int Position[25][2]={//  0中心  1     2       3      4      5
                      {31,22},{46,6},{29,7},{14,7},{49,22},{30,22} , //各目标点位置
                    //  6        7     8        9
                      {13,24},{48,40},{27,48},{14,41},
                    //   10   11(间距9) 12 -2    13      14     15       16     17      18
                      {40,13},{48,14},{39,22},{38,33},{30,29},{23,33},{44,14},{ 38,7},{19,7},
                      //19      20      21      22     23      24
                      {12,17},{12,33},{20,41},{41,41},{48,33},{21,15},
                     };             

extern PID_Parameter X_Parameter;
extern PID_Parameter Y_Parameter;
extern PID_LocTypeDef PID_X;
extern PID_LocTypeDef PID_Y;
extern Ball_Position Ball;
extern int PID_x;  
extern int PID_y; 

int mode=1;//工作模式，一个要求一个模式

uint16 outcount=0;//计数
uint16  comein_count=0;//到达目标点判定
long int Time_ms=0;//MS计数


extern  uint8  DMA_Over_Flg;                             //置1时图像采集完成
extern int Out_finalX;
extern int Out_finalY;

   
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void sendimg(void *imgaddr, uint32 imgsize);            //串口发送图像
void PIT0_IRQHandler(void);                             //用于控制舵机
void PIT1_IRQHandler(void);                             //用于倒车
void uart4_handler(void);


/*!
 *  @brief      main函数
 *  @since      v5.0
 *  @note      
 */
void  main(void)

{    
# if data

    Site_t site = {3,5};
#endif  
    
    camera_init(imgbuff);   
    Init(); //各类初始化
    
    
#if image   
    Site_t site     = {0, 0};                           //显示图像左上角位置
    Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
    Size_t size;                                       //显示区域图像大小
    size.H = LCD_H;
    size.W = LCD_W;
#endif    
    //  
   
#if data
    
    LCD_str(site,"XNOW",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 20;
    LCD_str(site,"YNOW",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 20;
    LCD_str(site,"MODE",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 20;
    LCD_str(site,"TIME",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 20;
    LCD_str(site,"COME",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 20;
    LCD_str(site,"SETY",FCOLOUR,BCOLOUR);   //显示8*16字符串

    site.x = 3 + 6*8;

#endif
    
     set_vector_handler(UART4_RX_TX_VECTORn,uart4_handler);   // 设置中断服务函数到中断向量表里
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);    //设置PIT0的中断服务函数为 PIT0_IRQHandler
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler); //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);   //设置 DMA0 的中断服务函数为 PORTA_IRQHandler
    enable_irq (PIT0_IRQn);                                 //使能PIT0中断
    while(1)
    {
   
   //   pit_time_start  (PIT2);  
#if data     
        site.y = 5;     
        LCD_num_C (site, Ball.now_x , FCOLOUR , BCOLOUR);
        site.y = 25;       
        LCD_num_C (site, Ball.now_y , FCOLOUR , BCOLOUR);
        
        site.y =45;    
        LCD_num_C (site, mode , FCOLOUR , BCOLOUR);
        site.y =65;    
        LCD_num_C (site, Time_ms , FCOLOUR , BCOLOUR);
        site.y =85;    
        LCD_num_C (site, comein_count , FCOLOUR , BCOLOUR);

        
       
#endif
         
#if image
   LCD_Img_Binary_Z(site, size, imgbuff, imgsize);
#endif
 
   
   if(DMA_Over_Flg==0)                        
        {
          camera_get_img();
          img_extract_haomi(img,imgbuff,CAMERA_SIZE);//此处感谢李浩蜜
          Position_Calculate();
          DMA_Over_Flg=0;      //如果中断清零，DMA_Over_Flg置为1 或者注释
        }
   //printf("x %d, y %d\n",Ball.now_x,Ball.now_y);
 
    }
}



/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif


}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}


//发送图像到上位机显示
//不同的上位机，不同的命令，这里使用 yy_摄像头串口调试 软件
//如果使用其他上位机，则需要修改代码
void sendimg(void *imgaddr, uint32 imgsize)
{
    uint8 cmd[4] = {0, 255, 1, 0 };    //yy_摄像头串口调试 使用的命令

    uart_putbuff(VCAN_PORT, (uint8_t *)cmd, sizeof(cmd));    //先发送命令

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //再发送图像
}

/*!
 *  @brief      正交解码
 *  @since      v5.0
 */
void PIT0_IRQHandler(void)
{
   
  if(PIT_TFLG(PIT0) == 1)
  {
        Time_ms++; 
        
      if(Time_ms%10==0)
      {
         //100ms计算一次输出

//        Servo_ctl_X(Position[0][0],&PID_X);
//        Servo_ctl_Y(Position[0][1],&PID_Y);
        Servo_Plan();
        outcount = 0;
      }
      if(Time_ms%5 == 0)
      {
          outcount++;
          Servo_Out();  //10ms控制一次舵机
      }
     
  }
  PIT_Flag_Clear(PIT0);       //清中断标志位
      
}

void uart4_handler(void)
{
    char ch;

    if(uart_query    (UART4) == 1)   //接收数据寄存器满
    {
        //用户需要处理接收数据
        uart_getchar   (UART4, &ch);                    //无限等待接受1个字节
        uart_putchar    (UART4 , ch);
       switch(ch)
       {
       case 'M' :uart_putstr    (UART4 , "OK\n");  break;                  //发送字符串
       case 'A' :mode=1;       uart_putstr    (UART4 , "OK,MODE=1\n");                  break; 
       case 'B' :mode=2;       uart_putstr    (UART4 , "OK,MODE=2\n");                   break; 
       case 'C' :mode=3;       uart_putstr    (UART4 , "OK,MODE=3\n");                   break; 
       case 'D' :mode=4;       uart_putstr    (UART4 , "OK,MODE=4\n");                   break; 
       case 'E' :mode=5;       uart_putstr    (UART4 , "OK,MODE=5\n");                   break;
       case 'F' :mode=6;       uart_putstr    (UART4 , "OK,MODE=6\n");                   break;
       case 'G' :mode=7;       uart_putstr    (UART4 , "OK,MODE=7\n");                   break; 
       
       }
    }
}
