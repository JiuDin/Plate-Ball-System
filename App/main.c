/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��K60 ƽ̨������
 * @author     ɽ��Ƽ�
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
uint8           imgbuff[CAMERA_SIZE];                       //����洢����ͼ�������
uint8           img[CAMERA_H*CAMERA_W];


int Position[25][2]={//  0����  1     2       3      4      5
                      {31,22},{46,6},{29,7},{14,7},{49,22},{30,22} , //��Ŀ���λ��
                    //  6        7     8        9
                      {13,24},{48,40},{27,48},{14,41},
                    //   10   11(���9) 12 -2    13      14     15       16     17      18
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

int mode=1;//����ģʽ��һ��Ҫ��һ��ģʽ

uint16 outcount=0;//����
uint16  comein_count=0;//����Ŀ����ж�
long int Time_ms=0;//MS����


extern  uint8  DMA_Over_Flg;                             //��1ʱͼ��ɼ����
extern int Out_finalX;
extern int Out_finalY;

   
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void sendimg(void *imgaddr, uint32 imgsize);            //���ڷ���ͼ��
void PIT0_IRQHandler(void);                             //���ڿ��ƶ��
void PIT1_IRQHandler(void);                             //���ڵ���
void uart4_handler(void);


/*!
 *  @brief      main����
 *  @since      v5.0
 *  @note      
 */
void  main(void)

{    
# if data

    Site_t site = {3,5};
#endif  
    
    camera_init(imgbuff);   
    Init(); //�����ʼ��
    
    
#if image   
    Site_t site     = {0, 0};                           //��ʾͼ�����Ͻ�λ��
    Size_t imgsize  = {CAMERA_W, CAMERA_H};             //ͼ���С
    Size_t size;                                       //��ʾ����ͼ���С
    size.H = LCD_H;
    size.W = LCD_W;
#endif    
    //  
   
#if data
    
    LCD_str(site,"XNOW",FCOLOUR,BCOLOUR);   //��ʾ8*16�ַ���
    site.y += 20;
    LCD_str(site,"YNOW",FCOLOUR,BCOLOUR);   //��ʾ8*16�ַ���
    site.y += 20;
    LCD_str(site,"MODE",FCOLOUR,BCOLOUR);   //��ʾ8*16�ַ���
    site.y += 20;
    LCD_str(site,"TIME",FCOLOUR,BCOLOUR);   //��ʾ8*16�ַ���
    site.y += 20;
    LCD_str(site,"COME",FCOLOUR,BCOLOUR);   //��ʾ8*16�ַ���
    site.y += 20;
    LCD_str(site,"SETY",FCOLOUR,BCOLOUR);   //��ʾ8*16�ַ���

    site.x = 3 + 6*8;

#endif
    
     set_vector_handler(UART4_RX_TX_VECTORn,uart4_handler);   // �����жϷ��������ж���������
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);    //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler); //���� PORTA ���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);   //���� DMA0 ���жϷ�����Ϊ PORTA_IRQHandler
    enable_irq (PIT0_IRQn);                                 //ʹ��PIT0�ж�
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
          img_extract_haomi(img,imgbuff,CAMERA_SIZE);//�˴���л�����
          Position_Calculate();
          DMA_Over_Flg=0;      //����ж����㣬DMA_Over_Flg��Ϊ1 ����ע��
        }
   //printf("x %d, y %d\n",Ball.now_x,Ball.now_y);
 
    }
}



/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n;    //���ź�
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //ʹ�����ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif


}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}


//����ͼ����λ����ʾ
//��ͬ����λ������ͬ���������ʹ�� yy_����ͷ���ڵ��� ���
//���ʹ��������λ��������Ҫ�޸Ĵ���
void sendimg(void *imgaddr, uint32 imgsize)
{
    uint8 cmd[4] = {0, 255, 1, 0 };    //yy_����ͷ���ڵ��� ʹ�õ�����

    uart_putbuff(VCAN_PORT, (uint8_t *)cmd, sizeof(cmd));    //�ȷ�������

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //�ٷ���ͼ��
}

/*!
 *  @brief      ��������
 *  @since      v5.0
 */
void PIT0_IRQHandler(void)
{
   
  if(PIT_TFLG(PIT0) == 1)
  {
        Time_ms++; 
        
      if(Time_ms%10==0)
      {
         //100ms����һ�����

//        Servo_ctl_X(Position[0][0],&PID_X);
//        Servo_ctl_Y(Position[0][1],&PID_Y);
        Servo_Plan();
        outcount = 0;
      }
      if(Time_ms%5 == 0)
      {
          outcount++;
          Servo_Out();  //10ms����һ�ζ��
      }
     
  }
  PIT_Flag_Clear(PIT0);       //���жϱ�־λ
      
}

void uart4_handler(void)
{
    char ch;

    if(uart_query    (UART4) == 1)   //�������ݼĴ�����
    {
        //�û���Ҫ�����������
        uart_getchar   (UART4, &ch);                    //���޵ȴ�����1���ֽ�
        uart_putchar    (UART4 , ch);
       switch(ch)
       {
       case 'M' :uart_putstr    (UART4 , "OK\n");  break;                  //�����ַ���
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
