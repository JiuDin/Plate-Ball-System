#include "include.h"
#include  "Init.h"
#include "common.h"

int ZERO_X=1400;//��λ  1460  1100-1900 ������
int ZERO_Y=1600; //��λ  1700  1000-1800 ������ 


void Init()
{
    led_init(LED0);
    led_init(LED1);
    LCD_init();
  
    ftm_pwm_init(FTM2, FTM_CH0,100,ZERO_X);      //��ʼ�� ���1 
    ftm_pwm_init(FTM2, FTM_CH1,100,ZERO_Y);      //��ʼ�� �����
 
    pit_init_ms(PIT0, 1);    //1MS��ʱ�ж�
    pit_init_ms(PIT1, 1000);   
   

    uart_rx_irq_en (UART4); 
    NVIC_SetPriorityGrouping(2);                      //�����ж����ȼ�����
    NVIC_SetPriority(PIT0_IRQn,2);
    NVIC_SetPriority(UART4_RX_TX_IRQn,3);
    NVIC_SetPriority(PORTA_IRQn,0);
    NVIC_SetPriority(DMA0_IRQn,1);

  
     
}

