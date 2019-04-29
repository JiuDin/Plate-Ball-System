#include "include.h"
#include  "Init.h"
#include "common.h"

int ZERO_X=1400;//零位  1460  1100-1900 低至高
int ZERO_Y=1600; //零位  1700  1000-1800 低至高 


void Init()
{
    led_init(LED0);
    led_init(LED1);
    LCD_init();
  
    ftm_pwm_init(FTM2, FTM_CH0,100,ZERO_X);      //初始化 舵机1 
    ftm_pwm_init(FTM2, FTM_CH1,100,ZERO_Y);      //初始化 舵机二
 
    pit_init_ms(PIT0, 1);    //1MS定时中断
    pit_init_ms(PIT1, 1000);   
   

    uart_rx_irq_en (UART4); 
    NVIC_SetPriorityGrouping(2);                      //配置中断优先级分组
    NVIC_SetPriority(PIT0_IRQn,2);
    NVIC_SetPriority(UART4_RX_TX_IRQn,3);
    NVIC_SetPriority(PORTA_IRQn,0);
    NVIC_SetPriority(DMA0_IRQn,1);

  
     
}

