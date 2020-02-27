#include "stm8l15x.h"
 
#ifndef _FUNCTIONS_H_
 #define _FUNCTIONS_H_


#define USE_dev_board
//#define USE_release_board



void init_ALL();
void ADC_out();
void STM_COMInit(int COM, uint32_t USART_BaudRate,
                      USART_WordLength_TypeDef USART_WordLength,
                      USART_StopBits_TypeDef USART_StopBits,
                      USART_Parity_TypeDef USART_Parity,
                      USART_Mode_TypeDef USART_Mode);
void Delay(uint32_t nCount);
float ADC_real_voltage_V(ADC_Group_TypeDef ADC_channel_speed,ADC_Channel_TypeDef ADC_channel,int conv_times,int delayms);
float ADC_result(ADC_Group_TypeDef ADC_channel_speed,ADC_Channel_TypeDef ADC_channel,int conv_times,int delayms);
void sleep_seconds(uint16_t s);
void send_temperature(float  temperature);
void init_clk();
void task_per_2ms();
void beep_battery_low();
void temprature_check();
void GPIO_init();
static void init_LED_display_timer();
void Delay_ms(uint32_t time) ;
void task_per_2ms();

float Vrefint_factory_V();

#endif