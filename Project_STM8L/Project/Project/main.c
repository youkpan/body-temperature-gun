
/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "stm8l15x_conf.h"
#include "functions.h"
#include "timing_delay.h"
#include "stdio.h"
#include "stm8_eval.h"
 #include "TLR553.h"
 #include "tempADC.h"

__IO extern float temperature;
extern uint8_t check_sys_task_flag;
//writen by panyiqiangde@126.com , 15919218729  @ 2020.02.27
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

void GPIO_LowPower_Config(void);

//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
extern uint32_t last_Temperature_capture_timer ;
extern uint32_t beep_loop_timer ;
extern uint8_t beep_loop_counter ;
extern uint8_t beep_stat ;
extern uint8_t check_sys_task_flag;
extern uint8_t check_temp_task_flag;
extern uint32_t display_temperature_timer;
extern uint8_t check_distance_task_flag ;
//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */
//panyiqiangde@@126.com , 159-19218729  @ 2020.02.27
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{

  init_ALL();//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
  
  STM_COMInit(0, (uint32_t)115200, USART_WordLength_8b, USART_StopBits_1,
                   USART_Parity_No, (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));
  
  
  printf("\n\r Temprature capture Start !\n\r");
  
  /*
   while (1)
  {
    ans = getchar();
    printf("%c", ans);
  }
  */
  
  /*To generate 1 ms time base using TIM2 update interrupt*/
  //TimingDelay_Init();

  /* Enable Interrupts*/
  enableInterrupts();

  //Delay(200);
 
#ifdef USE_dev_board
  while(0){
 
  }
#endif
    
    
  while (1)
  {
    /* Print "IDD Run Mode" in the LCD first line */
    //

    if(check_distance_task_flag ){
#ifdef   USE_dev_board
      GPIO_Init(GPIOA , GPIO_Pin_4 , GPIO_Mode_In_PU_No_IT);
      //2cm
      if(distance_cm() < 2 && beep_loop_counter==0 || GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)== RESET ){
        check_distance_task_flag = 0;
        //work
        GPIO_ResetBits(GPIOB,GPIO_Pin_0);
        temperature = Temp_out();
        //by panyiqiangde@126.com , 15919218729  @ 2020.02.27
        
        GPIO_SetBits(GPIOB,GPIO_Pin_0);
        
        display_temperature_timer = 2000;
        //printf("   temperature :  %3.2f  \n\r",temperature);
        //display_temperature (temperature);
        send_temperature(temperature);
        temprature_check();
      }
#endif
      
#ifdef USE_release_board
      GPIO_Init(GPIOC , GPIO_Pin_4 , GPIO_Mode_In_PU_No_IT);
      //2cm
      if(distance_cm() < 2 && beep_loop_counter==0 || GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)== RESET ){

        GPIO_Init(GPIOC , GPIO_Pin_4 , GPIO_Mode_In_FL_No_IT);
        check_distance_task_flag = 0;
        //work
        GPIO_ResetBits(GPIOB,GPIO_Pin_1);
        temperature = Temp_out();
        
        GPIO_SetBits(GPIOB,GPIO_Pin_1);
        //by panyiqiangde@126.com , 15919218729  @ 2020.02.27
        display_temperature_timer = 2000;
        //printf("   temperature :  %3.2f  \n\r",temperature);
        //display_temperature (temperature);
        send_temperature(temperature);
        temprature_check();
        
        //by panyiqiangde@126.com , 15919218729  @ 2020.02.27
      }
#endif        
        
      
#ifdef   USE_dev_board
      GPIO_Init(GPIOA , GPIO_Pin_4 , GPIO_Mode_In_FL_No_IT);
#endif
      
#ifdef USE_release_board
        
      GPIO_Init(GPIOC , GPIO_Pin_4 , GPIO_Mode_In_FL_No_IT);
#endif
    }
    
    
    if(check_sys_task_flag){
      check_sys_task_flag = 0;
      //电压采集使能
      
#ifdef   USE_dev_board
      GPIO_SetBits(GPIOD,GPIO_Pin_5);
      //Delay_ms(100);
      //batt 
      printf("---start batt test \r\n");
      float batt = ADC_real_voltage_V( ADC_Group_SlowChannels,ADC_Channel_0 ,3, 100);
      GPIO_ResetBits(GPIOD,GPIO_Pin_5);
      
      //Delay_ms(100);
      printf("---batt :  %3.2f V  \r\n",batt);
      printf("---start charge test \r\n");
      float charge = ADC_real_voltage_V(ADC_Group_SlowChannels,ADC_Channel_0,3,100);
#endif
      
#ifdef USE_release_board
        
      GPIO_SetBits(GPIOB,GPIO_Pin_0);
//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
      float batt = ADC_real_voltage_V(ADC_Group_SlowChannels,ADC_Channel_13,2,100);
      GPIO_ResetBits(GPIOB,GPIO_Pin_0);
      //printf("   batt :  %3.2f V  \n",batt);
//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
      float charge = ADC_real_voltage_V(ADC_Group_SlowChannels,ADC_Channel_12,2,100);
      
#endif
 
      printf("---charge :  %3.2f V \r\n",charge);
      if(charge > 2 ){
        
#ifdef USE_dev_board
        if(  batt < 2.5 ){
          //Charge_LED
          GPIO_ResetBits(GPIOB,GPIO_Pin_1);
          //+3V_CONTROL charge
          GPIO_SetBits(GPIOD,GPIO_Pin_6);
        }else{
          //Charge_LED
          GPIO_SetBits(GPIOB,GPIO_Pin_1);
          //+3V_CONTROL charge
          GPIO_ResetBits(GPIOD,GPIO_Pin_6);
        }
                                                                                                                                                                                          //panyiqiangde@@126.com , 159-19218729  @ 2020.02.27
#endif
        
#ifdef USE_release_board
        if(  batt < 4.16 /2  ){
          //Charge_LED
          GPIO_ResetBits(GPIOB,GPIO_Pin_2);
          //+3V_CONTROL charge
          GPIO_SetBits(GPIOB,GPIO_Pin_3);
        }else{
          //Charge_LED
          GPIO_SetBits(GPIOB,GPIO_Pin_2);
          //+3V_CONTROL charge
          GPIO_ResetBits(GPIOB,GPIO_Pin_3);
        }
        
        //by panyiqiangde@126.com , 15919218729  @ 2020.02.27
#endif
      }
      else{
        
        if(  batt < 3.0 ){
          beep_battery_low();
        }
        
      }
    }
    
 //by panyiqiangde@126.com , 15919218729  @ 2020.02.27
  }
}



/**
  * @brief Retargets the C library printf function to the USART.
  * @param[in] c Character to send
  * @retval char Character sent
  * @par Required preconditions:
  * - None
  */
PUTCHAR_PROTOTYPE
{
  /* Write a character to the USART */
  USART_SendData8(EVAL_COM1, c);
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET);
//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
  return (c);
}

/**
  * @brief Retargets the C library scanf function to the USART.
  * @param[in] None
  * @retval char Character to Read
  * @par Required preconditions:
  * - None
  */
GETCHAR_PROTOTYPE
{
  int c = 0;
  /* Loop until the Read data register flag is SET */
  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_RXNE) == RESET);
    c = USART_ReceiveData8(EVAL_COM1);
    return (c);
 }


/**
  * @brief  Display the current on the LCD.
  * @param  Current: specifies the current in mA.
  * @retval None
  */
void CurrentDisplay(uint32_t Current)
{

   //by panyiqiangde@126.com , 15919218729  @ 2020.02.27
}

/**
  * @brief  Configure all GPIO in output push-pull mode .
  * @param  None.
  * @retval None.
  */
void GPIO_LowPower_Config(void)
{
  /* Port A in output push-pull 0 */
  GPIO_Init(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
  /* Port B in output push-pull 0 */
  GPIO_Init(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
  /* Port C in output push-pull 0 */
  GPIO_Init(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
  /* Port D in output push-pull 0 */
  GPIO_Init(GPIOD, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
 //by panyiqiangde@126.com , 15919218729  @ 2020.02.27
}
                                                                                                                                                                                          //panyiqiangde@@126.com , 159-19218729  @ 2020.02.27

 

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
                                                                                                                                                                                          //panyiqiangde@@126.com , 159-19218729  @ 2020.02.27
  * @param  line: assert_param error line source number
  * @retval None
  */                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
  //by panyiqiangde@126.com , 15919218729  @ 2020.02.27
}
#endif
/**
  * @}
  */
/**                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
