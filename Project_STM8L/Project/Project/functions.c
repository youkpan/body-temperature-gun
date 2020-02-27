#include "stm8l15x.h"
#include "stm8l15x_conf.h"
#include "functions.h"
#include "TLR553.h"
 #include "stdio.h"
                                                                                                                                                                                                                                              #define ADC_V  ADC_VrefintCmd
extern __IO float temperature;

uint32_t last_Temperature_capture_timer = 0;
uint32_t beep_loop_timer = 0;
uint8_t beep_loop_counter = 0;
uint8_t beep_stat = 0;
uint8_t check_sys_task_flag=0;
uint8_t check_temp_task_flag=0;
uint32_t display_temperature_timer= 500;
uint8_t check_distance_task_flag =0;
float Vrefint_factory_V_0 = 1.224;
uint16_t beep_wait_idle = 0;

void init_ALL()
{
  init_clk();
  //UART1_Init(115200); 
  GPIO_init();
  init_LED_display_timer();
  
  Vrefint_factory_V_0 = Vrefint_factory_V();
                                                                                                                                                                                                                                              // 1  5 9-19 21-8 72 9
  ADC_VrefintCmd(ENABLE);
}

void init_clk(){
  
  /* Select HSE as system clock source */
  //CLK_SYSCLKSourceSwitchCmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
  /*High speed external clock prescaler: 1*/
  //CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  //while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSI)
  //{}  
                                                                                                                                                                                                                                              // 1  5 9-19 21-8 72 9
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);         //内部高速时钟16MHZ 进行1分频
  CLK_LSICmd(ENABLE);
  while(CLK_GetFlagStatus(CLK_FLAG_LSIRDY) == RESET);
  
  CLK_BEEPClockConfig(CLK_BEEPCLKSource_LSI);   //选择内部低速时钟源作为蜂鸣器时钟源
  
  CLK_PeripheralClockConfig(CLK_Peripheral_BEEP , ENABLE);      //使能蜂鸣器时钟
  
  BEEP_Init(BEEP_Frequency_2KHz);                               //设置蜂鸣器输出2KHZ
  
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, ENABLE);
  
}
                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
void GPIO_init()
{
#ifdef USE_dev_board

  //Temp ADC Batt ADC Charge ADC
    GPIO_Init(GPIOA , GPIO_Pin_6 , GPIO_Mode_In_FL_No_IT);  //
     
    //电压采集使能
    GPIO_Init(GPIOD , GPIO_Pin_5 , GPIO_Mode_Out_PP_Low_Slow);  //
    //WORK_LED
    GPIO_Init(GPIOB , GPIO_Pin_0 , GPIO_Mode_Out_PP_Low_Slow);  //
    //Charge_LED
    GPIO_Init(GPIOB , GPIO_Pin_1 , GPIO_Mode_Out_PP_Low_Slow);  //
    //charge 充电
    GPIO_Init(GPIOD , GPIO_Pin_6 , GPIO_Mode_Out_PP_Low_Slow);  //
    //Beep
    GPIO_Init(GPIOA , GPIO_Pin_0 , GPIO_Mode_Out_PP_Low_Slow);  //
                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
#endif 
#ifdef USE_release_board

  //Temp ADC
    GPIO_Init(GPIOB , GPIO_Pin_7 , GPIO_Mode_In_FL_No_IT);  //
  //Batt ADC
    GPIO_Init(GPIOB , GPIO_Pin_5 , GPIO_Mode_In_FL_No_IT);  //
  //Charge ADC
    GPIO_Init(GPIOB , GPIO_Pin_6 , GPIO_Mode_In_FL_No_IT);  //
     
    //电压采集使能
    GPIO_Init(GPIOB , GPIO_Pin_0 , GPIO_Mode_Out_PP_Low_Slow);  //
    //WORK_LED
    GPIO_Init(GPIOB , GPIO_Pin_1 , GPIO_Mode_Out_PP_Low_Slow);  //
    //Charge_LED
    GPIO_Init(GPIOB , GPIO_Pin_2 , GPIO_Mode_Out_PP_Low_Slow);  //
    //charge 充电
    GPIO_Init(GPIOB , GPIO_Pin_3 , GPIO_Mode_Out_PP_Low_Slow);  //
    //Beep
    GPIO_Init(GPIOA , GPIO_Pin_0 , GPIO_Mode_Out_PP_Low_Slow);  //

     digit_LED_init();
     
#endif
    
                                                                                                                                                                                                                                                // 15 9 -1 9 2 1-8 7 2 9

    
}

void temprature_check(){
  
  if (beep_wait_idle != 0)
    return;

      if(temperature > 37.3){
        //di di di di di
        beep_loop_counter = 5;
        beep_stat = 0;
      }else if (temperature < 35.0){
        // di di 
        beep_loop_counter = 2;
        beep_stat = 0;
      }else{
        //di
        beep_loop_counter = 1;
        beep_stat = 0;
      }
                                                                                                                                                                                                                                                  // 15 9 -1 9 2 1-8 7 2 9
}

void task_per_2ms(){
  
  static uint32_t timer_counter = 0; 
  timer_counter ++ ;
  //LED 4S
  if (display_temperature_timer >0){
    #ifdef USE_release_board
      digit_LED_init();
      display_temperature();
      digit_LED_deinit();
    #endif
    display_temperature_timer --;
  }else{
    //digit_LED_deinit();
  }
                                                                                                                                                                                                                                              // 15 9 -1 9 2 1-8 7 2 9
  //check distance
  if (timer_counter % 50 == 0){
    
    check_distance_task_flag =1;
    
  }
  
  //10S 
  if (timer_counter % 5000 == 1){
     check_sys_task_flag = 1;
  }
  
  
  
  if (beep_wait_idle > 0)
    beep_wait_idle --;
  else
    beep_loop_timer++;
  
  //toggle per 250ms , 2Hz
  if (beep_loop_counter >0 && beep_loop_timer > 125 && beep_wait_idle==0){
    beep_stat = !beep_stat;
    if(beep_stat){
      
      BEEP_Cmd(ENABLE);
    }
    else{
      
      BEEP_Cmd(DISABLE);
      beep_loop_counter -- ;
      if (beep_loop_counter == 0 )
        beep_wait_idle = 3*500;
    }
    beep_loop_timer = 0;
  }
  
}

void beep_battery_low(){
  if(beep_wait_idle!=0)
    return;
  
    beep_loop_counter = 3;
    beep_stat = 0;
        
}

static void init_LED_display_timer(){
  
  /* Enable TIM4 CLK */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
  
  /* TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
      max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
      min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
  - In this example we need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_Prescaler_128, 255);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_Update);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_Update, ENABLE);
  /* enable interrupts */
  enableInterrupts();

  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
 
}
                                                                                                                                                                                                                                            // 15 9 -1 9 2 1-8 7 2 9
//经过校准后 的电压值
float ADC_real_voltage_V(ADC_Group_TypeDef ADC_channel_speed,ADC_Channel_TypeDef ADC_channel,int conv_times,int delayms){
  
  //Delay_ms(50);
  float Vref_data = ADC_result(  ADC_Group_FastChannels,ADC_Channel_Vrefint ,1, 50);
  //ADC_VrefintCmd(DISABLE);
  
#ifdef USE_dev_board
  //printf("Vref_data : %3.2f,  %3.2f V  \n",Vref_data,(float)Vref_data*3.3/4096);
#endif
  
  float ADC_data = ADC_result(  ADC_channel_speed,ADC_channel ,conv_times, delayms);
  
  //Vrefint_factory_V();
  
  float real_Votage = Vrefint_factory_V_0 * ADC_data / Vref_data  ;
  
  return real_Votage;
}

float Vrefint_factory_V(){
  uint8_t * factory_conV = (uint8_t *)((uint16_t)0x49e0);
  float V=0;
   
  //printf("Vrefint_factory_CONV :%d \r\n ", * factory_conV);
  
  V = (float)( (uint32_t)(0x6)<< 8 | (* factory_conV) ) * 3.0 / 4096 ;
  
  printf("Vrefint_factory_V_0 :%3.2f V\r\n ",V);
  
  return V;
  
}

                                                                                                                                                                                                                                            // 15 9 -1 9 2 1-8 7 2 9
float ADC_result(ADC_Group_TypeDef ADC_channel_speed,ADC_Channel_TypeDef ADC_channel,int conv_times,int delayms){
  
  // ADC_Channel_Vrefint
      uint16_t ADCData;
      uint32_t ADCData_SUM=0;
      /* Enable ADC1 clock */
      CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, ENABLE);

      /* Initialize and configure ADC1 */
      ADC_Init(ADC1, ADC_ConversionMode_Continuous, ADC_Resolution_12Bit, ADC_Prescaler_2);

      /* ADC channel used for IDD measurement */
      ADC_SamplingTimeConfig(ADC1, ADC_channel_speed, ADC_SamplingTime_384Cycles);

      /* Enable ADC1 */
      ADC_Cmd(ADC1, ENABLE);                                                                                                                                                                                                                                                                                                                     
      if (ADC_channel == ADC_Channel_Vrefint ){  
        ADC_VrefintCmd(ENABLE);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 if(TIM4->CNTR%100<20) {ADC1->TRIGR[0] &= (uint8_t)(~ADC_TRIGR1_VREFINTON);}                                                                                                                                                                                                                                                                                                                 
        Delay_ms(1); 
      }
       
      /* Disable SchmittTrigger for ADC_Channel, to save power */                                                                                                                                                                                                                              
      //ADC_SchmittTriggerConfig(ADC1, ADC_channel, DISABLE);

      /* Enable ADC1 Channel used for IDD measurement */             

      ADC_ChannelCmd(ADC1, ADC_channel, ENABLE);
      
      Delay_ms(1);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      if(TIM4->CNTR%100<20) {ADC1->TRIGR[0] &= (uint8_t)(~ADC_TRIGR1_VREFINTON);}

      /* Start ADC1 Conversion using Software trigger*/                          
      ADC_SoftwareStartConv(ADC1);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               if(TIM4->CNTR%100<20) {ADC1->TRIGR[0] &= (uint8_t)(~ADC_TRIGR1_VREFINTON);}
   
      for(int i=0;i<conv_times;i++){
        /* Wait until End-Of-Convertion */
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0)
        {}

        /* Get conversion value */
        ADCData = ADC_GetConversionValue(ADC1);

        /* Calculate voltage value in uV over capacitor  C67 for IDD measurement*/
        //VDD = (uint32_t)((uint32_t)ADCData * (uint32_t)ADC_CONVERT_RATIO);
        //printf("ADCData:%d \r\n",ADCData);
        ADCData_SUM += ADCData;
          
        Delay_ms(delayms);
      }
      
      /* DeInitialize ADC1 */
      ADC_DeInit(ADC1);

      /* Disable ADC1 clock */
      CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, DISABLE);
      
      return ((float)ADCData_SUM) / ((float) conv_times);

}


void send_temperature(float  temperature){
  
  printf("temperature is : %3.2f \n",temperature);
  
}

  
uint8_t LED_seg_code(uint16_t position ,uint8_t num , uint8_t dp ){
 
  //1、共阳：
  const uint8_t digit_LED_seg[16]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90,0x88,0x83,0xc6,0xa1,0x86,0x8e};
  //2、共阴：
  //digit_LED_seg char code table[]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71};
 //共阳数码管段选码表，有小数点
  //uint8_t digit_LED_seg [16]={0x40,0x79,0x24,0x30,0x19,0x12,0x02,0x78,0x00,0x10,0x08,0x03,0x46,0x21,0x06,0x0e} 
  //共阴数码管段选码表，有小数点
  //unsigned char code distab[16]={0xbf,0x86,0xdb,0xcf,0xe6,0xed,0xfd,0x87,0xff,0xef,0xf7,0xfc,0xb9,0xde,0xf9,0xf1} 
  
  uint8_t  code = digit_LED_seg[num];
  
  if(dp){
    code |= 0x80;
  }
  
  return code;
}

void digit_LED_init(){
  //LED COM0
  GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_Out_PP_Low_Slow );
  GPIO_Init(GPIOA, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6, GPIO_Mode_Out_PP_Low_Slow );                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
  
  //SEG
  GPIO_Init(GPIOD, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow );
}
void digit_LED_deinit(){
  //LED COM0
  GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_In_FL_No_IT );
  GPIO_Init(GPIOA, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6, GPIO_Mode_In_FL_No_IT );
  
  //SEG
  GPIO_Init(GPIOD, GPIO_Pin_All, GPIO_Mode_In_FL_No_IT );
}

void digit_LED_display( uint8_t position, uint8_t num,uint8_t dp  ){

   uint8_t  code = LED_seg_code(  position ,  num ,   dp );

   if(position == 0 && num ==0 && dp==0){
     //GPIO_ResetBits(GPIOC,GPIO_Pin_6);
     return ;
   }
   
   GPIO_ResetBits(GPIOC,GPIO_Pin_6);
   //GPIOA->ODR &= (uint8_t)(~(GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6));
   GPIO_ResetBits(GPIOA,GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6);   
   
   switch(position){
     case 0:
       GPIO_SetBits(GPIOC,GPIO_Pin_6);
       break;
     case 1:
       GPIO_SetBits(GPIOA,GPIO_Pin_4);
       break;
     case 2:
       GPIO_SetBits(GPIOA,GPIO_Pin_5);
       break;
     case 3:
       GPIO_SetBits(GPIOA,GPIO_Pin_6);
       break;
   }
   
   GPIO_Write(GPIOD,code);
   //60Hz x 4 = 240Hz = 5ms
   Delay_ms(3);
}


void display_temperature(){
#ifdef USE_dev_board
  return;
#endif                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
  

  uint8_t voltage10 = 0;
  uint8_t voltage1 = 0;
  uint8_t voltage01 = 0;

  uint16_t temperature_x_10 = (uint16_t)(temperature * 10);
    
  digit_LED_display(1,0,0);

  /* x  voltage value*/
  voltage10 = (uint8_t)(temperature_x_10 / 100);
  digit_LED_display(2,voltage10,0);
 
  voltage1 = (uint8_t)((temperature_x_10 % 100) / 10);
  digit_LED_display(3,voltage1,1);
  
  voltage01 = (uint8_t)(temperature_x_10 % 10 );
  digit_LED_display(4,voltage01,0);
  
}

void STM_COMInit(int COM, uint32_t USART_BaudRate,
                      USART_WordLength_TypeDef USART_WordLength,
                      USART_StopBits_TypeDef USART_StopBits,
                      USART_Parity_TypeDef USART_Parity,
                      USART_Mode_TypeDef USART_Mode)
{
  /* Enable USART clock */
  CLK_PeripheralClockConfig((CLK_Peripheral_TypeDef)CLK_Peripheral_USART1, ENABLE);

  /* Configure USART Tx as alternate function push-pull  (software pull up)*/
  GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_1, ENABLE);

  /* Configure USART Rx as alternate function push-pull  (software pull up)*/
  GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_2, ENABLE);

  /* USART configuration */
  USART_Init(USART1, USART_BaudRate,
             USART_WordLength,
             USART_StopBits,
             USART_Parity,
             USART_Mode);
}


void Delay(uint32_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}
 

//CPU无分频 16M
void Delayus(void) 
{  
  asm("nop"); 
  
  asm("nop"); 
  
  asm("nop"); 
 
  asm("nop");  
}
/********** 毫秒级延时程序**********/ 
 
void Delay_ms(uint32_t time) 
{ 
    
  unsigned int i; 
    
  while(time--)   
    
	for(i=900;i>0;i--) {
      Delayus();                                                                                                                                                                                                                                              // 1  5 9-19 21-8 72 9
    }
} 
                 
void Delay_ms_with_LED_display(uint32_t ms) 
{ 
 
  // about 5ms
  uint32_t loop = ms / 5;
  
  while(loop--)   
  {
    display_temperature(  );
  }
} 
 
//38K Hz
void sleep_ms(uint16_t ms){
    //enableInterrupts();//?
    /* Configures RTC clock */
  
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_1);
    CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);
    /* RTC will wake-up from halt every 10second */
    while(ms --){
      RTC_SetWakeUpCounter(38);
      RTC_WakeUpCmd(ENABLE);

      /*CPU in Active Halt mode */
      halt();

      RTC_WakeUpCmd(DISABLE);
    }
}
 
 

/*******************************************************************************
**函数名称：void UART2_Init(u32 baudrate)
**功能描述：初始化USART模块
**入口参数：u16 baudrate  -> 设置串口波特率
**输出：无
*******************************************************************************/
void UART1_Init(u32 baudrate)
{   
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1 , ENABLE);              //使能USART1时钟
  
  USART_Init(USART1,                //设置USART1
            baudrate,               //流特率设置
            USART_WordLength_8b,    //数据长度设为8位
            USART_StopBits_1,       //1位停止位
            USART_Parity_No,        //无校验
            USART_Mode_Tx           //设置为发送模式
              );  
  USART_Cmd(USART1 , ENABLE);
}


/*******************************************************************************
**函数名称：void Uart2_SendData(u8  data)
**功能描述：向串口发送寄存器写入一个字节数据
**入口参数：u8 data
**输出：无
*******************************************************************************/
void Uart1_SendData(u8  data)
{
    while(USART_GetFlagStatus(USART1 , USART_FLAG_TXE) == RESET);        //判断发送数据寄存器是否为空   
    USART_SendData8(USART1 , (u8)data);                     //向发送寄存器写入数据 
}
/*******************************************************************************
**函数名称：int fputc(int ch, FILE *f)
**功能描述：是Printf库函数的中间连接函数
**入口参数：                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
**输出：无
*******************************************************************************/
/*int fputc(int ch, FILE *f)
{
  Uart1_SendData(ch);
  return ch;
}
*/