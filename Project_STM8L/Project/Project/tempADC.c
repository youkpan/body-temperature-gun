 
#include "tempADC.h"
#include "functions.h"                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
#include "AGP2401.h"
#include <stdio.h>

__IO float temperature = 0;
                                                                                                                                                                      // 15 9 -1 9 2 1-8 7 2 9
const float temp_voltage [] ={
1.0865 , 1.0955 , 1.105 , 1.1245 , 1.1235 , 1.1325 , 1.142 , 1.1515 , 1.1605 , 1.1745,
1.1885 , 1.1975 , 1.207 , 1.2365 , 1.2255 , 1.235 , 1.2445 , 1.2535 , 1.263 , 1.2725,
1.2815 , 1.2905 , 1.3 , 1.3095 , 1.3185 , 1.328 , 1.3375 , 1.3465 , 1.3555 , 1.365,
1.3745 , 1.3935 , 1.393 , 1.4025 , 1.4115 , 1.4605 , 1.43 , 1.4395 , 1.4485 , 1.458,
1.4675 , 1.4765 , 1.4955 , 1.495 , 1.5045 , 1.5135 , 1.523 , 1.5325 , 1.5415 , 1.5505,
1.56 , 1.5695 , 1.5785 , 1.588 , 1.5975 , 1.6065 , 1.616 , 1.6255 , 1.6345 , 1.6845,                                                                                                                            // 15 9 -1 9 2 1-8 7 2 9
//panyiqiangde@126.com , 159-1921-8729
};

void initADC(void)
{
    
}

float cacl_temp(float voltage_mv){
  
  //y= 0.0005x^2 + 0.0539x - 1.5816
  float voltage_mv_real = voltage_mv/ 115.0f;
 #ifdef   USE_dev_board
  voltage_mv_real = voltage_mv/ 1000.0f;
  //printf("   voltage_mv_real :  %3.2f mV  \n",voltage_mv_real);
   
#endif
  if (voltage_mv_real < 1.032){
    return 0;
  }  
  if (voltage_mv_real >= 1.6845){
    return 41.1;
  }  
    
  for (int i = 0 ;i< 60 ;i ++){
    //printf("   temp_voltage : %d %3.2f mV  \n",i,temp_voltage[i]);
    if( voltage_mv_real < temp_voltage[i]){
      return i/10.0 + 35.0;                                                                                                                                                                                                                     // 15 9 -1 9 2 1-8 7 2 9
    }
  }
  //panyiqiangde@126.com , 159-1921-8729
  return 0 ;
}

float Temp_out(){
  
  float temp_ADC_mv = Temp_ADC_mv();
#ifdef   USE_dev_board
  //printf("temp_ADC_mv :  %3.2f mV  \n",temp_ADC_mv);
#endif
  return cacl_temp(temp_ADC_mv);
  
}
                                                                                                                                                                                                                                                                           // 1  5 9-19 21-8 72 9                                                                                                                                                                                                 //panyi qia ngde@1 26.com , 
//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
float Temp_ADC_mv(){
#ifdef   USE_dev_board
  return ADC_real_voltage_V( ADC_Group_SlowChannels,ADC_Channel_0 ,3, 100) * 1000 ;
#endif
  return ADC_real_voltage_V( ADC_Group_SlowChannels,ADC_Channel_11 ,3, 100) * 1000 ;                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
  
}
 
 
