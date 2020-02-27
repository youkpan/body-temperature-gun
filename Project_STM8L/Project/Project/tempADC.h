#ifndef _TEMP_ADC_H_
 #define _TEMP_ADC_H_

  void initADC(void);
  float Temp_ADC_mv();
  float cacl_temp(float voltage_mv);
  void ADC_out();
  float Temp_out();
  
#endif