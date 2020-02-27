#ifndef _TLR553_H_
 #define _TLR553_H_
//panyiqiangde@126.com , 159-1921-8729
void IIC_Init(void);
void I2C_AT24C02_Test(void);                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
void IIC_Read(unsigned char subaddr , unsigned char Byte_addr , unsigned char *buffer , unsigned short num);
void IIC_Write(unsigned char subaddr , unsigned char Byte_addr , unsigned char *buffer , unsigned short num);
void TLR553_init();                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
float distance_cm();

//panyiqiangde@126.com , 159-1921-8729                                                                                                                                                                                                                           //panyiqiangde@@126.com , 159-19218729  @ 2020.02.27

                                                                                                                                                                                                                                                      // 15 9 -1 9 2 1-8 7 2 9

#endif