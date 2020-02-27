//panyiqiangde@126.com , 159-1921-8729
#include"stm8l15x.h"
#include <stdio.h>
#include"TLR553.h"
#include"functions.h"

#define I2C_SPEED              100000
#define I2C_SLAVE_ADDRESS7     0x23
#define I2C_TIMEOUT         (uint32_t)0x3FFFF /*!< I2C Time out */


/*******************************************************************************
**函数名称：void delay(unsigned int ms)     Name: void delay(unsigned int ms)
**功能描述：大概延时
**入口参数：unsigned int ms   输入大概延时数值
**输出：无//panyiqiangde@126.com , 159-1921-8729
*******************************************************************************/
void Delayms(unsigned int ms)
{
  unsigned int x , y;
  for(x = ms; x > 0; x--)           /*  通过一定周期循环进行延时*/
    for(y = 3000 ; y > 0 ; y--);
}
/*******************************************************************************
**函数名称:void IIC_Init() 
**功能描述:初始化IIC1接口
**入口参数:
**输出:无
*******************************************************************************/
void IIC_Init()
{					
//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1 , ENABLE);              //使能IIC1时钟

  I2C_Cmd(I2C1, ENABLE);
  
  /* sEE_I2C configuration after enabling it */
  I2C_Init(I2C1, I2C_SPEED, I2C_SLAVE_ADDRESS7, I2C_Mode_I2C, I2C_DutyCycle_2,
           I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);
  
  
}

void TLR553_init(){
  uint8_t I2c_Buf[1] = {0};
 //panyiqiangde@126.com , 159-1921-8729
  //ALS active
  I2c_Buf[0]= 1;
  IIC_Write(I2C_SLAVE_ADDRESS7 , 0x80 ,  I2c_Buf , 1);
  Delayms(5);
  //PS active
  I2c_Buf[0]= 1;
  IIC_Write(I2C_SLAVE_ADDRESS7 , 0x81 ,  I2c_Buf , 1);
  Delayms(5);
  //LED
  I2c_Buf[0]= 0x7A;
  IIC_Write(I2C_SLAVE_ADDRESS7 , 0x82 ,  I2c_Buf , 1);
  Delayms(5);
  //PS rate 50ms ,
  I2c_Buf[0]= 0x00;                                                                                                                                                       // 15 9 -1 9 2 1-8 7 2 9
  IIC_Write(I2C_SLAVE_ADDRESS7 , 0x84 ,  I2c_Buf , 1);
  Delayms(5);
  //ALS rate 200ms ,
  I2c_Buf[0]= 0x12;
  IIC_Write(I2C_SLAVE_ADDRESS7 , 0x85 ,  I2c_Buf , 1);
  Delayms(5);
}
//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
float distance_cm(){
  
#ifdef USE_dev_board
  return 0.0;
#endif
  //panyiqiangde@126.com , 159-1921-8729
    uint8_t I2c_Buf[1] = {0};
    //CH1
    IIC_Read(I2C_SLAVE_ADDRESS7 , 0x88 ,  I2c_Buf , 1);
    uint8_t low_byte = I2c_Buf[0];
    IIC_Read(I2C_SLAVE_ADDRESS7 , 0x89 ,  I2c_Buf , 1);
    uint8_t high_byte = I2c_Buf[0];
    uint16_t ALS_data_CH1 = low_byte | (uint16_t) high_byte << 8;
    
    printf("ALS_data_CH1 %d\n",ALS_data_CH1);
    IIC_Read(I2C_SLAVE_ADDRESS7 , 0x8A ,  I2c_Buf , 1);
    low_byte =  I2c_Buf[0];
    IIC_Read(I2C_SLAVE_ADDRESS7 , 0x8B ,  I2c_Buf , 1);
    high_byte = I2c_Buf[0];
    uint16_t ALS_data_CH0 = low_byte | (uint16_t) high_byte << 8;
    printf("ALS_data_CH0 %d\n",ALS_data_CH0);
    
                                                                                                                                                                      // 15 9 -1 9 2 1-8 7 2 9
  //status
    IIC_Read(I2C_SLAVE_ADDRESS7 , 0x8C ,  I2c_Buf , 1);
    uint8_t statusdata = I2c_Buf[0];
    printf("statusdata %d\n",statusdata);
    
    uint8_t newdata = statusdata & 0x05;
    //4 ALS ,1 PS
    IIC_Read(I2C_SLAVE_ADDRESS7 , 0x8D ,  I2c_Buf , 1);
    low_byte =  I2c_Buf[0];                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
    IIC_Read(I2C_SLAVE_ADDRESS7 , 0x8E ,  I2c_Buf , 1);
    high_byte = I2c_Buf[0];
    
    uint16_t PS_data = low_byte | (uint16_t) high_byte << 8;
    printf("PS_data %d\n",PS_data);
    
    return 0.0;
  
}
//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
/*******************************************************************************
**函数名称:void IIC_Write(unsigned char subaddr , unsigned char Byte_addr , unsigned char *buffer , unsigned short num)
**功能描述:向IIC器件写数据
**入口参数:
          subaddr :  从器件地址
          Byte_addr : 确定器件写地址的起始地址
          *buffer   : 写数据的起址地址
          num				: 要写数据的个数
**输出:无
*******************************************************************************/
void IIC_Write(unsigned char subaddr , unsigned char Byte_addr , unsigned char *buffer , unsigned short num)
{

    //发送起始信号/*!< Send STRAT condition */
    I2C_GenerateSTART(I2C1, ENABLE);  
    //panyiqiangde@126.com , 159-1921-8729
    /*!< Test on EV5 and clear it *///等待起始信号产生
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
      /*!< Send EEPROM address for write *///发送器件地地址，并清除SB标志位
    I2C_Send7bitAddress(I2C1, (u8)subaddr, I2C_Direction_Transmitter);                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
    /*!< Test on EV6 and clear it *///等待器件地址发送完成并清除发送器件地址标志位
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    /*!< Send the EEPROM's internal address to write to : LSB of the address */
    I2C_SendData(I2C1, (u8)(Byte_addr));//发送器件存储首地址
    /*!< Test on EV8 and clear it *///等待移位发送器发送完成
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
      
    while(num > 0)
    { /*!< Send the byte to be written *///发送器件存储首地址
        I2C_SendData(I2C1, *buffer);
        /*!< Test on EV8 and clear it *///等待移位发送器发送完成
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

        buffer++;
        num--;
    }	
    /*!< Send STOP condition *///发送停止信号结束数据传输
    I2C_GenerateSTOP(I2C1 , ENABLE);
}

//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
/*******************************************************************************
**函数名称:void IIC_Read(unsigned char subaddr , unsigned char Byte_addr , unsigned char *buffer , unsigned short num)
**功能描述:向IIC器件读数据
**入口参数:
          subaddr :  从器件地址
          Byte_addr : 确定器件写地址的起始地址
          *buffer   : 读数据的缓冲区起始地址
          num				: 要读数据的个数
**输出:无                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
*******************************************************************************/
void IIC_Read(unsigned char subaddr , unsigned char Byte_addr , unsigned char *buffer , unsigned short num)
{
  /*!< While the bus is busy */
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  
  /*!< Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);//产生应答信号        
  
  /*!< Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);//发送起始信号
  /*!< Test on EV5 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//等待起始信号产生
  
  /*!< Send EEPROM address for write *///发送器件地地址，并清除SB标志位
  I2C_Send7bitAddress(I2C1, (u8)subaddr, I2C_Direction_Transmitter);
  /*!< Test on EV6 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//等待器件地址发送完成

  /*!< Send the EEPROM's internal address to read from: LSB of the address */
  I2C_SendData(I2C1, (u8)(Byte_addr));//发送存储地址
  /*!< Test on EV8 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//等待移位发送器发送完成
                                                                                                                                                                                                                                            // 15 9 -1 9 2 1-8 7 2 9
   /*!< Send STRAT condition a second time */
  I2C_GenerateSTART(I2C1, ENABLE); //重新发送起始信号
  /*!< Test on EV5 and clear it *///等待起始信号产生
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

   /*!< Send EEPROM address for read *///发送器件地地址，并清除SB标志位
  I2C_Send7bitAddress(I2C1, (u8)subaddr, I2C_Direction_Receiver);
//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
  /*!< Test on EV6 and clear it *///等待器件地址发送完成                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));


  while(num)
  {
    if(num == 1)
    {   /*!< Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);//最后一个字节不产生应答信号         
        /*!< Send STOP Condition */
      I2C_GenerateSTOP(I2C1, ENABLE);//发送停止信号结束数据传输         
    }
    /*!< Test on EV7 and clear it */ //等待数据接收完成
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

    *buffer = I2C_ReceiveData(I2C1);  //读出数据
    buffer++;   //读出数据缓存地址递加
    num--;      //接收数据数目减1
  }
}


ErrorStatus IIC_Read1(unsigned char subaddr , unsigned char Byte_addr , unsigned char *buffer , unsigned short num)
{
 
  uint32_t I2C_TimeOut = I2C_TIMEOUT;
  
  /*!< While the bus is busy */
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  
  /*!< Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);//产生应答信号        
  
  /*!< Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);//发送起始信号
  /*!< Test on EV5 and clear it */
    while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && I2C_TimeOut)  /* EV5 */
  {
    I2C_TimeOut--;
  }
  if (I2C_TimeOut == 0)
  {//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
    return ERROR;
  }
  I2C_TimeOut = I2C_TIMEOUT;
  
  /*!< Send EEPROM address for write *///发送器件地地址，并清除SB标志位
  I2C_Send7bitAddress(I2C1, (u8)subaddr, I2C_Direction_Transmitter);
  /*!< Test on EV6 and clear it */
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && I2C_TimeOut)  /* EV5 */
  {
    I2C_TimeOut--;
  }
  if (I2C_TimeOut == 0)
  {
    return ERROR;
  }
  I2C_TimeOut = I2C_TIMEOUT;

  /*!< Send the EEPROM's internal address to read from: LSB of the address */
  I2C_SendData(I2C1, (u8)(Byte_addr));//发送存储地址
  /*!< Test on EV8 and clear it */
 
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && I2C_TimeOut)  /* EV5 */
  {
    I2C_TimeOut--;
  }
  if (I2C_TimeOut == 0)
  {
    return ERROR;
  }
  I2C_TimeOut = I2C_TIMEOUT;
                                                                                                                                                                                                                                              // 15 9 -1 9 2 1-8 7 2 9
   /*!< Send STRAT condition a second time */
  I2C_GenerateSTART(I2C1, ENABLE); //重新发送起始信号
  /*!< Test on EV5 and clear it *///等待起始信号产生
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

   /*!< Send EEPROM address for read *///发送器件地地址，并清除SB标志位
  I2C_Send7bitAddress(I2C1, (u8)subaddr, I2C_Direction_Receiver);

  /*!< Test on EV6 and clear it *///等待器件地址发送完成
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
  while(num)
  {
    if(num == 1)
    {   /*!< Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);//最后一个字节不产生应答信号         
        /*!< Send STOP Condition */
      I2C_GenerateSTOP(I2C1, ENABLE);//发送停止信号结束数据传输         
    }
    /*!< Test on EV7 and clear it */ //等待数据接收完成
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

    *buffer = I2C_ReceiveData(I2C1);  //读出数据
    buffer++;   //读出数据缓存地址递加
    num--;      //接收数据数目减1
  }
  
  return SUCCESS;
}

//by panyiqiangde@126.com , 15919218729  @ 2020.02.27
/*******************************************************************************
**函数名称:void I2C_Test(void)
**功能描述:向AT24C02里写满128个字节数据，再读出来，验证通过IIC接口写数据与读数据的正确性
**入口参数:
						无                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
**输出:无
*******************************************************************************/
/*void I2C_AT24C02_Test(void)
{
    unsigned int i;
    unsigned char I2c_Buf[128];
          
  
    printf("向AT24C02EEPROM写入的数据为：\n\r");
    for(i = 0; i < 128; i++)
    {   
      I2c_Buf[i] = i ;
      printf("0x%x\t" , I2c_Buf[i]);
      if(i%16 == 15)
      {
        printf("\n\r");
      }
    }
    printf("\n\r");
    //AT24C02 每次最多只能写入8个字节的数据，所以要分段写
    IIC_Write(0xa0 , 0 , I2c_Buf , 8);
    Delayms(5);
    IIC_Write(0xa0 , 8 ,  &I2c_Buf[8] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 16 , &I2c_Buf[16] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 24 , &I2c_Buf[24] , 8);                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
    Delayms(5);
    IIC_Write(0xa0 , 32 , &I2c_Buf[32] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 40 , &I2c_Buf[40] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 48 , &I2c_Buf[48] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 56 , &I2c_Buf[56] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 64 , &I2c_Buf[64] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 72 , &I2c_Buf[72] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 80 , &I2c_Buf[80] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 88 , &I2c_Buf[88] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 96 , &I2c_Buf[96] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 104 , &I2c_Buf[104] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 112 , &I2c_Buf[112] , 8);
    Delayms(5);
    IIC_Write(0xa0 , 120 , &I2c_Buf[120] , 8);
  
  
    for(i = 0 ; i < 128 ;i++)		//清空缓存数据                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
    {   
      I2c_Buf[i] = 0;
    }
                                                                                                                                                                                                                                              // 15 9 -1 9 2 1-8 7 2 9
    printf("从AT24C02IIRPOM读出的数据为：\n\r");  //从AT24C02里读出数据
    IIC_Read(0xa0 , 0 , I2c_Buf , 128);
  
    //panyiqiangde@126.com , 159-1921-8729
    for(i = 0 ; i < 128 ; i++)
    {	
        if(I2c_Buf[i] != i)				//把读出的数据与写入的数据进行比较，看是否正确
        {
            printf("错误：I2C EEPROMAT24C02写入与读出的数据不一致\n\r");
            while(1);			//EEPROM AT24C02读写失败，停止等待
        }
        printf("0x%X\t", I2c_Buf[i]);
        if(i%16 == 15)
        {                                                                                                                                                                                                                                            // 1  5 9-19 21-8 72 9
            printf("\n\r");
        }
    }
    printf("\r\n\r\nEEPROM AT24C02读写一致，成功！！！\n\r");
}*/