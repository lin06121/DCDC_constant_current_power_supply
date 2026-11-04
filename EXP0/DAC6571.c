//*****************************************************************************
//
// DAC6571.c - API for DAC6571.
//
// Copyright：2020-2021, 上海交通大学电子工程系实验教学中心
// 
// Author:	上海交通大学电子工程系实验教学中心
// Version: 1.0.0.20201203 
// Date：2020-12-03
// History：
//
//*****************************************************************************

#include "DAC6571.h"
extern uint32_t ui32SysClock;

//*****************************************************************************
// 
// 函数原型：void DAC6571_Byte_Transmission(uint8_t byte_data)
// 函数功能：向DAC6571串行输入1个字节（8位）数据
// 函数参数：byte_data  8位输入（从高位到低位依次输入）
// 函数返回值： 无
//
//*****************************************************************************
void DAC6571_Byte_Transmission(uint8_t byte_data)
{
	uint8_t i,shelter;
	
    shelter = 0x80;

	for (i = 1; i <= 8; i++)
	{
		if ((byte_data & shelter) == 0)    // SDA设置为Low,SDA_L;
            GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, 0x0);     
		else                               // SDA设置为High,SDA_H;
            GPIOPinWrite(SDA_PIN_BASE, SDA_PIN,SDA_PIN);     
        
        //SCL_H; SCL_L;
        GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, SCL_PIN);
        GPIOPinWrite(SCL_PIN_BASE, SCL_PIN,0x0);
        
		shelter >>= 1;
	}
	//SDA_IN;SCL_H; SCL_L;SDA_OUT;
    GPIOPinTypeGPIOInput(SDA_PIN_BASE, SDA_PIN);     // SDA设置为输入
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, SCL_PIN);
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN,0x0);
    GPIOPinTypeGPIOOutput(SDA_PIN_BASE, SDA_PIN);    // SDA设置为输出
}

//*****************************************************************************
// 
// 函数原型：void DAC6571_Fastmode_Operation(uint32_t DAC6571_code)
// 函数功能：完成一次DAC转换
// 函数参数：DAC6571_code 待转换的D/A数据
// 函数返回值： 无
//
//*****************************************************************************
void DAC6571_Fastmode_Operation(uint32_t DAC6571_code)
{
	uint8_t msbyte,lsbyte;
    uint32_t DAC6571_code1 = DAC6571_code << 2;  // 左移两位
    
	//SCL_H; SDA_H; SDA_L; SCL_L;       // START condition
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, SCL_PIN);
    GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, SDA_PIN); 
    GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, 0x0);
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, 0x0);
    
	DAC6571_Byte_Transmission(DAC6571_address);

    msbyte = DAC6571_code1 / 256;
	lsbyte = DAC6571_code1 - msbyte * 256;
	DAC6571_Byte_Transmission(msbyte);
	DAC6571_Byte_Transmission(lsbyte);

    //SDA_L; SCL_H; SDA_H;        // STOP condition
    GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, 0x0);
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, SCL_PIN);
    GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, SDA_PIN); 
}
