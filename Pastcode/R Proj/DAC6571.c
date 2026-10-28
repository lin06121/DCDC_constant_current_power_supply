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

void DAC6571_Byte_Transmission(uint8_t byte_data) {
	uint8_t i,shelter;
	
    shelter = 0x80; // 掩码1000 0000，将掩码和byte_data交错可以获取最高位，随后将掩码右移

	for (i = 1; i <= 8; i++) {

		// 将byte_data与掩码交错，并依此输出
		if ((byte_data & shelter) == 0)    // SDA设置为Low,SDA_L;
            GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, 0x0);     
		else                               // SDA设置为High,SDA_H;
            GPIOPinWrite(SDA_PIN_BASE, SDA_PIN,SDA_PIN);     
        
        //SCL_H; SCL_L; 构造一个时钟周期，发送SDA上的数据
        GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, SCL_PIN);
        GPIOPinWrite(SCL_PIN_BASE, SCL_PIN,0x0);

		//右移掩码，查下一位
		shelter >>= 1;
	}

	//这一位是ACK位，直接舍弃了
	//SDA_IN;SCL_H; SCL_L;SDA_OUT;
    GPIOPinTypeGPIOInput(SDA_PIN_BASE, SDA_PIN);     // SDA设置为输入
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, SCL_PIN);
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN,0x0);
    GPIOPinTypeGPIOOutput(SDA_PIN_BASE, SDA_PIN);    // SDA设置为输出
}

void DAC6571_Fastmode_Operation(uint32_t DAC6571_code) {

	uint8_t msbyte,lsbyte;
	// DAC code是一个0-1023的10bit变量，我们左移两位，使用中间10个bit
	// 信号的格式是 0 0 PD1 PD0 D9 D8 D7 D6    D5 D4 D3 D2 D1 D0 X X (DAC data sheet P16)
    uint32_t shifted_DAC_Code = DAC6571_code << 2;  // 左移两位

	// 初始信号，两线同时给一个1-0
	//SCL_H; SDA_H; SDA_L; SCL_L;       // START condition
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, SCL_PIN);
    GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, SDA_PIN); 
    GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, 0x0);
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, 0x0);

	// 第一位发送IIC地址
	DAC6571_Byte_Transmission(DAC6571_address);

	// 第二三位是16位数据信号
    msbyte = shifted_DAC_Code / 256;
	lsbyte = shifted_DAC_Code - msbyte * 256;
	DAC6571_Byte_Transmission(msbyte);
	DAC6571_Byte_Transmission(lsbyte);

    //SDA_L; SCL_H; SDA_H;        // STOP condition
    GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, 0x0);
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, SCL_PIN);
    GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, SDA_PIN); 
}
