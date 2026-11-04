//*****************************************************************************
//
// DAC6571.h - Prototypes for the DAC6571 driver.
//
// Copyright：2020-2021, 上海交通大学电子工程系实验教学中心
// 
// Author:	上海交通大学电子工程系实验教学中心
// Version: 1.0.0.20201203 
// Date：2020-12-03
// History：
//
//*****************************************************************************

#ifndef __DAC6571_H__
#define __DAC6571_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"        // 基址宏定义
#include "inc/hw_types.h"         // 数据类型宏定义，寄存器访问函数
#include "inc/hw_i2c.h"
#include "driverlib/debug.h"      // 调试用
#include "driverlib/gpio.h"       // 通用IO口宏定义
#include "driverlib/pin_map.h"    // TM4C系列MCU外围设备管脚宏定义
#include "driverlib/sysctl.h"	  // 系统控制定义
#include "driverlib/i2c.h"

    
////DAC6571操作

// 用于连接DAC6571的TM4C1294 GPIO引脚定义
#define SDA_PIN_BASE  GPIO_PORTL_BASE
#define SDA_PIN       GPIO_PIN_0
#define SCL_PIN_BASE  GPIO_PORTL_BASE
#define SCL_PIN       GPIO_PIN_1

#define DAC6571_code_max        1024  // DAC6571是10bits的DAC, 2^10=1024
#define DAC6571_address         0x98  // DAC6571的地址，1001 10 A0 0  A0=0

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
	
// 完成一次DAC转换
extern void DAC6571_Fastmode_Operation(uint32_t);

// 向DAC6571串行输入1个字节（8位）数据
extern void DAC6571_Byte_Transmission(uint8_t byte_data);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __DAC6571_H__
