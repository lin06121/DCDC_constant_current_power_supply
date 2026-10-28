#ifndef PART_TM4C1294NCPDT
#define PART_TM4C1294NCPDT
#endif

#ifndef ADC_H
#define ADC_H

// 头文件
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/adc.h"
#include "inc/hw_memmap.h"        // 基址宏定义
#include "inc/hw_types.h"         // 数据类型宏定义，寄存器访问函数
#include "driverlib/debug.h"      // 调试用
#include "driverlib/gpio.h"       // 通用IO口宏定义
#include "driverlib/pin_map.h"    // TM4C系列MCU外围设备管脚宏定义
#include "driverlib/sysctl.h"	  // 系统控制定义
#include "driverlib/systick.h"    // SysTick Driver 原型
#include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver 原型

#include "tm1638.h"               // 与控制TM1638芯片有关的函数


// 用于展示的数字
extern uint32_t adc_opt[4];
extern bool adc_ready;
extern uint8_t adc_bfr_ptr;

void ADCInit(void);
void ADC_Sample(void);
void ADC_Update(void);

#endif
