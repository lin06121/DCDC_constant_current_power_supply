//*************************************************************************************
//
// Copyright: 2020-2021, 上海交通大学电子工程系实验教学中心
// File name: adc_demo.c
// Description:
//    1.该示例展示利用CH0/PE3、CH1/PE2、CH2/PE1实现三路ADC采样；
//    2.左侧四个数码管显示CH0/PE3的ADC电压值[0.000-3.300V]；
//    3.右侧四个数码管显示CH1/PE2的ADC电压值[0.000-3.300V]；
//    4.采用Timer0A(1kHz)触发采样 + 硬件过采样16x + 滑动窗口平均滤波；
//    5.注意：输入电压值范围必须为[0-3.3V]，否则会烧坏端口。
// Author:	上海交通大学电子工程系实验教学中心
// Version: 1.0.0.20201125
// Date：2020-11-25
// History：
//		2026-10-16：扩展为三路采样，改为定时器触发+硬件过采样+滑动平均，显示电压值
//
//*************************************************************************************

//*************************************************************************************
//
// 头文件
//
//*************************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"         // 中断向量编号宏定义（INT_ADC0SS1等）
#include "inc/hw_memmap.h"        // 基址宏定义
#include "inc/hw_types.h"         // 数据类型宏定义，寄存器访问函数
#include "driverlib/debug.h"      // 调试用
#include "driverlib/gpio.h"       // 通用IO口宏定义
#include "driverlib/pin_map.h"    // TM4C系列MCU外围设备管脚宏定义
#include "driverlib/sysctl.h"	  // 系统控制定义
#include "driverlib/systick.h"    // SysTick Driver 原型
#include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver 原型
#include "driverlib/adc.h"        // 与ADC有关的定义
#include "driverlib/timer.h"      // Timer Driver 原型（定时器触发ADC）
#include "tm1638.h"               // 与控制TM1638芯片有关的函数
#include "uart_debug.h"           // UART调试输出模块

//*************************************************************************************
//
// 宏定义
//
//*************************************************************************************
#define SYSTICK_FREQUENCY		50		// SysTick频率为50Hz，即循环定时周期20ms

#define V_T100ms 5                      // 0.1s软件定时器溢出值，5个20ms（UI 10Hz刷新）

// ADC采样与滤波参数
#define ADC_NUM_CH        3             // ADC通道数：CH0/PE3、CH1/PE2、CH2/PE1
#define ADC_TRIGGER_HZ    1000          // Timer0A触发ADC频率：1kHz
#define ADC_OVS_FACTOR    16            // 硬件过采样16x（等效原始采样率16kHz）
#define ADC_WINDOW_SIZE   16            // 软件滑动窗口点数（16ms）
#define ADC_CONST_MV      3300          // 满量程3.300V，单位mV

//*************************************************************************************
//
// 函数原型声明
//
//*************************************************************************************
void GPIOInit(void);        // GPIO初始化
void ADCInit(void);         // ADC初始化
void TimerADCInit(void);    // Timer0A定时触发ADC初始化
void SysTickInit(void);     // 设置SysTick中断
void DevicesInit(void);     // MCU器件初始化，注：会调用上述函数
uint32_t RawToMilliVolts(uint32_t raw);  // ADC原始值换算为毫伏值

//*************************************************************************************
//
// 变量定义
//
//*************************************************************************************

// 软件定时器计数（SysTick中断中更新，主循环读取，故用volatile）
volatile uint8_t clock100ms = 0;

// 软件定时器溢出标志（SysTick中断中更新，主循环读取，故用volatile）
volatile uint8_t clock100ms_flag = 0;

// 8位数码管显示的数字或字母符号
// 注：板上数码位从左到右序号排列为4、5、6、7、0、1、2、3
uint8_t digit[8]={' ',' ',' ',' ',' ',' ',' ',' '};

// 8位小数点 1亮  0灭
// 注：板上数码位小数点从左到右序号排列为4、5、6、7、0、1、2、3
uint8_t pnt = 0x0;

// 8个LED指示灯状态，0灭，1亮
// 注：板上指示灯从左到右序号排列为7、6、5、4、3、2、1、0
//     对应元件LED8、LED7、LED6、LED5、LED4、LED3、LED2、LED1
uint8_t led[] = {0, 0, 0, 0, 0, 0, 0, 0};

// 系统时钟频率
uint32_t ui32SysClock;

// ADC滑动窗口滤波状态（ADC中断中更新，主循环中读取，故用volatile）
volatile uint32_t g_ui32ADCWin[ADC_NUM_CH][ADC_WINDOW_SIZE]; // 各通道环形缓冲
volatile uint32_t g_ui32ADCWinSum[ADC_NUM_CH];               // 运行和（O(1)增量更新）
volatile uint8_t  g_ui8ADCWinIdx = 0;                        // 环形缓冲写指针
volatile uint8_t  g_ui8ADCWinCnt = 0;                        // 已填充点数
volatile uint32_t g_ui32ADCAvg[ADC_NUM_CH];                  // 各通道最新平均原始值[0-4095]

//*************************************************************************************
//
// 主程序
//
//*************************************************************************************
 int main(void)
{
	DevicesInit();            //  MCU器件初始化

	while (clock100ms < 3);   // 延时>60ms,等待TM1638上电完成
	TM1638_Init();	          // 初始化TM1638

	while (1)
	{
        if (clock100ms_flag == 1)       // 检查100ms定时是否到（10Hz刷新）
        {
            uint32_t mv0, mv1, mv2;

            clock100ms_flag = 0;

			// 读取滑动平均后的原始值，换算为毫伏值
            mv0 = RawToMilliVolts(g_ui32ADCAvg[0]);   // CH0/PE3
            mv1 = RawToMilliVolts(g_ui32ADCAvg[1]);   // CH1/PE2
            mv2 = RawToMilliVolts(g_ui32ADCAvg[2]);   // CH2/PE1（暂不显示，用于调试/阶段三）

            // 左侧4位显示CH0/PE3电压，格式X.YYY（整数位+三位小数）
            digit[4] = mv0 / 1000; 	  // 整数位（0-3）
            digit[5] = mv0 / 100 % 10;  // 十分位
            digit[6] = mv0 / 10 % 10;   // 百分位
            digit[7] = mv0 % 10;        // 千分位

            // 右侧4位显示CH1/PE2电压，格式X.YYY
            digit[0] = mv1 / 1000; 	  // 整数位（0-3）
            digit[1] = mv1 / 100 % 10;  // 十分位
            digit[2] = mv1 / 10 % 10;   // 百分位
            digit[3] = mv1 % 10;        // 千分位

            // 两组各点亮第一位小数点，显示为 "X.YYY"
            pnt = (1 << 4) | (1 << 0);

            // 串口调试输出三路电压与原始值（10Hz）
            UART_DebugADCDump(g_ui32ADCAvg[0], g_ui32ADCAvg[1], g_ui32ADCAvg[2],
                              mv0, mv1, mv2);
        }
	}
}

//*************************************************************************************
//
// 函数原型：void GPIOInit(void)
// 函数功能：GPIO初始化。使能PortK，设置PK4,PK5为输出；使能PortM，设置PM0为输出。
//          （PK4连接TM1638的STB，PK5连接TM1638的DIO，PM0连接TM1638的CLK）
// 函数参数：无
// 函数返回值：无
//
//*************************************************************************************
void GPIOInit(void)
{
	//配置TM1638芯片管脚
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);				// 使能端口 K
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){};		// 等待端口 K准备完毕

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);				// 使能端口 M
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){};		// 等待端口 M准备完毕

   // 设置端口 K的第4,5位（PK4,PK5）为输出引脚		PK4-STB  PK5-DIO
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4|GPIO_PIN_5);
	// 设置端口 M的第0位（PM0）为输出引脚   PM0-CLK
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);
}

//*************************************************************************************
//
// 函数原型：void ADCInit(void)
// 函数功能：ADC0初始化。选择CH0/PE3、CH1/PE2和CH2/PE1作为ADC0采样输入，
//          选用采样序列产生器1，由Timer0A定时器触发，并使能中断。
// 函数参数：无
// 函数返回值：无
//
//*************************************************************************************
void ADCInit(void)
{
    // 使能ADC0模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // 使能端口E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // 使用CH0/PE3、CH1/PE2和CH2/PE1引脚作为ADC输入
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1);

    // 选用采样序列产生器1，采样起始信号由定时器触发，优先级为0
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);

    // 开启硬件过采样（16次采样自动平均），与底层高速触发配合构成低通滤波
    ADCHardwareOversampleConfigure(ADC0_BASE, ADC_OVS_FACTOR);

    // 设置采样序列产生器1第0步骤:选择CH0为采样通道
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);

    // 设置采样序列产生器1第1步骤:选择CH1为采样通道
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);

    // 设置采样序列产生器1第2步骤:
    // ADC_CTL_CH2--选择CH2为采样通道， ADC_CTL_END--采样序列的最后一步
    // ADC_CTL_IE--采样结束后产生中断
    ADCSequenceStepConfigure( ADC0_BASE, 1, 2,
                              ADC_CTL_CH2 | ADC_CTL_END | ADC_CTL_IE);

    // 使能采样序列产生器1
    ADCSequenceEnable(ADC0_BASE, 1);

    // 使能ADC0采样序列1中断，并在NVIC中使能相应中断源
    ADCIntEnable(ADC0_BASE, 1);
    IntEnable(INT_ADC0SS1);

    // 在采样前，必须清除中断状态标志
    ADCIntClear(ADC0_BASE, 1);
}

//*************************************************************************************
//
// 函数原型：void TimerADCInit(void)
// 函数功能：配置Timer0A为1kHz周期定时，输出触发ADC0采样序列1。
//          即底层硬件以高频自动触发采样，无需CPU轮询触发。
// 函数参数：无
// 函数返回值：无
//
//*************************************************************************************
void TimerADCInit(void)
{
    // 使能定时器0模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){};

    // 配置Timer0A为分离定时器、周期模式（向下计数）
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

    // 设定重装载值，产生ADC_TRIGGER_HZ（1kHz）的触发频率
    TimerLoadSet(TIMER0_BASE, TIMER_A, (ui32SysClock / ADC_TRIGGER_HZ) - 1);

    // 使能Timer0A的ADC触发输出
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    // 启动Timer0A，开始连续触发ADC采样
    TimerEnable(TIMER0_BASE, TIMER_A);
}

//*************************************************************************************
//
// 函数原型：uint32_t RawToMilliVolts(uint32_t raw)
// 函数功能：将ADC原始值[0-4095]换算为毫伏值[0-3300]。
//          阶段二/三进行定标时，可在此加入 k*x+b 系数。
// 函数参数：raw  ADC原始采样值
// 函数返回值：毫伏值
//
//*************************************************************************************
uint32_t RawToMilliVolts(uint32_t raw)
{
    // mv = raw * 3300 / 4096，加2048做四舍五入，>>12代替除以4096（整数运算）
    return (raw * ADC_CONST_MV + 2048) >> 12;
}

//*************************************************************************************
//
// 函数原型：void ADC0SS1_Handler(void)
// 函数功能：ADC0采样序列1中断服务程序（1kHz），读取三路采样并做滑动窗口平均。
// 函数参数：无
// 函数返回值：无
//
//*************************************************************************************
void ADC0SS1_Handler(void)
{
    uint32_t data[8];                 // FIFO深度为8，足够容纳全部待读样本
    int32_t  n, i, base;

    // 清除ADC0序列1中断标志
    ADCIntClear(ADC0_BASE, 1);

    // 读取FIFO中当前全部样本（一次触发产生ADC_NUM_CH个，多余样本一并取走）
    n = ADCSequenceDataGet(ADC0_BASE, 1, data);
    if (n < ADC_NUM_CH)               // 不足一个完整序列（如序列尚未结束），跳过
        return;

    // 取最近一个完整的三路序列（data[base .. base+2] 即 CH0/CH1/CH2）
    base = n - ADC_NUM_CH;

    // 各通道更新环形缓冲区与运行和
    for (i = 0; i < ADC_NUM_CH; i++)
    {
        // 先减去即将被覆盖的旧值，再加新值，实现O(1)滑动平均
        g_ui32ADCWinSum[i] -= g_ui32ADCWin[i][g_ui8ADCWinIdx];
        g_ui32ADCWin[i][g_ui8ADCWinIdx] = data[base + i];
        g_ui32ADCWinSum[i] += data[base + i];

        // 窗口未满前按已填充点数平均，满后除以窗口大小
        if (g_ui8ADCWinCnt < ADC_WINDOW_SIZE)
            g_ui32ADCAvg[i] = g_ui32ADCWinSum[i] / (g_ui8ADCWinCnt + 1);
        else
            g_ui32ADCAvg[i] = g_ui32ADCWinSum[i] / ADC_WINDOW_SIZE;
    }

    // 前进写指针，并累计已填充点数
    g_ui8ADCWinIdx = (g_ui8ADCWinIdx + 1) % ADC_WINDOW_SIZE;
    if (g_ui8ADCWinCnt < ADC_WINDOW_SIZE)
        g_ui8ADCWinCnt++;
}

//*************************************************************************************
//
// 函数原型：SysTickInit(void)
// 函数功能：设置SysTick中断
// 函数参数：无
// 函数返回值：无
//
//*************************************************************************************
void SysTickInit(void)
{
    SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // 设置心跳节拍,定时周期20ms
    SysTickEnable();  			// SysTick使能
    SysTickIntEnable();			// SysTick中断允许
}

//*************************************************************************************
//
// 函数原型：DevicesInit(void)
// 函数功能：MCU器件初始化，包括系统时钟设置、GPIO初始化、ADC初始化、
//          定时器触发ADC初始化和SysTick中断设置
// 函数参数：无
// 函数返回值：无
//
//*************************************************************************************
void DevicesInit(void)
{
	// 使用外部25MHz主时钟源，经过PLL，然后分频为20MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |
	                                   SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480),
	                                   20000000);

  GPIOInit();             // GPIO初始化
  ADCInit();              // ADC初始化
  TimerADCInit();         // Timer0A定时触发ADC初始化
  UART_DebugInit(ui32SysClock);   // UART调试输出初始化
  SysTickInit();          // 设置SysTick中断
  IntMasterEnable();	  // 总中断允许
}

//*************************************************************************************
//
// 函数原型：void SysTick_Handler(void)
// 函数功能：SysTick中断服务程序
// 函数参数：无
// 函数返回值：无
//
//*************************************************************************************
void SysTick_Handler(void)       // 定时周期为20ms
{
	// 0.1秒钟软定时器计数（10Hz，用于UI刷新/后续阶段三均流控制）
	if (++clock100ms >= V_T100ms)
	{
		clock100ms_flag = 1; // 当0.1秒到时，溢出标志置1
		clock100ms = 0;
	}

	// 刷新全部数码管和LED
	TM1638_RefreshDIGIandLED(digit, pnt, led);

}