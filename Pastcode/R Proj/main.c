//*****************************************************************************
//
// Copyright: 2019-2020, 上海交通大学工程实践与科技创新II-A教学组
// File name: exp2_0.c
// Description: 
//    1.开机或复位后，底板上右边4位数码管自动显示计时数值，最低位对应单位是0.1秒；
//    2.开机或复位后，底板上8个LED灯以跑马灯形式由左向右循环变换，约0.5秒变换1次；
//    3.当没有按键按下时，从左边数第二位数码管显示“0”；
//      当人工按下某键，数码管显示该键的编号；
//      此刻四位计时数码管暂停变化，停止计时，直到放开按键后自动继续计时。
// Author:	上海交通大学工程实践与科技创新II-A教学组
// Version: 1.0.0.20200208 
// Date：2020-02-08
// History：
//
//*****************************************************************************

#ifndef PART_TM4C1294NCPDT
#define PART_TM4C1294NCPDT
#endif


// 头文件
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"        // 基址宏定义
#include "inc/hw_types.h"         // 数据类型宏定义，寄存器访问函数
#include "driverlib/debug.h"      // 调试用
#include "driverlib/gpio.h"       // 通用IO口宏定义
#include "driverlib/pin_map.h"    // TM4C系列MCU外围设备管脚宏定义
#include "driverlib/sysctl.h"	  // 系统控制定义
#include "driverlib/systick.h"    // SysTick Driver 原型
#include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver 原型

#include "tm1638.h"               // 与控制TM1638芯片有关的函数

#include "adc.h"
#include "DAC6571.h"

// ---------------------------------------------------------
// 宏定义， 主要是记时相关的常数
// ---------------------------------------------------------

// SysTick频率为100Hz，即循环定时周期10ms
#define SYSTICK_FREQUENCY		100

#define V_T10ms	10             // 0.1s软件定时器溢出值，10个10ms
#define V_T500ms	50             // 0.5s软件定时器溢出值，50个10ms

// ---------------------------------------------------------
// 函数
// ---------------------------------------------------------

void GPIOInit(void);
void SysTickInit(void);
void DevicesInit(void);
void DetectKey(void);
void ShowError(void);
void ClearError(void);
void LightBlink(void);
void ImplantNumer(uint32_t x, bool High);

//--------------------------------------------------------------
// 记时相关变量
//--------------------------------------------------------------

// 系统时钟频率
uint32_t ui32SysClock;
// 软件定时器计数
uint8_t clock10ms = 0;	uint8_t clock10ms_flag = 0;
uint8_t clock500ms = 0;	uint8_t clock500ms_flag = 0;

// 数码管相关变量
//--------------------------------------------------------------

// 8位数码管显示的数字或字母符号
// 注：板上数码位从左到右序号排列为4、5、6、7、0、1、2、3
uint8_t digit[8]={' ',' ',' ',' ','_',' ','_',' '};
// 8位小数点 1亮  0灭
// 注：板上数码位小数点从左到右序号排列为4、5、6、7、0、1、2、3
uint8_t pnt = 0x11;
// 8个LED指示灯状态，0灭，1亮
// 注：板上指示灯从左到右序号排列为7、6、5、4、3、2、1、0
//     对应元件LED8、LED7、LED6、LED5、LED4、LED3、LED2、LED1
// 这里led数组必须有9个地址，否则key_code会被越界修改，不知道为什么
uint8_t led[9] = {1, 1, 1, 1, 1, 1, 1, 1};

// 底板信息
//--------------------------------------------------------------

// 当前按键值
uint8_t key_code = 0;
bool key_state_pre[9] = {0,0,0, 0,0,0, 0,0,0};
// 九个按键位的flag值
bool key_flag[9] ;

// ADC设备传回来的值
extern uint32_t adc_opt[4];
// 对外输出的测定电流，0-1023
uint16_t CurrentSet=0;

// 使用的全局变量
int i;
uint8_t StateMachine = 0;
const uint8_t StateTotal = 3;

int main(void) {

	// 初始化代码段
	DevicesInit();            //  MCU器件初始化
	while (clock10ms < 3);   // 延时>60ms,等待TM1638上电完成
	TM1638_Init();	          // 初始化TM1638
	LightBlink();
	
	while (1) {
		// 更新所有的flag行为
		// 0.1s 更新adc采样
		if (clock10ms_flag == 1) {
			clock10ms_flag = 0;
			ADC_Update();
		}
		// 0.5s 更新
		if (clock500ms_flag == 1){
			clock500ms_flag = 0;
			DAC6571_Fastmode_Operation(CurrentSet<1000?CurrentSet:1000);
			LightBlink();
		}
		// 左按键
		if (key_flag[4]) { key_flag[4] = 0;
			StateMachine = StateMachine==StateTotal-1 ? 0 : StateMachine+1;
		}
		// 右按键
		if (key_flag[6]) {
			key_flag[6] = 0;
			StateMachine = StateMachine==0 ? StateTotal-1 : StateMachine-1;
		}
		// 上按键
		if (key_flag[2]) {
			key_flag[2] = 0;
			CurrentSet = CurrentSet+1;
		}
		// 下按键
		if (key_flag[8]) {
			key_flag[8] = 0;
			CurrentSet = CurrentSet-1;
		}
		// 就是这句话，会导致led越界，目前不知道为什么
		for ( i=0; i<8; i++) led[i]= (i!=StateMachine);
		//更新状态机
		switch (StateMachine) {
			case 0:
				ImplantNumer(adc_opt[0],1);
				ImplantNumer(adc_opt[1],0);
				break;
			case 1:
				ImplantNumer(adc_opt[2],1);
				ImplantNumer(adc_opt[3],0);//稳流源电流
				break;
			case 2:
				ImplantNumer(CurrentSet,1);
				ImplantNumer(adc_opt[3],0);
				break;
			default: break;
		}
	}
}

// 将数字x植入数码管，High=true对应左侧数码管，High=false对应右侧数码管
void ImplantNumer(uint32_t x, bool High) {
	digit[High*4+3]=x%10;
	digit[High*4+2]=(x/10)%10;
	digit[High*4+1]=(x/100)%10;
	digit[High*4+0]=(x/1000)%10;
}


// SysTick中断服务程序
void SysTick_Handler(void) {       // 定时周期为10ms

	//LightBlink();
	// 0.1秒钟软定时器计数
	if (++clock10ms >= V_T10ms) {
		clock10ms_flag = 1; // 当0.1秒到时，溢出标志置1
		clock10ms = 0;
	}
	
 	// 0.5秒钟软定时器计数
	if (++clock500ms >= V_T500ms) {
		clock500ms_flag = 1; // 当0.5秒到时，溢出标志置1
		clock500ms = 0;
	}

	// 刷新全部数码管和LED指示灯
	TM1638_RefreshDIGIandLED(digit, pnt, led);

	// 检查当前键盘输入，0代表无键操作，1-9表示有对应按键
	// 键号显示在一位数码管上
	DetectKey();
}
// 检测按键是否按下，存储在key_flag中
void DetectKey(void) {
	key_code = TM1638_Readkeyboard();
	switch (key_code) {
		case 2:	case 4:	case 5:	case 6:	case 8:
			if (key_state_pre[key_code]==0) { key_flag[key_code] = true;}
			break;
		default: break;
	}
	for (i=0;i<=8;i++) {key_state_pre[i] = (key_code==i) ;}
}


//GPIO初始化。使能PortK，设置PK4,PK5为输出；使能PortM，设置PM0为输出。
//  PK4连接TM1638的STB，PK5连接TM1638的DIO，PM0连接TM1638的CLK
void GPIOInit(void) {
	//配置TM1638芯片管脚
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);						// 使能端口 K
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){};		// 等待端口 K准备完毕

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);						// 使能端口 M
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){};		// 等待端口 M准备完毕

	// 设置端口 K的第4,5位（PK4,PK5）为输出引脚		PK4-STB  PK5-DIO
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4|GPIO_PIN_5);
	// 设置端口 M的第0位（PM0）为输出引脚   PM0-CLK
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);

	////配置LED输出引脚--Port F
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // 使能端口 F
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) ; // 等待端口 F 准备完毕
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0); // 设置 PF0 为输出

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);				// 使能端口 L
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){};		// 等待端口 L准备完毕
	// 设置端口 L的第0,1位（PL0,PL1）为输出引脚		PL0-SDA  PL1-SCL (DAC6571)
	GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1);
}
// 函数功能：设置SysTick中断
void SysTickInit(void) {
	SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // 设置心跳节拍,定时周期10ms
	SysTickEnable();  			// SysTick使能
	SysTickIntEnable();			// SysTick中断允许
}
// CU器件初始化，包括系统时钟设置、GPIO初始化和SysTick中断设置
void DevicesInit(void) {
	// 使用外部25MHz主时钟源，经过PLL，然后分频为20MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |
									   SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480),
									   20000000);

	ADCInit();
	GPIOInit();             // GPIO初始化
	SysTickInit();          // 设置SysTick中断
	IntMasterEnable();			// 总中断允许
}


// 这是一个预留的函数，用1294芯片上的LED灯表示系统发生了未预料的错误
// 当会此错误发生时，需从代码逻辑层面调试，无法通过人机界面操作解除
// 往往被用于各类Default调用，因为大多数Default语句是不被使用的
// 问题还有可能出现在状态机混乱(如在状态4显示状态2等)
void ShowError(void) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
}
// 用于清除点亮的LED灯，可以用来进行更麻烦一点的调试或者做闪烁
void ClearError(void) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
}
// 利用ShowError和ClearError制作的闪烁，会让LED灯闪烁一次，可以用于按键是否按下的调试
void LightBlink(void) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
    for (i=0;i < 5000;i++);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
}
