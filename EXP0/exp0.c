//*************************************************************************************
//
// 版权所有：2020-2021，上海交通大学电子信息与电气工程学院
// 文件名：adc_demo.c
// 描述：
//    1. 在 CH0/PE3、CH1/PE2 和 CH2/PE1 上进行三通道 ADC 采样。
//    2. 左侧 4 位数码管显示 CH0/PE3 电压 [0.000-3.300 V]。
//    3. 右侧 4 位数码管显示输出电流CH1/PE2，单位 A [0.000-9.999 A]。
//    4. Timer0A（1 kHz）触发采样 + 16 倍硬件过采样 + 滑动窗口平均。
//    5. 输入电压必须保持在 [0-3.3 V] 范围内，否则可能损坏引脚。
// 作者：上海交通大学电子信息与电气工程学院
// 版本：1.0.0.20201125
// 日期：2020-11-25
// 历史：
//		2026-10-16：扩展为三通道；定时器触发采样 + 硬件
//		            过采样 + 滑动平均；显示电压（mV）。
//
//*************************************************************************************

//*************************************************************************************
//
// 头文件
//
//*************************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"         // 中断向量号（INT_ADC0SS1 等）
#include "inc/hw_memmap.h"        // 基地址宏
#include "inc/hw_types.h"         // 数据类型和寄存器访问辅助宏
#include "driverlib/debug.h"      // 调试辅助
#include "driverlib/gpio.h"       // GPIO API
#include "driverlib/pin_map.h"    // TM4C 外设引脚复用宏定义
#include "driverlib/sysctl.h"	  // 系统控制 / 时钟
#include "driverlib/systick.h"    // SysTick 驱动
#include "driverlib/interrupt.h"  // NVIC 中断控制器驱动
#include "driverlib/adc.h"        // ADC API
#include "driverlib/timer.h"      // 定时器驱动（定时器触发 ADC）
#include "tm1638.h"               // TM1638 显示驱动
#include "uart_debug.h"           // UART 调试输出模块

//*************************************************************************************
//
// 宏定义
//
//*************************************************************************************
#define SYSTICK_FREQUENCY	50		// SysTick 频率 = 50 Hz -> 20 ms 循环周期

#define V_T100ms 5                      // 0.1 s 软件定时器溢出：5 * 20 ms（UI 刷新频率 @10 Hz）

// ADC 采样和滤波参数
#define ADC_NUM_CH        3             // ADC 通道数：CH0/PE3、CH1/PE2、CH2/PE1
#define ADC_TRIGGER_HZ    1000          // Timer0A ADC 触发频率：1 kHz
#define ADC_OVS_FACTOR    16            // 硬件过采样 16 倍（原始采样率 16 kHz）
#define ADC_WINDOW_SIZE   20            // 软件滑动窗口大小：20 个样本（20 ms，恰好陷波 50/100 Hz）
#define ADC_CONST_MV      3300          // 满量程 3.300 V，单位 mV

// ---- 电流通道校准（0.1 欧姆采样电阻，PE2/CH1） ----
// 针对万用表测量值进行最小二乘拟合：
//     I(mA) = 0.665761 * V_adc(mV) + 28.453，其中 V_adc(mV) = RawToMilliVolts(raw)
// 斜率乘以 65536 以使用定点数学（避免浮点）；由此产生的偏移误差低于 1 mA 显示分辨率。
#define CAL_I_MA_PER_MVQ   43632u       // 0.665761 * 65536 ~= 43632（mA 每 ADC-mV）
#define CAL_I_MA_OFFSET    28u          // 截距 b = 28.453 mA，四舍五入

// ---- 显示滞回（变化达到该阈值才刷新数码管，避免末位抖动） ----
// 电压单位为 mV、电流单位为 mA，均以 X.YYY 显示（1 LSB = 0.001 V / 0.001 A）。
// 仅当新值与数码管当前显示值之差达到该 LSB 数时才更新数码管；串口打印不受影响。
#define DISP_V_DEADBAND    3           // 电压滞回：3 mV
#define DISP_I_DEADBAND    3           // 电流滞回：3 mA

//*************************************************************************************
//
// 函数原型
//
//*************************************************************************************
void GPIOInit(void);        // GPIO 初始化
void ADCInit(void);         // ADC 初始化
void TimerADCInit(void);    // Timer0A 定时器触发 ADC 初始化
void SysTickInit(void);     // SysTick 中断设置
void DevicesInit(void);     // MCU 外设初始化；调用上述函数
uint32_t RawToMilliVolts(uint32_t raw);     // 将 ADC 原始值转换为毫伏
uint32_t CurrentToMilliAmps(uint32_t raw);  // 将 CH1/PE2 原始值转换为实际电流（mA）

//*************************************************************************************
//
// 变量
//
//*************************************************************************************

// 软件定时器计数器（在 SysTick ISR 中更新，在主循环中读取 -> volatile）
volatile uint8_t clock100ms = 0;

// 软件定时器溢出标志（在 SysTick ISR 中更新，在主循环中读取 -> volatile）
volatile uint8_t clock100ms_flag = 0;

// 8 位数码管显示字符
// 注意：板上物理数码管从左到右顺序为 4,5,6,7,0,1,2,3
uint8_t digit[8]={' ',' ',' ',' ',' ',' ',' ',' '};

// 8 个小数点；1 = 亮，0 = 灭
// 注意：板上小数点从左到右顺序为 4,5,6,7,0,1,2,3
uint8_t pnt = 0x0;

// 8 个 LED 指示状态；0 = 灭，1 = 亮
// 注意：板上 LED 从左到右顺序为 7,6,5,4,3,2,1,0（LED8..LED1）
uint8_t led[] = {0, 0, 0, 0, 0, 0, 0, 0};

// 系统时钟频率
uint32_t ui32SysClock;

// ADC 滑动窗口滤波状态（在 ADC ISR 中更新，在主循环中读取 -> volatile）
volatile uint32_t g_ui32ADCWin[ADC_NUM_CH][ADC_WINDOW_SIZE]; // 每通道环形缓冲区
volatile uint32_t g_ui32ADCWinSum[ADC_NUM_CH];               // 运行和（O(1) 更新）
volatile uint8_t  g_ui8ADCWinIdx = 0;                        // 环形缓冲区写入索引
volatile uint8_t  g_ui8ADCWinCnt = 0;                        // 已填充样本数
volatile uint32_t g_ui32ADCAvg[ADC_NUM_CH];                  // 最新每通道平均值（0-4095）

// 数码管当前实际显示的值（滞回目标），单位 mV / mA。初值取 0xFFFFFFFF，使上电
// 首拍强制刷新一次（否则输入为 0 时会一直保持空白）。
uint32_t g_ui32DispVoltage = 0xFFFFFFFF;
uint32_t g_ui32DispCurrent = 0xFFFFFFFF;

// 两个无符号值的绝对差，用于显示滞回判断。
static uint32_t DiffU32(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

//*************************************************************************************
//
// 主程序
//
//*************************************************************************************
int main(void)
{
	DevicesInit();            // MCU 外设初始化

	while (clock100ms < 3);   // 等待 >60 ms，等待 TM1638 上电完成
	TM1638_Init();	          // 初始化 TM1638

	while (1)
	{
        if (clock100ms_flag == 1)       // 到达 100 ms 节拍（UI 刷新频率 @10 Hz）
        {
            uint32_t mv0, mv1, mv2, ima;

            clock100ms_flag = 0;

            // 读取平均后的原始值并转换为毫伏
            mv0 = RawToMilliVolts(g_ui32ADCAvg[0]);   // CH0/PE3（输出电压）
            mv1 = RawToMilliVolts(g_ui32ADCAvg[1]);   // CH1/PE2（电流采样电压）
            mv2 = RawToMilliVolts(g_ui32ADCAvg[2]);   // CH2/PE1（预留；调试 / 阶段 3）
            ima = CurrentToMilliAmps(g_ui32ADCAvg[1]); // CH1/PE2 实际电流（mA）

            // 仅当数值相对当前显示的增量达到滞回阈值时才刷新数码管，避免微小噪声导致
            // 末位抖动。下面的串口输出仍每拍打印原始值（mv0/ima），不受滞回影响。
            if (DiffU32(mv0, g_ui32DispVoltage) >= DISP_V_DEADBAND)
            {
                g_ui32DispVoltage = mv0;
                digit[4] = mv0 / 1000; 	  // 整数位（0-3）
                digit[5] = mv0 / 100 % 10;  // 十分位
                digit[6] = mv0 / 10 % 10;   // 百分位
                digit[7] = mv0 % 10;        // 千分位
            }

            if (DiffU32(ima, g_ui32DispCurrent) >= DISP_I_DEADBAND)
            {
                g_ui32DispCurrent = ima;
                digit[0] = ima / 1000; 	  // 整数位（A）
                digit[1] = ima / 100 % 10;  // 0.1 A
                digit[2] = ima / 10 % 10;   // 0.01 A
                digit[3] = ima % 10;        // 0.001 A
            }

            // 点亮每组的第一位小数点 -> 显示 "X.YYY"
            pnt = (1 << 4) | (1 << 0);

            // UART 调试输出：三路电压 + 原始值，@10 Hz
            UART_DebugADCDump(g_ui32ADCAvg[0], g_ui32ADCAvg[1], g_ui32ADCAvg[2],
                              mv0, mv1, mv2, ima);
        }
	}
}

//*************************************************************************************
//
// 函数：	void GPIOInit(void)
// 功能：	GPIO 初始化。使能 Port K，并将 PK4/PK5 设置为输出；使能
//              Port M，并将 PM0 设置为输出。
//              （PK4 = TM1638 STB，PK5 = TM1638 DIO，PM0 = TM1638 CLK）
// 参数：	无
// 返回值：	无
//
//*************************************************************************************
void GPIOInit(void)
{
	// 配置 TM1638 芯片引脚
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);				// 使能 Port K
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){};		// 等待 Port K 就绪

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);				// 使能 Port M
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){};		// 等待 Port M 就绪

   // 将 Port K 引脚 4、5（PK4、PK5）设置为输出		PK4-STB  PK5-DIO
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4|GPIO_PIN_5);
	// 将 Port M 引脚 0（PM0）设置为输出   PM0-CLK
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);
}

//*************************************************************************************
//
// 函数：	void ADCInit(void)
// 功能：	ADC0 初始化。选择 CH0/PE3、CH1/PE2 和 CH2/PE1 作为 ADC0 输入；
//              使用采样序列器 1，由 Timer0A 触发，并使能其中断。
// 参数：	无
// 返回值：	无
//
//*************************************************************************************
void ADCInit(void)
{
    // 使能 ADC0 模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // 使能 Port E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // 将 CH0/PE3、CH1/PE2 和 CH2/PE1 引脚用作 ADC 输入
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1);

    // 使用采样序列器 1，定时器触发，优先级 0
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);

    // 使能硬件过采样（自动平均 16 个样本）；结合
    // 硬件触发，形成针对快速开关纹波的低通滤波器
    ADCHardwareOversampleConfigure(ADC0_BASE, ADC_OVS_FACTOR);

    // 采样序列器 1，步骤 0：采样 CH0
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);

    // 采样序列器 1，步骤 1：采样 CH1
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);

    // 采样序列器 1，步骤 2：
    // ADC_CTL_CH2 -- 采样 CH2；ADC_CTL_END -- 序列最后一步；
    // ADC_CTL_IE -- 序列完成时产生中断
    ADCSequenceStepConfigure( ADC0_BASE, 1, 2,
                              ADC_CTL_CH2 | ADC_CTL_END | ADC_CTL_IE);

    // 使能采样序列器 1
    ADCSequenceEnable(ADC0_BASE, 1);

    // 使能 ADC0 序列器 1 中断及其 NVIC 中断源
    ADCIntEnable(ADC0_BASE, 1);
    IntEnable(INT_ADC0SS1);

    // 采样前清除中断状态标志
    ADCIntClear(ADC0_BASE, 1);
}

//*************************************************************************************
//
// 函数：	void TimerADCInit(void)
// 功能：	将 Timer0A 配置为 1 kHz 周期定时器，其输出异步触发 ADC0
//              采样序列器 1（无需 CPU 参与）。
// 参数：	无
// 返回值：	无
//
//*************************************************************************************
void TimerADCInit(void)
{
    // 使能 Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){};

    // 将 Timer0A 配置为分离、周期、向下计数定时器
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

    // 设置重装载值以产生 ADC_TRIGGER_HZ（1 kHz）
    TimerLoadSet(TIMER0_BASE, TIMER_A, (ui32SysClock / ADC_TRIGGER_HZ) - 1);

    // 使能 Timer0A ADC 触发输出
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    // 启动 Timer0A；它现在会连续触发 ADC 采样
    TimerEnable(TIMER0_BASE, TIMER_A);
}

//*************************************************************************************
//
// 函数：	uint32_t RawToMilliVolts(uint32_t raw)
// 功能：	将 ADC 原始值 [0-4095] 转换为毫伏 [0-3300]。
//              阶段 2/3 校准可在此处插入 k*x+b 线性修正。
// 参数：	raw   ADC 原始采样值
// 返回值：	毫伏值
//
//*************************************************************************************
uint32_t RawToMilliVolts(uint32_t raw)
{
    // mv = raw * 3300 / 4096；加 2048 用于四舍五入；>>12 代替除以 4096
    return (raw * ADC_CONST_MV + 2048) >> 12;
}

//*************************************************************************************
//
// 函数：	uint32_t CurrentToMilliAmps(uint32_t raw)
// 功能：	将 CH1/PE2 的 ADC 原始值转换为实际输出电流（mA）。
//              0.1 欧姆采样电阻上的压降经前端放大后在 PE2 读取；
//              实测拟合为 I(mA) = 0.665761 * V_adc(mV) + 28.453，使用定点实现。
// 参数：	raw   CH1 原始采样值
// 返回值：	实际电流，单位 mA
//
//*************************************************************************************
uint32_t CurrentToMilliAmps(uint32_t raw)
{
    uint32_t mv = RawToMilliVolts(raw);   // PE2 引脚电压（mV）
    return ((CAL_I_MA_PER_MVQ * mv + 32768) >> 16) + CAL_I_MA_OFFSET;
}

//*************************************************************************************
//
// 函数：	void ADC0SS1_Handler(void)
// 功能：	ADC0 采样序列器 1 中断服务程序（1 kHz）；读取三个样本并
//              执行滑动窗口平均。
// 参数：	无
// 返回值：	无
//
//*************************************************************************************
void ADC0SS1_Handler(void)
{
    uint32_t data[8];                 // FIFO 深度为 8，足以容纳所有待处理样本
    int32_t  n, i, base;

    // 清除 ADC0 序列器 1 中断标志
    ADCIntClear(ADC0_BASE, 1);

    // 读取所有待处理 FIFO 样本（一次触发产生 ADC_NUM_CH 个样本；清空多余样本）
    n = ADCSequenceDataGet(ADC0_BASE, 1, data);
    if (n < ADC_NUM_CH)               // 还不是完整序列 -> 跳过
        return;

    // 取最近一次完整的三通道序列（data[base .. base+2] = CH0/CH1/CH2）
    base = n - ADC_NUM_CH;

    // 更新每个通道的环形缓冲区和运行和
    for (i = 0; i < ADC_NUM_CH; i++)
    {
        // 先减去即将被覆盖的值，再加上新值（O(1) 滑动平均）
        g_ui32ADCWinSum[i] -= g_ui32ADCWin[i][g_ui8ADCWinIdx];
        g_ui32ADCWin[i][g_ui8ADCWinIdx] = data[base + i];
        g_ui32ADCWinSum[i] += data[base + i];

        // 在窗口填满之前，按实际已填充的样本数求平均
        if (g_ui8ADCWinCnt < ADC_WINDOW_SIZE)
            g_ui32ADCAvg[i] = g_ui32ADCWinSum[i] / (g_ui8ADCWinCnt + 1);
        else
            g_ui32ADCAvg[i] = g_ui32ADCWinSum[i] / ADC_WINDOW_SIZE;
    }

    // 推进写指针并累计已填充样本数
    g_ui8ADCWinIdx = (g_ui8ADCWinIdx + 1) % ADC_WINDOW_SIZE;
    if (g_ui8ADCWinCnt < ADC_WINDOW_SIZE)
        g_ui8ADCWinCnt++;
}

//*************************************************************************************
//
// 函数：	void SysTickInit(void)
// 功能：	配置 SysTick 中断。
// 参数：	无
// 返回值：	无
//
//*************************************************************************************
void SysTickInit(void)
{
    SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // 设置节拍周期（20 ms）
    SysTickEnable();  			// 使能 SysTick
    SysTickIntEnable();			// 使能 SysTick 中断
}

//*************************************************************************************
//
// 函数：	void DevicesInit(void)
// 功能：	MCU 外设初始化：系统时钟、GPIO、ADC、定时器触发
//              ADC 和 SysTick 中断设置。
// 参数：	无
// 返回值：	无
//
//*************************************************************************************
void DevicesInit(void)
{
	// 通过 PLL 使用外部 25 MHz 晶振，分频到 20 MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |
	                                   SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480),
	                                   20000000);

  GPIOInit();             // GPIO 初始化
  ADCInit();              // ADC 初始化
  TimerADCInit();         // Timer0A 定时器触发 ADC 初始化
  UART_DebugInit(ui32SysClock);   // UART 调试输出初始化
  SysTickInit();          // SysTick 中断设置
  IntMasterEnable();	  // 使能全局中断
}

//*************************************************************************************
//
// 函数：	void SysTick_Handler(void)
// 功能：	SysTick 中断服务程序。
// 参数：	无
// 返回值：	无
//
//*************************************************************************************
void SysTick_Handler(void)       // 节拍周期 = 20 ms
{
	// 0.1 s 软件定时器（10 Hz），用于 UI 刷新 / 后续阶段 3 均流
	if (++clock100ms >= V_T100ms)
	{
		clock100ms_flag = 1; // 当 0.1 s 到时，设置溢出标志
		clock100ms = 0;
	}

	// 刷新所有数码管和 LED
	TM1638_RefreshDIGIandLED(digit, pnt, led);

}