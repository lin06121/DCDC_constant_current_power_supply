//*****************************************************************************
//
// uart_debug.c - UART 调试输出模块实现
//
// 参�? E:\code\keil\EXP3\exp3-8.c �? UART 范式：UART0 挂在 PA0(RX)/PA1(TX)�?
// �? UARTConfigSetExpClk 配置 115200 8-N-1，发送用 UARTCharPut 阻塞发送�?
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "uart_debug.h"

//*****************************************************************************
//
// 函数原型：void UART_DebugInit(uint32_t ui32SysClock)
// 函数功能：使�? UART0/GPIOA，配�? PA0(RX)/PA1(TX)，波特率 115200 8-N-1
// 函数参数：ui32SysClock  系统时钟频率
// 函数返回值：�?
//
//*****************************************************************************
void UART_DebugInit(uint32_t ui32SysClock)
{
    // 使能 UART0 �? GPIOA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){};

    // 配置 PA0(U0RX) / PA1(U0TX) �? UART 引脚
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // 配置波特�? 115200, 8-N-1
    UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
}

//*****************************************************************************
//
// 函数原型：void UART_DebugString(const char *s)
// 函数功能：阻塞发送以 '\0' 结尾的字符串
// 函数参数：s  待发送字符串
// 函数返回值：�?
//
//*****************************************************************************
void UART_DebugString(const char *s)
{
    while(*s != '\0')
    {
        UARTCharPut(UART0_BASE, *(s++));
    }
}

//*****************************************************************************
//
// 函数原型：void UART_DebugLine(const char *s)
// 函数功能：发送字符串并在末尾追加 "\r\n"
// 函数参数：s  待发送字符串
// 函数返回值：�?
//
//*****************************************************************************
void UART_DebugLine(const char *s)
{
    UART_DebugString(s);
    UART_DebugString("\r\n");
}

//*****************************************************************************
//
// 函数原型：void UART_DebugADCDump(...)
// 函数功能：以一行输出三�? ADC 的电压（X.YYY V）与原始值（0-4095�?
// 函数参数：raw0/raw1/raw2  三路平均原始�?
//           mv0/mv1/mv2     三路毫伏�?
// 函数返回值：�?
//
//*****************************************************************************
void UART_DebugADCDump(uint32_t raw0, uint32_t raw1, uint32_t raw2,
                       uint32_t mv0, uint32_t mv1, uint32_t mv2)
{
    char buf[64];

    sprintf(buf, "CH0=%u.%03uV(%u)  CH1=%u.%03uV(%u)  CH2=%u.%03uV(%u)\r\n",
            mv0 / 1000, mv0 % 1000, raw0,
            mv1 / 1000, mv1 % 1000, raw1,
            mv2 / 1000, mv2 % 1000, raw2);

    UART_DebugString(buf);
}