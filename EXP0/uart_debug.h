//*****************************************************************************
//
// uart_debug.h - UART 调试输出模块接口
//
// 通过 UART0(PA0-RX / PA1-TX) @115200 8-N-1 输出调试信息�?
// 用于�? PC 终端实时观察三路 ADC 电压 / 原始值�?
//
//*****************************************************************************

#ifndef __UART_DEBUG_H__
#define __UART_DEBUG_H__

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

// 初始�? UART0�?115200, 8-N-1），使能 GPIOA，配�? PA0/PA1 �? UART 引脚
extern void UART_DebugInit(uint32_t ui32SysClock);

// 阻塞发送字符串（以 '\0' 结尾�?
extern void UART_DebugString(const char *s);

// 发送字符串并在末尾追加 "\r\n"
extern void UART_DebugLine(const char *s);

// 输出一行三�? ADC 数据：电�? + 原始�?
extern void UART_DebugADCDump(uint32_t raw0, uint32_t raw1, uint32_t raw2,
                              uint32_t mv0, uint32_t mv1, uint32_t mv2);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __UART_DEBUG_H__