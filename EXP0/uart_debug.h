//*****************************************************************************
//
// uart_debug.h - UART debug output module interface
//
// Outputs debug info over UART0 (PA0-RX / PA1-TX) @115200 8-N-1, for watching
// the three ADC voltages / raw values on a PC terminal in real time.
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

// Initialize UART0 (115200, 8-N-1), enable GPIOA, configure PA0/PA1 as UART pins
extern void UART_DebugInit(uint32_t ui32SysClock);

// Blocking send of a '\0'-terminated string
extern void UART_DebugString(const char *s);

// Send a string and append "\r\n"
extern void UART_DebugLine(const char *s);

// Output one line of three ADC channels: voltage + raw value + current
extern void UART_DebugADCDump(uint32_t raw0, uint32_t raw1, uint32_t raw2,
                              uint32_t mv0, uint32_t mv1, uint32_t mv2,
                              uint32_t ima);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __UART_DEBUG_H__