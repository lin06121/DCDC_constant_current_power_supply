//*****************************************************************************
//
// uart_debug.c - UART debug output module implementation
//
// Reference: E:\code\keil\EXP3\exp3-8.c UART pattern: UART0 on PA0(RX)/PA1(TX),
// UARTConfigSetExpClk configures 115200 8-N-1, blocking send via UARTCharPut.
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
// Function:    void UART_DebugInit(uint32_t ui32SysClock)
// Function:    enable UART0/GPIOA, configure PA0(RX)/PA1(TX), baud 115200 8-N-1
// Parameters:  ui32SysClock   system clock frequency
// Returns:     none
//
//*****************************************************************************
void UART_DebugInit(uint32_t ui32SysClock)
{
    // enable UART0 and GPIOA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){};

    // configure PA0(U0RX) / PA1(U0TX) as UART pins
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // configure baud 115200, 8-N-1
    UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
}

//*****************************************************************************
//
// Function:    void UART_DebugString(const char *s)
// Function:    blocking send of a '\0'-terminated string
// Parameters:  s   string to send
// Returns:     none
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
// Function:    void UART_DebugLine(const char *s)
// Function:    send a string and append "\r\n"
// Parameters:  s   string to send
// Returns:     none
//
//*****************************************************************************
void UART_DebugLine(const char *s)
{
    UART_DebugString(s);
    UART_DebugString("\r\n");
}

//*****************************************************************************
//
// Function:    void UART_DebugADCDump(...)
// Function:    output one line holding three ADC voltages (X.YYY V), their raw
//              values (0-4095) and the calibrated current (X.YYY A).
// Parameters:  raw0/raw1/raw2   three averaged raw values
//              mv0/mv1/mv2      three millivolt values
//              ima              actual current in mA
// Returns:     none
//
//*****************************************************************************
void UART_DebugADCDump(uint32_t raw0, uint32_t raw1, uint32_t raw2,
                       uint32_t mv0, uint32_t mv1, uint32_t mv2,
                       uint32_t ima)
{
    char buf[80];

    sprintf(buf, "CH0=%u.%03uV(%u)  CH1=%u.%03uV(%u)  CH2=%u.%03uV(%u)  I=%u.%03uA\r\n",
            mv0 / 1000, mv0 % 1000, raw0,
            mv1 / 1000, mv1 % 1000, raw1,
            mv2 / 1000, mv2 % 1000, raw2,
            ima / 1000, ima % 1000);

    UART_DebugString(buf);
}