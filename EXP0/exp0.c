//*************************************************************************************
//
// Copyright: 2020-2021, Shanghai Jiao Tong University, Dept. of Electronic Engineering
// File name: exp0.c
// Description:
//    1. Three-channel ADC sampling on CH0/PE3, CH1/PE2 and CH2/PE1.
//    2. Left 4 digits show current setpoint or output voltage; SW2 changes page.
//    3. Right 4 digits show the output current in A [0.000-9.999 A].
//    4. Timer0A (1 kHz) triggered sampling + 16x hardware oversampling + sliding-window average.
//    5. Input voltage must stay within [0-3.3 V], otherwise the pin may be damaged.
//    6. SW1: +0.1 A; SW4: -0.1 A; range 0.1-1.0 A, power-on default 0.8 A.
//    7. LED8: setpoint page; LED7: output-voltage page. Right digits always show current.
//    8. DAC6571.c/.h drive PL0-SDA and PL1-SCL. Analog hardware performs regulation.
// Author:	Shanghai Jiao Tong University, Dept. of Electronic Engineering
// Version: 1.0.0.20201125
// Date:	2020-11-25
// History:
//		2026-10-16: extended to three channels; timer-triggered sampling + hardware
//		            oversampling + sliding average; shows voltage (mV).
//
//*************************************************************************************

//*************************************************************************************
//
// Header files
//
//*************************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"         // interrupt vector numbers (INT_ADC0SS1, ...)
#include "inc/hw_memmap.h"        // base address macros
#include "inc/hw_types.h"         // data types and register access helpers
#include "driverlib/debug.h"      // debug helpers
#include "driverlib/gpio.h"       // GPIO API
#include "driverlib/pin_map.h"    // TM4C peripheral pin-mux macro definitions
#include "driverlib/sysctl.h"	  // system control / clocking
#include "driverlib/systick.h"    // SysTick driver
#include "driverlib/interrupt.h"  // NVIC interrupt controller driver
#include "driverlib/adc.h"        // ADC API
#include "driverlib/timer.h"      // Timer driver (timer-triggered ADC)
#include "tm1638.h"               // TM1638 display driver
#include "uart_debug.h"           // UART debug output module
#include "DAC6571.h"              // course DAC6571 driver

//*************************************************************************************
//
// Macro definitions
//
//*************************************************************************************
#define SYSTICK_FREQUENCY	50		// SysTick rate = 50 Hz -> 20 ms loop period

#define V_T100ms 5                      // 0.1 s software timer overflow: 5 * 20 ms (UI refresh @10 Hz)

// ADC sampling and filtering parameters
#define ADC_NUM_CH        3             // number of ADC channels: CH0/PE3, CH1/PE2, CH2/PE1
#define ADC_TRIGGER_HZ    1000          // Timer0A ADC trigger rate: 1 kHz
#define ADC_OVS_FACTOR    16            // hardware oversampling 16x (16 kHz raw rate)
#define ADC_WINDOW_SIZE   16            // software sliding-window size: 16 samples (~16 ms)
#define ADC_CONST_MV      3300          // full scale 3.300 V, in mV

// ---- Current-channel calibration (0.1 ohm sense resistor, PE2/CH1) ----
// Least-squares fit against multimeter measurements:
//     I(mA) = 0.665761 * V_adc(mV) + 28.453,  where V_adc(mV) = RawToMilliVolts(raw)
// The slope is scaled by 65536 for fixed-point math (avoiding floating point); the
// resulting offset error is below the 1 mA display resolution.
#define CAL_I_MA_PER_MVQ   43632u       // 0.665761 * 65536 ~= 43632 (mA per ADC-mV)
#define CAL_I_MA_OFFSET    28u          // intercept b = 28.453 mA, rounded

// Stage-2 setpoint and keypad parameters
#define CURRENT_MIN_MA       100
#define CURRENT_MAX_MA       1000
#define CURRENT_STEP_MA      100
#define CURRENT_DEFAULT_MA   800

// Provisional board values: the .epro has empty values / generic library defaults.
// Replace with actual resistors or calibrated ratios before testing output accuracy.
// Voltage divider: Vout = Vadc * (R18 + R19) / R19; assume equal resistors for now.
#define VOLTAGE_RATIO_NUM    2
#define VOLTAGE_RATIO_DEN    1
#define DAC_SUPPLY_MV        3300
// U5.1: Vfb=(1+R21/R20)*R28/(R22+R28)*ISEN-(R21/R20)*Vdac.
// Example matched gain of 10; actual values are not yet confirmed.
#define CC_R20_OHM           10000
#define CC_R21_OHM           100000
#define CC_R22_OHM           10000
#define CC_R28_OHM           100000
#define TPS_FB_MV            700

//*************************************************************************************
//
// Function prototypes
//
//*************************************************************************************
void GPIOInit(void);        // GPIO initialization
void ADCInit(void);         // ADC initialization
void TimerADCInit(void);    // Timer0A timer-triggered ADC initialization
void SysTickInit(void);     // SysTick interrupt setup
void DevicesInit(void);     // MCU peripheral init; calls the functions above
uint32_t RawToMilliVolts(uint32_t raw);     // convert ADC raw value to millivolts
uint32_t CurrentToMilliAmps(uint32_t raw);  // convert CH1/PE2 raw value to actual current (mA)
uint32_t CurrentToDACCode(uint32_t current_ma);
void ScanKey(void);

//*************************************************************************************
//
// Variables
//
//*************************************************************************************

// Software timer counter (updated in SysTick ISR, read in the main loop -> volatile)
volatile uint8_t clock100ms = 0;

// Software timer overflow flag (updated in SysTick ISR, read in the main loop -> volatile)
volatile uint8_t clock100ms_flag = 0;

// 8-digit display characters
// NOTE: physical digit order on the board is 4,5,6,7,0,1,2,3 left to right
uint8_t digit[8]={' ',' ',' ',' ',' ',' ',' ',' '};

// 8 decimal points; 1 = on, 0 = off
// NOTE: decimal-point order on the board is 4,5,6,7,0,1,2,3 left to right
uint8_t pnt = 0x0;

// 8 LED indicator states; 0 = off, 1 = on
// NOTE: on-board LED order is 7,6,5,4,3,2,1,0 left to right (LED8..LED1)
uint8_t led[] = {0, 0, 0, 0, 0, 0, 0, 0};

// Stage-2 UI state; ScanKey() updates it in the SysTick ISR.
volatile uint16_t current_set_ma = CURRENT_DEFAULT_MA;
volatile uint8_t dac_update_flag = 0;
volatile uint8_t display_page = 0;       // 0: Iset/Iout; 1: Vout/Iout

// Keypad scan state, following the course dac_demo.c.
uint8_t key_code = 0;
uint8_t key_cnt = 0;

// System clock frequency
uint32_t ui32SysClock;

// ADC sliding-window filter state (updated in the ADC ISR, read in the main loop -> volatile)
volatile uint32_t g_ui32ADCWin[ADC_NUM_CH][ADC_WINDOW_SIZE]; // per-channel ring buffer
volatile uint32_t g_ui32ADCWinSum[ADC_NUM_CH];               // running sum (O(1) update)
volatile uint8_t  g_ui8ADCWinIdx = 0;                        // ring buffer write index
volatile uint8_t  g_ui8ADCWinCnt = 0;                        // number of filled samples
volatile uint32_t g_ui32ADCAvg[ADC_NUM_CH];                  // latest per-channel average (0-4095)

//*************************************************************************************
//
// Main program
//
//*************************************************************************************
int main(void)
{
	DevicesInit();            // MCU peripheral initialization

	while (clock100ms < 3);   // wait >60 ms for TM1638 power-up to complete
	TM1638_Init();	          // initialize TM1638
    DAC6571_Fastmode_Operation(CurrentToDACCode(CURRENT_DEFAULT_MA));

	while (1)
	{
        if (clock100ms_flag == 1)       // 100 ms tick reached (UI refresh @10 Hz)
        {
            uint32_t mv0, mv1, mv2, ima, left_value;

            clock100ms_flag = 0;

            if (dac_update_flag == 1)
            {
                // Clear first: a press during conversion/transmission remains pending.
                dac_update_flag = 0;
                DAC6571_Fastmode_Operation(CurrentToDACCode(current_set_ma));
            }

            // read the averaged raw values and convert them to millivolts
            mv0 = RawToMilliVolts(g_ui32ADCAvg[0]);   // CH0/PE3 (output voltage)
            mv1 = RawToMilliVolts(g_ui32ADCAvg[1]);   // CH1/PE2 (current-sense voltage)
            mv2 = RawToMilliVolts(g_ui32ADCAvg[2]);   // CH2/PE1 (reserved; debug / phase 3)
            ima = CurrentToMilliAmps(g_ui32ADCAvg[1]); // CH1/PE2 actual current (mA)

            // Left page: current setpoint (A) or divider-corrected output voltage (V).
            left_value = display_page == 0 ? current_set_ma : mv0 * VOLTAGE_RATIO_NUM / VOLTAGE_RATIO_DEN;
            digit[4] = left_value / 1000;
            digit[5] = left_value / 100 % 10;
            digit[6] = left_value / 10 % 10;
            digit[7] = left_value % 10;
            led[7] = (display_page == 0);       // LED8: current setpoint
            led[6] = (display_page == 1);       // LED7: output voltage

            // right 4 digits show the output current in X.YYY A format (ima is in mA)
            digit[0] = ima / 1000; 	  // integer digit (A)
            digit[1] = ima / 100 % 10;  // 0.1 A
            digit[2] = ima / 10 % 10;   // 0.01 A
            digit[3] = ima % 10;        // 0.001 A

            // light the first decimal point of each group -> display "X.YYY"
            pnt = (1 << 4) | (1 << 0);

            // UART debug output: three voltages + raw values, @10 Hz
            UART_DebugADCDump(g_ui32ADCAvg[0], g_ui32ADCAvg[1], g_ui32ADCAvg[2],
                              mv0, mv1, mv2, ima);
        }
	}
}

//*************************************************************************************
//
// Function:	void GPIOInit(void)
// Function:	GPIO initialization. Enable Port K and set PK4/PK5 as outputs; enable
//              Port M and set PM0 as output.
//              (PK4 = TM1638 STB, PK5 = TM1638 DIO, PM0 = TM1638 CLK)
// Parameters:	none
// Returns:		none
//
//*************************************************************************************
void GPIOInit(void)
{
	// configure the TM1638 chip pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);				// enable Port K
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){};		// wait for Port K ready

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);				// enable Port M
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){};		// wait for Port M ready

   // set Port K pins 4,5 (PK4,PK5) as outputs		PK4-STB  PK5-DIO
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4|GPIO_PIN_5);
	// set Port M pin 0 (PM0) as output   PM0-CLK
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);

    // GPIO I2C used by the supplied DAC6571.c: PL0-SDA, PL1-SCL.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){};
    GPIOPinTypeGPIOOutput(SDA_PIN_BASE, SDA_PIN);
    GPIOPinTypeGPIOOutput(SCL_PIN_BASE, SCL_PIN);
    GPIOPinWrite(SDA_PIN_BASE, SDA_PIN, SDA_PIN);
    GPIOPinWrite(SCL_PIN_BASE, SCL_PIN, SCL_PIN);
}

//*************************************************************************************
//
// Function:	void ADCInit(void)
// Function:	ADC0 initialization. Select CH0/PE3, CH1/PE2 and CH2/PE1 as ADC0 inputs;
//              use sample sequencer 1, trigger it from Timer0A and enable its interrupt.
// Parameters:	none
// Returns:		none
//
//*************************************************************************************
void ADCInit(void)
{
    // enable the ADC0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // enable Port E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // use CH0/PE3, CH1/PE2 and CH2/PE1 pins as the ADC inputs
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1);

    // use sample sequencer 1, timer-triggered, priority 0
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);

    // enable hardware oversampling (16 samples automatically averaged); combined with
    // the hardware trigger this forms a low-pass filter for the fast switching ripple
    ADCHardwareOversampleConfigure(ADC0_BASE, ADC_OVS_FACTOR);

    // sample sequencer 1, step 0: sample CH0
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);

    // sample sequencer 1, step 1: sample CH1
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);

    // sample sequencer 1, step 2:
    // ADC_CTL_CH2 -- sample CH2; ADC_CTL_END -- last step of the sequence;
    // ADC_CTL_IE -- generate an interrupt when the sequence completes
    ADCSequenceStepConfigure( ADC0_BASE, 1, 2,
                              ADC_CTL_CH2 | ADC_CTL_END | ADC_CTL_IE);

    // enable sample sequencer 1
    ADCSequenceEnable(ADC0_BASE, 1);

    // enable the ADC0 sequencer-1 interrupt and its NVIC source
    ADCIntEnable(ADC0_BASE, 1);
    IntEnable(INT_ADC0SS1);

    // clear the interrupt status flag before sampling
    ADCIntClear(ADC0_BASE, 1);
}

//*************************************************************************************
//
// Function:	void TimerADCInit(void)
// Function:	Configure Timer0A as a 1 kHz periodic timer whose output triggers ADC0
//              sample sequencer 1 asynchronously (no CPU involvement).
// Parameters:	none
// Returns:		none
//
//*************************************************************************************
void TimerADCInit(void)
{
    // enable Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){};

    // configure Timer0A as a split, periodic, down-counting timer
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

    // set the reload value to produce ADC_TRIGGER_HZ (1 kHz)
    TimerLoadSet(TIMER0_BASE, TIMER_A, (ui32SysClock / ADC_TRIGGER_HZ) - 1);

    // enable the Timer0A ADC trigger output
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    // start Timer0A; it now continuously triggers ADC sampling
    TimerEnable(TIMER0_BASE, TIMER_A);
}

//*************************************************************************************
//
// Function:	uint32_t RawToMilliVolts(uint32_t raw)
// Function:	Convert an ADC raw value [0-4095] to millivolts [0-3300].
//              Phase 2/3 calibration can insert a k*x+b linear correction here.
// Parameters:	raw   ADC raw sample value
// Returns:		millivolt value
//
//*************************************************************************************
uint32_t RawToMilliVolts(uint32_t raw)
{
    // mv = raw * 3300 / 4096; add 2048 for rounding; >>12 replaces division by 4096
    return (raw * ADC_CONST_MV + 2048) >> 12;
}

//*************************************************************************************
//
// Function:	uint32_t CurrentToMilliAmps(uint32_t raw)
// Function:	Convert the CH1/PE2 ADC raw value to the actual output current (mA).
//              The voltage dropped across the 0.1 ohm sense resistor is read on PE2
//              after front-end amplification; the measured fit is
//              I(mA) = 0.665761 * V_adc(mV) + 28.453, implemented in fixed point.
// Parameters:	raw   CH1 raw sample value
// Returns:		actual current in mA
//
//*************************************************************************************
uint32_t CurrentToMilliAmps(uint32_t raw)
{
    uint32_t mv = RawToMilliVolts(raw);   // PE2 pin voltage (mV)
    return ((CAL_I_MA_PER_MVQ * mv + 32768) >> 16) + CAL_I_MA_OFFSET;
}

//*************************************************************************************
//
// Function:	void ADC0SS1_Handler(void)
// Function:	ADC0 sample-sequencer-1 ISR (1 kHz); reads the three samples and
//              performs the sliding-window average.
// Parameters:	none
// Returns:		none
//
//*************************************************************************************
void ADC0SS1_Handler(void)
{
    uint32_t data[8];                 // FIFO depth 8, enough for all pending samples
    int32_t  n, i, base;

    // clear the ADC0 sequencer-1 interrupt flag
    ADCIntClear(ADC0_BASE, 1);

    // read all pending FIFO samples (one trigger produces ADC_NUM_CH samples; drain extras)
    n = ADCSequenceDataGet(ADC0_BASE, 1, data);
    if (n < ADC_NUM_CH)               // not a complete sequence yet -> skip
        return;

    // take the most recent complete three-channel sequence (data[base .. base+2] = CH0/CH1/CH2)
    base = n - ADC_NUM_CH;

    // update each channel's ring buffer and running sum
    for (i = 0; i < ADC_NUM_CH; i++)
    {
        // subtract the value about to be overwritten, then add the new one (O(1) sliding average)
        g_ui32ADCWinSum[i] -= g_ui32ADCWin[i][g_ui8ADCWinIdx];
        g_ui32ADCWin[i][g_ui8ADCWinIdx] = data[base + i];
        g_ui32ADCWinSum[i] += data[base + i];

        // average over the samples actually filled until the window is full
        if (g_ui8ADCWinCnt < ADC_WINDOW_SIZE)
            g_ui32ADCAvg[i] = g_ui32ADCWinSum[i] / (g_ui8ADCWinCnt + 1);
        else
            g_ui32ADCAvg[i] = g_ui32ADCWinSum[i] / ADC_WINDOW_SIZE;
    }

    // advance the write pointer and accumulate the filled-sample count
    g_ui8ADCWinIdx = (g_ui8ADCWinIdx + 1) % ADC_WINDOW_SIZE;
    if (g_ui8ADCWinCnt < ADC_WINDOW_SIZE)
        g_ui8ADCWinCnt++;
}

//*************************************************************************************
//
// Function:	void SysTickInit(void)
// Function:	Configure the SysTick interrupt.
// Parameters:	none
// Returns:		none
//
//*************************************************************************************
void SysTickInit(void)
{
    SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // set the tick period (20 ms)
    SysTickEnable();  			// enable SysTick
    SysTickIntEnable();			// enable the SysTick interrupt
}

//*************************************************************************************
//
// Function:	void DevicesInit(void)
// Function:	MCU peripheral initialization: system clock, GPIO, ADC, timer-triggered
//              ADC and SysTick interrupt setup.
// Parameters:	none
// Returns:		none
//
//*************************************************************************************
void DevicesInit(void)
{
	// use the external 25 MHz crystal through the PLL, divided to 20 MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |
	                                   SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480),
	                                   20000000);

  GPIOInit();             // GPIO initialization
  ADCInit();              // ADC initialization
  TimerADCInit();         // Timer0A timer-triggered ADC initialization
  UART_DebugInit(ui32SysClock);   // UART debug output initialization
  SysTickInit();          // SysTick interrupt setup
  IntMasterEnable();	  // enable global interrupts
}

//*************************************************************************************
//
// Function:	void SysTick_Handler(void)
// Function:	SysTick interrupt service routine.
// Parameters:	none
// Returns:		none
//
//*************************************************************************************
void SysTick_Handler(void)       // tick period = 20 ms
{
	// 0.1 s software timer (10 Hz), used for UI refresh / later phase-3 current sharing
	if (++clock100ms >= V_T100ms)
	{
		clock100ms_flag = 1; // when 0.1 s elapses, set the overflow flag
		clock100ms = 0;
	}

    // Keep keypad reads and display writes together on the TM1638 bus.

    ScanKey();
    TM1638_RefreshDIGIandLED(digit, pnt, led);

}

// Convert a current setpoint to the 10-bit DAC code using the U5.1 circuit model.
// This reuses the stage-1 ISEN calibration; recalibrate it for the actual stage-2 board.
uint32_t CurrentToDACCode(uint32_t current_ma)
{
    int64_t isen_uv, dac_uv, code;

    isen_uv = ((int64_t)current_ma - CAL_I_MA_OFFSET) * 65536 * 1000 / CAL_I_MA_PER_MVQ;
    dac_uv = (isen_uv * (CC_R20_OHM + CC_R21_OHM) * CC_R28_OHM /
              (CC_R22_OHM + CC_R28_OHM) -
              (int64_t)TPS_FB_MV * 1000 * CC_R20_OHM) / CC_R21_OHM;
    code = (dac_uv * DAC6571_code_max + DAC_SUPPLY_MV * 500) / (DAC_SUPPLY_MV * 1000);

    // The supplied driver's code_max is the number of levels (1024).
    // Its valid input codes are 0..1023; keep the computed value in that range.
    if (code < 0) code = 0;
    if (code >= DAC6571_code_max) code = DAC6571_code_max - 1;
    return (uint32_t)code;
}

// Called every 20 ms; use the key_code/key_cnt pattern from dac_demo.c.
void ScanKey(void)
{
    key_code = TM1638_Readkeyboard();

    if (key_code != 0)
    {
        if (key_cnt < 4) key_cnt++;     // debounce: four 20 ms intervals
        else if (key_cnt == 4)
        {
            switch (key_code)
            {
                case 1:
                    if (current_set_ma < CURRENT_MAX_MA)
                    {
                        current_set_ma += CURRENT_STEP_MA;
                        dac_update_flag = 1;
                        display_page = 0;
                    }
                    break;
                case 4:
                    if (current_set_ma > CURRENT_MIN_MA)
                    {
                        current_set_ma -= CURRENT_STEP_MA;
                        dac_update_flag = 1;
                        display_page = 0;
                    }
                    break;
                case 2:
                    display_page = !display_page;
                    break;
                default:
                    break;
            }
            key_cnt = 5;              // a held key acts only once
        }
    }
    else key_cnt = 0;                  // release permits the next press
}
