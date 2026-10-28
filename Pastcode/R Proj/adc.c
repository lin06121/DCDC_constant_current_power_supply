
#include "adc.h"

// 窗口宽度
const uint32_t WIN_WID=30;
// 存放从ADC FIFO读取的采样数据 [0-4095]
uint32_t adc0_value[4];
// ADC采样结果，放入buffer便于取平均
uint32_t adc_val_bfr[4][WIN_WID];
// buffer循环队列使用的变量
uint8_t adc_bfr_ptr = 0;
// 用于表示adc队列成功完成一个循环，后续对整个队列取平均即可
bool adc_ready = false;
// 用于展示的数字
uint32_t adc_opt[4];
uint32_t prev_adc_opt[4];
// 阻滞门限法的误差
const double Adc_Err[4]={0,0,0,0};

extern void ShowError(void);
extern void ClearError(void);
extern void LightBlink(void);

// ADC0初始化
// 选择CHO/PE3、CH1/PE2、CH2/PE1、CH3/PE0引脚作为ADC0采样输入端口，选用采样序列产生器1
void ADCInit(void) {
    // 使能ADC0模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // 使能端口E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // 使用CH0/PE3和CH1/PE2引脚作为ADC输入
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);

    // 选用采样序列产生器1，采样起始信号由ADCProcessorTrigger函数触发，优先级为0
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

    // 设置采样序列产生器1第0步骤:选择CH0为采样通道
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2);

    // 设置采样序列产生器1第1步骤:
    // ADC_CTL_CH1--选择CH1为采样通道， ADC_CTL_END--采样序列的最后一步
    // ADC_CTL_IE--采样结束后产生中断
    ADCSequenceStepConfigure( ADC0_BASE, 1, 3,
                              ADC_CTL_CH3 | ADC_CTL_END | ADC_CTL_IE);

    // 使能采样序列产生器1
    ADCSequenceEnable(ADC0_BASE, 1);

    // 在采样前，必须清除中断状态标志
    ADCIntClear(ADC0_BASE, 1);
}

// 获取4个通道的ADC采样值
void ADC_Sample(void)
{
    // adc0_value数组用于从ADC FIFO读取的数据
    extern uint32_t adc0_value[4];

    // 触发ADC0采样序列发生器1开始工作
    ADCProcessorTrigger(ADC0_BASE, 1);

    // 等待ADC0采样序列发生器1采样转换完成
    while(!ADCIntStatus(ADC0_BASE, 1, false));

    // 清除ADC0中断标志
    ADCIntClear(ADC0_BASE, 1);

    // 读取ADC0采样值，存储到数组adc0_value中
    ADCSequenceDataGet(ADC0_BASE, 1, adc0_value);
}

// 使用滑动窗口取平均更新
void ADC_Update(void){

    int i = 0, j = 0;

    ADC_Sample();

    for (i = 0; i < 4; i++) {
        // buffer是一个循环队列，依次存储新的adc采样值，旧的覆盖新的，当前位置为adc_bfr_ptr
        adc_val_bfr[i][adc_bfr_ptr] = adc0_value[i];

        // 滑动窗口取平均，即获取最近WIN_WID个点的平均值
        adc_opt[i] = 0;
        for (j = 0; j < WIN_WID;j++) {
            adc_opt[i] += adc_val_bfr[i][j];
        }

        adc_opt[i] /= adc_ready ? WIN_WID : adc_bfr_ptr+1;
    }
    adc_opt[0] = adc_opt[0] * 6600/4095;    // 稳压源电压
    adc_opt[1] = 0.53045 * adc_opt[1] + 75; // 稳压源电流
    adc_opt[2] = adc_opt[2];    // 稳流源电压
    adc_opt[3] = adc_opt[3];                // 稳流源电流

    // 阻滞门限法
    for (i = 0; i < 4; i++) {
        if (adc_opt[i] - prev_adc_opt[i] < Adc_Err[i] && prev_adc_opt[i] - adc_opt[i] < Adc_Err[i]) {
            adc_opt[i] = prev_adc_opt[i];
        }
        prev_adc_opt[i] = adc_opt[i];
    }

    adc_bfr_ptr++;
    if(adc_bfr_ptr > WIN_WID-1) adc_bfr_ptr = 0, adc_ready = true;
}
