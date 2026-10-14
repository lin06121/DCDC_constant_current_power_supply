/* 工科创3A version:3.4
 *
 * ** 引脚定义 **
 * P1.0 Volt J2 稳压电压/稳流电压
 * P1.1 CC J1 稳流电流
 * P1.2 CV J1 稳压电流
 * P1.4 SDA J6 稳流
 * P1.5 SCL J7 稳流
 *
 * ** 按键定义 **
 * SW1 转到第一阶段
 * SW2 转到第二阶段，再按切换显示
 * SW3 转到第三阶段，再按切换显示
 * SW5 第二阶段适用，输出电流-10mA
 * SW6 第二阶段适用，输出电流-100mA
 * SW7 第二阶段适用，输出电流+100mA
 * SW8 第二阶段适用，输出电流+10mA
 *
 * ** 屏显定义 **
 * Stage1: LED8;数码管上左边显示输出电压（V），右边显示输出电流（A）
 * Stage2: LED7; Volt, CC; Shift: __, output_current.第一屏数码管右边显示为输出电流设点（A），此时通过按键SW5-SW8可以调整输出电流（精度10mA）
 * 第二屏显示实测输出电压电流，数码管上左边显示输出电压（V），右边显示输出电流。该模块有限压保护功能，输出电压最大约为5.2V
 * Stage3: LED6; __, Volt; Shift: CC, CV
 * 第一屏左边显示为并联输出电压。第二屏左边显示为稳流源输出电流，右边为稳压源输出电流。
 */

//本程序时钟采用内部RC 振荡器。DCO：8MHz,供CPU 时钟; SMCLK：1MHz,供定时器时钟
#include <msp430g2553.h>
#include <tm1638.h> //与TM1638 有关的变量及函数定义均在该H 文件中
#include <math.h>

//////////////////////////////
// 常量定义//
//////////////////////////////

#define V_T 50
// ADC10 参考电压及量化单位

#define n_sample 32
#define N_avg 200

// 0.1s 软件定时器溢出值，5 个20ms
#define V_T100ms 5
// 0.5s 软件定时器溢出值，25 个20ms
#define V_T500ms 25
// 软件定时器溢出值，N_ctrl 个20ms
#define V_T_ctrl 5
//DAC6571 操作P1.4 接SDA(pin4),P1.5 接SCL(pin5)
#define SCL_L P1OUT&=~BIT5
#define SCL_H P1OUT|=BIT5
#define SDA_L P1OUT&=~BIT4
#define SDA_H P1OUT|=BIT4
#define SDA_IN P1OUT|=BIT4; P1DIR&=~BIT4; P1REN|=BIT4
#define SDA_OUT P1DIR|=BIT4; P1REN&=~BIT4
#define output_current_max  4999
#define DAC6571_address 0x98 // 1001 10 A0 0 A0=0
//////////////////////////////
// 变量定义//
//////////////////////////////
// ADC10 变量定义
unsigned int sample[3]={0};//存放ADC 采样结果（一次转换产生的两个结果）
unsigned int sample_v[n_sample] = {0};
unsigned int sample_i1[n_sample] = {0};
unsigned int sample_i2[n_sample] = {0};
unsigned int i_sample = 0;
unsigned int iter_avg = 0;
long v_sum, i1_sum, i2_sum;

unsigned char stage = 3;  // first, second, third
unsigned char shift=0;  // 切换屏显
int Volt, CCurrent, VCurrent; // 电压， 稳流源电压，稳压源电压

// 软件定时器计数
unsigned char clock100ms=0;
unsigned char clock500ms=0;
unsigned char clock=0;
// 软件定时器溢出标志
unsigned char clock100ms_flag=0;
unsigned char clock500ms_flag=0;
unsigned char clock_flag=0;
// 测试用计数器
unsigned int test_counter=0;
// 8 位数码管显示的数字或字母符号
// 注：板上数码位从左到右序号排列为4、5、6、7、0、1、2、3
unsigned char digit[8]={' ',' ',' ',' ',' ',' ',' ',' '};
// 8 位小数点1 亮0 灭
// 注：板上数码位小数点从左到右序号排列为4、5、6、7、0、1、2、3
unsigned char pnt=0x11;
// 8 个LED 指示灯状态，每个灯4 种颜色状态，0 灭，1 绿，2 红，3 橙（红+绿）
// 注：板上指示灯从左到右序号排列为7、6、5、4、3、2、1、0
// 对应元件LED8、LED7、LED6、LED5、LED4、LED3、LED2、LED1
unsigned char led[]={0,0,0,0,0,0,0,1};
// 当前按键值
unsigned char key_code=0;
unsigned char key_cnt=0;

// DAC6571
unsigned int dac6571_code=0x0800;
unsigned int output_current=2500;
unsigned char dac6571_flag=0;
//////////////////////////////
// 系统初始化//
//////////////////////////////
// I/O 端口和引脚初始化
void Init_Ports(void)
{
    P2SEL &= ~(BIT7+BIT6); //P2.6、P2.7 设置为通用I/O 端口（GPIO）
    //因两者默认连接外晶振，故需此修改
    P2DIR |= BIT7 + BIT6 + BIT5; //P2.5、P2.6、P2.7 设置为输出
    //本电路板中三者用于连接显示和键盘管理器TM1638，工作原理详见其DATASHEET
    P1SEL |= (BIT0+BIT1+BIT2); //P1.0、P1.1 设置为通用I/O 端口（GPIO）
    P1DIR &=~ (BIT0+BIT1+BIT2); //P1.0、P1.1 设置为输入
    P1DIR |= BIT4 + BIT5; //P1.4、P1.5 设置为输出
    P1OUT |= BIT4 + BIT5; //P1.4、P1.5 设置为高电平
}
// 定时器TIMER0 初始化，循环定时20ms
void Init_Timer0(void)
{
    TA0CTL = TASSEL_2 + MC_1 ; // Source: SMCLK=1MHz, UP mode,
    TA0CCR0 = 20000; // 1MHz 时钟,计满20000 次为20ms
    TA0CCTL0 = CCIE; // TA0CCR0 interrupt enabled
}
// ADC10 初始化
void Init_ADC10()
{
    ADC10CTL1 = CONSEQ_1 + INCH_2; // 3 通道单次转换, 最大转换通道为A2
    ADC10CTL0 = ADC10SHT_2 + ADC10ON + ADC10IE + MSC;
    //采样保持时间为16 x ADC10CLKs，ADC 内核开，中断使能MSC 多次转换选择开
    //参考电压选默认值VCC 和VSS
    ADC10DTC1 = 0x03;
    ADC10AE0 |= BIT0+BIT1+BIT2;// 使能模拟输入脚A0 A1 A2
}
// MCU 器件初始化，注：会调用上述函数
void Init_Devices(void)
{
    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer，停用看门狗
    if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)
    {
        while(1); // If calibration constants erased, trap CPU!!
    }
    //设置时钟，内部RC 振荡器。DCO：8MHz,供CPU 时钟; SMCLK：1MHz,供定时器时钟
    BCSCTL1 = CALBC1_8MHZ; // Set range
    DCOCTL = CALDCO_8MHZ; // Set DCO step + modulation
    BCSCTL3 |= LFXT1S_2; // LFXT1 = VLO
    IFG1 &= ~OFIFG; // Clear OSCFault flag
    BCSCTL2 |= DIVS_3; // SMCLK = DCO/8
    Init_Ports(); //调用函数，初始化I/O 口
    Init_Timer0(); //调用函数，初始化定时器0
    Init_ADC10(); //初始化ADC
    _BIS_SR(GIE); //开全局中断
    //all peripherals are now initialized
}
void dac6571_byte_transmission(unsigned char byte_data)
{
    unsigned char i,shelter;
    shelter = 0x80;
    for (i=1; i<=8; i++)
    {
        if ((byte_data & shelter)==0) SDA_L;
        else SDA_H;
        SCL_H; SCL_L;
        shelter >>= 1;
    }
    SDA_IN;
    SCL_H; SCL_L;
    SDA_OUT;
}
void dac6571_fastmode_operation(void)
{
    unsigned char msbyte,lsbyte;
    SCL_H; SDA_H; SDA_L; SCL_L; // START condition
    dac6571_byte_transmission(DAC6571_address);
    msbyte = dac6571_code/256;
    lsbyte = dac6571_code - msbyte * 256;
    dac6571_byte_transmission(msbyte);
    dac6571_byte_transmission(lsbyte);
    SDA_L; SCL_H; SDA_H; // STOP condition
}
//////////////////////////////
// 中断服务程序//
//////////////////////////////
//20ms 软中断
// Timer0_A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void)
{
    // 0.1 秒钟软定时器计数
    if (++clock100ms>=V_T100ms)
    {
        clock100ms_flag = 1; //当0.1 秒到时，溢出标志置1
        clock100ms = 0;
    } // 0.5 秒钟软定时器计数
    if (++clock500ms>=V_T500ms)
    {
        clock500ms_flag = 1; //当0.5 秒到时，溢出标志置1
        clock500ms = 0;
    }
    if (++clock>=V_T_ctrl)
    {
        clock_flag = 1; //当0.1 秒到时，溢出标志置1
        clock = 0;
    }
    // 刷新全部数码管和LED 指示灯
    //TM1638_RefreshDIGIandLED(digit,pnt,led);
    // 检查当前键盘输入，0 代表无键操作，1-16 表示有对应按键
    // 键号显示在两位数码管上
    key_code=TM1638_Readkeyboard();
    if (key_code != 0)
    {
        if (key_cnt<4) key_cnt++;
        else if (key_cnt==4)
        {
            if (key_code==1)
            {
                stage = 1;
                shift = 0;
            }
            else if (key_code==2)
            {
                stage = 2;
                shift = !shift;
                output_current = (int)(output_current/10)*10;
            }
            else if (key_code==3)
            {
                stage = 3;
                shift = !shift;
            }
            else if(key_code>=5 && key_code<=8)
            {
                if(stage==2)
                {
                    if (key_code==5)
                    {
                        if (output_current > 10)
                        {
                            output_current-=10;
                            dac6571_flag =1;
                        }
                    }
                    else if (key_code==6)
                    {
                        if (output_current > 100)
                        {
                            output_current-=100;
                            dac6571_flag =1;
                        }
                    }
                    else if (key_code==7)
                    {
                        if (output_current < output_current_max-100)
                        {
                            output_current+=100;
                            dac6571_flag =1;
                        }
                    }
                    else if (key_code==8)
                    {
                        if (output_current < output_current_max-10)
                        {
                            output_current+=10;
                            dac6571_flag =1;
                        }
                    }

                }
            }
            key_cnt =5;
        }
    } else key_cnt=0;

}
////ADC10 中断服务程序
#pragma vector=ADC10_VECTOR
__interrupt void ADC10ISR (void)
{
    sample_v[i_sample] = sample[2];
    sample_i1[i_sample] = sample[1];
    sample_i2[i_sample] = sample[0];
}
//////////////////////////////
// 主程序//
//////////////////////////////
int main(void)
{
    float temp;
    Init_Devices( );
    while (clock100ms<3); // 延时60ms 等待TM1638 上电完成
    init_TM1638(); //初始化TM1638
    dac6571_flag = 1;
    while(1)
    {
        //ADC10 转换
        ADC10CTL0 |= ENC + ADC10SC;
        while(ADC10CTL1 & BUSY);//等待ADC10 转换完成
        ADC10SA = (unsigned int)sample;
        ADC10CTL0 &= ~ENC;
        ++i_sample;
        //当采满n_sample 个样本后
        if(i_sample == n_sample)
        {
            i_sample = 0;

            unsigned int k = 0;
            for( k = 0 ; k < n_sample; ++k)
            {
                v_sum += sample_v[k];
                i1_sum += sample_i1[k];
                i2_sum += sample_i2[k];
            }

            ++iter_avg;
            if (iter_avg == N_avg)
            {
                iter_avg = 0;

                //Volt = 6.9509*v_sum/n_sample/N_avg+3.652;
                //CCurrent = 3.5303*i1_sum/n_sample/N_avg+8.9375;
                //VCurrent = 3.4792*i2_sum/n_sample/N_avg-40.352;
                //这是初始的经验公式，后续根据定标所得参数进行了一定修改

                Volt = 6.9919*v_sum/n_sample/N_avg+3.6753;
                CCurrent = 3.5065*i1_sum/n_sample/N_avg+8.9946;
                VCurrent = 3.4858*i2_sum/n_sample/N_avg-40.514;

                v_sum = 0;
                i1_sum = 0;
                i2_sum = 0;

                if (stage==1)
                {
                    digit[4] = (Volt/1000)%10;
                    digit[5] = (Volt/100)%10;
                    digit[6] = (Volt/10)%10;
                    digit[7] = (Volt)%10;

                    digit[0] = (VCurrent/1000)%10;
                    digit[1] = (VCurrent/100)%10;
                    digit[2] = (VCurrent/10)%10;
                    digit[3] = (VCurrent)%10;
                }

                else if (stage==2 && shift==0)
                {
                    digit[4] = ' ';
                    digit[5] = ' ';
                    digit[6] = ' ';
                    digit[7] = ' ';

                    digit[0] = (output_current/1000)%10;
                    digit[1] = (output_current/100)%10;
                    digit[2] = (output_current/10)%10;
                    digit[3] = (output_current)%10;
                }

                else if (stage==2 && shift==1)
                {
                    digit[4] = (Volt/1000)%10;
                    digit[5] = (Volt/100)%10;
                    digit[6] = (Volt/10)%10;
                    digit[7] = (Volt)%10;

                    digit[0] = (CCurrent/1000)%10;
                    digit[1] = (CCurrent/100)%10;
                    digit[2] = (CCurrent/10)%10;
                    digit[3] = (CCurrent)%10;
                }

                else if (stage==3 && shift==0)
                {
                    digit[4] = (CCurrent/1000)%10;
                    digit[5] = (CCurrent/100)%10;
                    digit[6] = (CCurrent/10)%10;
                    digit[7] = (CCurrent)%10;

                    digit[0] = (VCurrent/1000)%10;
                    digit[1] = (VCurrent/100)%10;
                    digit[2] = (VCurrent/10)%10;
                    digit[3] = (VCurrent)%10;
                }
                else if (stage==3 && shift==1)
                {
                    digit[4] = (Volt/1000)%10;
                    digit[5] = (Volt/100)%10;
                    digit[6] = (Volt/10)%10;
                    digit[7] = (Volt)%10;

                    digit[0] = ' ';
                    digit[1] = ' ';
                    digit[2] = ' ';
                    digit[3] = ' ';
                }

            }
        }
        if (clock_flag==1)  //100ms
        {
            clock_flag=0;
            if(stage==2 && dac6571_flag==1)
            {
                dac6571_flag=0;
                temp = (1.5*output_current-127.83)*4096.0/(output_current_max+1);
                dac6571_code = temp;
                dac6571_fastmode_operation();
            }

            if(stage==3)
            {
                int tmp_int = (CCurrent+VCurrent)/2+2;
                if (abs(output_current - tmp_int)>4)
                    output_current = (tmp_int + output_current)/2;
                else
                    output_current = output_current;
                temp = (1.5*output_current-127.83)*4096.0/(output_current_max+1);
                dac6571_code = temp;
                dac6571_fastmode_operation();
            }

        }
        if (clock500ms_flag==1) // 检查0.5 秒定时是否到
        {
            clock500ms_flag=0;
            // 8 个指示灯以走马灯方式，每0.5 秒向右（循环）移动一格
            int k=0;
            for (k=7;k>=0;k--) led[k]=0;
            led[8-stage]=1;

            // 不懂为啥，不能注释
            TM1638_RefreshDIGIandLED(digit,pnt,led);
        }
    }
}
