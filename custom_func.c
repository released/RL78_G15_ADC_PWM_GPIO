/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>

#include "inc_main.h"

#include "misc_config.h"
#include "custom_func.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_TIMER_PERIOD_LEDs                     (flag_PROJ_CTL.bit1)
#define FLAG_PROJ_TIMER_PERIOD_ADC                      (flag_PROJ_CTL.bit2)
#define FLAG_PROJ_TRIG_BTN1       		                (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_TRIG_BTN2                             (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_TRIG_ADC_CH                           (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_PWM_DUTY_INC                          (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_PWM_DUTY_DEC                          (flag_PROJ_CTL.bit7)

#define FLAG_PROJ_TRIG_1                                (flag_PROJ_CTL.bit8)
#define FLAG_PROJ_TRIG_2                                (flag_PROJ_CTL.bit9)
#define FLAG_PROJ_TRIG_3                                (flag_PROJ_CTL.bit10)
#define FLAG_PROJ_TRIG_4                                (flag_PROJ_CTL.bit11)
#define FLAG_PROJ_TRIG_5                                (flag_PROJ_CTL.bit12)
#define FLAG_PROJ_REVERSE13                             (flag_PROJ_CTL.bit13)
#define FLAG_PROJ_REVERSE14                             (flag_PROJ_CTL.bit14)
#define FLAG_PROJ_REVERSE15                             (flag_PROJ_CTL.bit15)

/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned long counter_tick = 0;
volatile unsigned long btn_counter_tick = 0;

#define BTN_PRESSED_LONG                                (2500)

#define cLED1                                           (P2_bit.no2)
#define cLED2                                           (P2_bit.no3)
#define cLED3                                           (P0_bit.no7)
#define cLED4                                           (P0_bit.no6)
#define cLED5                                           (P0_bit.no5)
#define cLED6                                           (P0_bit.no1)
#define cLED7                                           (P0_bit.no0)
#define cLED8                                           (P4_bit.no1)

#define ENABLE_GPIO_REVERSE

#define DUTY_RESOLUTION                                 (100)
volatile unsigned long pwm_duty[8] = {0};

// #define VBG_VOLTAGE                                     (0.815)  // v
#define VBG_VOLTAGE                                     (815)  // mv

/*G15 V_BGR : 0.815 V*/
unsigned long vdd_Vbgr = 0;
unsigned short adc_buffer[11] = {0};

#define ADC_SAMPLE_COUNT                                (8)
#define ADC_SAMPLE_POWER	 				            (3)
#define ADC_SAMPLE_DROP 						        (4ul)
#define ADCTotalLength                                  (ADC_SAMPLE_COUNT+ADC_SAMPLE_DROP)   // drop first 4 ADC result

volatile signed short g_iConversionData[ADCTotalLength] = {0};
volatile unsigned short ADC_ch_value = 0;

// #define ENABLE_LOG_ADC
// #define ENABLE_LOG_PWM
// #define ENABLE_LOG_ADC_CVT_PWM
/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned long btn_get_tick(void)
{
	return (btn_counter_tick);
}

void btn_set_tick(unsigned long t)
{
	btn_counter_tick = t;
}

void btn_tick_counter(void)
{
	btn_counter_tick++;
    if (btn_get_tick() >= 60000)
    {
        btn_set_tick(0);
    }
}

unsigned long get_tick(void)
{
	return (counter_tick);
}

void set_tick(unsigned long t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

// void delay_ms(unsigned long ms)
// {
// 	#if 1
//     unsigned long tickstart = get_tick();
//     unsigned long wait = ms;
// 	unsigned long tmp = 0;
	
//     while (1)
//     {
// 		if (get_tick() > tickstart)	// tickstart = 59000 , tick_counter = 60000
// 		{
// 			tmp = get_tick() - tickstart;
// 		}
// 		else // tickstart = 59000 , tick_counter = 2048
// 		{
// 			tmp = 60000 -  tickstart + get_tick();
// 		}		
		
// 		if (tmp > wait)
// 			break;
//     }
	
// 	#else
// 	TIMER_Delay(TIMER0, 1000*ms);
// 	#endif
// }


void Sort_tab(unsigned short tab[], unsigned char length)
{
	unsigned char l = 0x00, exchange = 0x01; 
	unsigned short tmp = 0x00;

	/* Sort tab */
	while(exchange==1) 
	{ 
		exchange=0; 
		for(l=0; l<length-1; l++) 
		{
			if( tab[l] > tab[l+1] ) 
			{ 
				tmp = tab[l]; 
				tab[l] = tab[l+1]; 
				tab[l+1] = tmp; 
				exchange=1; 
			}
		}
	} 
}

unsigned short GetADC(e_ad_channel_t ch)
{
    unsigned short tmp_buffer = 0;

    #if 1   // execute average process
    unsigned short tmp = 0;
  	volatile unsigned long sum = 0;

    for ( tmp = 0 ; tmp < (ADCTotalLength) ; tmp++)
    {
        FLAG_PROJ_TRIG_ADC_CH = 0;
        R_Config_ADC_Start();
        while(!FLAG_PROJ_TRIG_ADC_CH);
        R_Config_ADC_Get_Result_10bit(&tmp_buffer);
        R_Config_ADC_Stop();
        FLAG_PROJ_TRIG_ADC_CH = 0;

        adc_buffer[ch] = tmp_buffer;    
        g_iConversionData[tmp] = tmp_buffer;
    }

    Sort_tab( (unsigned short*) g_iConversionData,ADCTotalLength);
    for (tmp = ADC_SAMPLE_DROP/2; tmp < ADCTotalLength - ADC_SAMPLE_DROP/2; tmp++)
    {
        sum += g_iConversionData[tmp];
    }
    sum = sum >> ADC_SAMPLE_POWER;
    tmp_buffer = (unsigned short) sum;

    #else
    // R_Config_ADC_Set_ADChannel(ch);
    FLAG_PROJ_TRIG_ADC_CH = 0;
    R_Config_ADC_Start();
    while(!FLAG_PROJ_TRIG_ADC_CH);
    R_Config_ADC_Get_Result_10bit(&tmp_buffer);
    R_Config_ADC_Stop();
    FLAG_PROJ_TRIG_ADC_CH = 0;

    adc_buffer[ch] = tmp_buffer;    
    #endif

    // printf_tiny("ch[%d]:0x%04X\r\n",ch,adc_buffer[ch]);

    return tmp_buffer;
}

/*
    COPY FROM R_Config_ADC_Create
    channel : 1
    P02
*/
void ADC_Channel_config_Init(void)
{
    volatile unsigned short w_count;

    ADCEN = 1U;    /* supply AD clock */
    ADMK = 1U;    /* disable INTAD interrupt */
    ADIF = 0U;    /* clear INTAD interrupt flag */
    /* Set INTAD priority */
    ADPR1 = 1U;
    ADPR0 = 1U;
    /* Set ANI1 pin */
    PMC0 |= 0x04U;
    PM0 |= 0x04U;
    ADM0 = _00_AD_CONVERSION_CLOCK_8 | _00_AD_TIME_MODE_NORMAL_1;
    ADM2 = _00_AD_RESOLUTION_10BIT;
    ADS = _01_AD_INPUT_CHANNEL_1;

    ADCE = 1U;

    /* Reference voltage stability wait time, 0.125us */
    for (w_count = 0U; w_count < AD_WAITTIME_B; w_count++ )
    {
        NOP();
    }
    
    // R_Config_ADC_Start();
}

void GetVREF(void)
{
    unsigned short tmp_buffer = 0;
    unsigned short adc_resolution = 1024;

    /*
        VDD = VBG * adc_resolution / ConversionResult
        VDD/VBG = adc_resolution / ConversionResult
    
    */
    // R_Config_ADC_Set_ADChannel(ADINTERREFVOLT);
    FLAG_PROJ_TRIG_ADC_CH = 0;
    R_Config_ADC_Start();    // to get ADC internal vref channel
    while(!FLAG_PROJ_TRIG_ADC_CH);
    R_Config_ADC_Get_Result_10bit(&tmp_buffer);
    R_Config_ADC_Stop();
    FLAG_PROJ_TRIG_ADC_CH = 0;

    vdd_Vbgr = (unsigned long) VBG_VOLTAGE*adc_resolution/tmp_buffer;

    // printf_tiny("Read VREF:%d(0x%04X),",tmp_buffer,tmp_buffer);
    // printf_tiny("VBGR:%4dmv\r\n",vdd_Vbgr);
}

/*
    COPY FROM R_Config_ADC_Create
    channel : internal reference voltage
*/
void ADC_VREF_config_Init(void)
{    
    volatile unsigned short w_count;

    ADCEN = 1U;    /* supply AD clock */
    ADMK = 1U;    /* disable INTAD interrupt */
    ADIF = 0U;    /* clear INTAD interrupt flag */
    /* Set INTAD priority */
    ADPR1 = 1U;
    ADPR0 = 1U;
    ADM0 = _00_AD_CONVERSION_CLOCK_8 | _00_AD_TIME_MODE_NORMAL_1;
    ADM2 = _00_AD_RESOLUTION_10BIT;
    ADS = _0D_AD_INPUT_INTERREFVOLT;

    ADCE = 1U;

    /* Reference voltage stability wait time, 0.125us */
    for (w_count = 0U; w_count < AD_WAITTIME_B; w_count++ )
    {
        NOP();
    }

    // R_Config_ADC_Start();  
}

void ADC_Process(void)
{    
    // get ADC channel
    ADC_ch_value = GetADC(ADCHANNEL1);
    	
    #if defined (ENABLE_LOG_ADC)   //ADC debug log
    {
        #if 1
        printf_tiny("VREF:%d,",vdd_Vbgr);
        printf_tiny("ch[%d]:0x%04X",ADCHANNEL1,ADC_ch_value);
        printf_tiny("(%d mv)", (vdd_Vbgr*ADC_ch_value) >> 10);
        printf_tiny("\r\n");
        #else
        printf_tiny("VREF:%d,",vdd_Vbgr);
        printf_tiny("ch[%d]:0x%04X",ADCHANNEL1,adc_buffer[ADCHANNEL1]);
        printf_tiny("(%d mv)", (vdd_Vbgr*adc_buffer[ADCHANNEL1]) >> 10);
        printf_tiny("\r\n");
        #endif
    }
    #endif
    
}

void ADC_init(void)
{    
    // init internal vref channel
    ADC_VREF_config_Init();
    // get VREF
    GetVREF();

    // init normal adc channel
    ADC_Channel_config_Init();
}

void ADC_Process_in_IRQ(void)
{
    FLAG_PROJ_TRIG_ADC_CH = 1;
}


/*
    TAU0 PWM : 10K
        - SLAVE 1 : P03 , duty : 100%
*/

unsigned long get_TAU0_pwm_ch_duty(unsigned char ch)
{
	return (pwm_duty[ch]);
}

void set_TAU0_pwm_ch_duty(unsigned char ch ,unsigned long duty)
{
    pwm_duty[ch] = duty;
}

/*copy from R_Config_TAU0_2_Create*/
void generate_TAU0_pwm_ch(void)
{
    TOM0 |= _0008_TAU_CH3_SLAVE_OUTPUT;
    TOL0 &= (unsigned short)~_0008_TAU_CH3_OUTPUT_LEVEL_L;
    TO0 &= (unsigned short)~_0008_TAU_CH3_OUTPUT_VALUE_1;
    TOE0 |= _0008_TAU_CH3_OUTPUT_ENABLE;
}

void PWM_Process_Adjust(void)
{
    int temp_duty = 0;
    unsigned long duty_hex = 0;
    unsigned short data_reg_default = get_TAU0_pwm_ch_data_reg_default();
    unsigned short tmp = 0;

    if (FLAG_PROJ_PWM_DUTY_INC)
    {
        FLAG_PROJ_PWM_DUTY_INC = 0;

        temp_duty = get_TAU0_pwm_ch_duty(3);
        #if defined (ENABLE_LOG_PWM)
        {
            printf_tiny("+duty1:0x%02X,0x%02X\r\n",temp_duty ,data_reg_default);
        }
        #endif

        duty_hex = (data_reg_default / DUTY_RESOLUTION) * 1;   //0.1 %
        temp_duty = (temp_duty >= data_reg_default) ? (data_reg_default) : (temp_duty + duty_hex ) ;  
        #if defined (ENABLE_LOG_PWM)
        {
            printf_tiny("+duty2:0x%02X,0x%02X\r\n",temp_duty ,duty_hex);
        }
        #endif

        set_TAU0_pwm_ch_duty(3,temp_duty);
        #if defined (ENABLE_LOG_PWM)
        {
            // printf_tiny("+duty:0x%02X(%2.2f)\r\n",temp_duty , (float) temp_duty/data_reg_default*100 );
        }
        #endif

        tmp = get_TAU0_pwm_ch_duty(3);
        set_TAU0_pwm_ch_data_reg(tmp);
        generate_TAU0_pwm_ch();
    }
    if (FLAG_PROJ_PWM_DUTY_DEC)
    {
        FLAG_PROJ_PWM_DUTY_DEC = 0;

        temp_duty = get_TAU0_pwm_ch_duty(3);
        #if defined (ENABLE_LOG_PWM)
        {
            printf_tiny("-duty1:0x%02X,0x%02X\r\n",temp_duty ,data_reg_default);
        }
        #endif

        duty_hex = (data_reg_default / DUTY_RESOLUTION) * 1;   //0.1 %
        temp_duty = (temp_duty <= 0) ? (0) : (temp_duty - duty_hex ) ;   
        #if defined (ENABLE_LOG_PWM)
        {
            printf_tiny("-duty2:0x%02X,0x%02X\r\n",temp_duty ,duty_hex); 
        }
        #endif

        set_TAU0_pwm_ch_duty(3,temp_duty);
        #if defined (ENABLE_LOG_PWM)
        {
            // printf_tiny("-duty:0x%02X(%2.2f)\r\n",temp_duty , (float) temp_duty/data_reg_default*100 );
        }
        #endif

        tmp = get_TAU0_pwm_ch_duty(3);
        set_TAU0_pwm_ch_data_reg(tmp);
        generate_TAU0_pwm_ch();
    }
}

void convert_ADC_to_PWM(void)
{
    unsigned long target_duty_hex = 0ul;
    unsigned short tmp = 0;
    unsigned short data_reg_default = get_TAU0_pwm_ch_data_reg_default();

    /*
        duty (%)     ADC value
        -------  = --------------
          100       10bit(0x3FF)

          duty(%) = (ADC value) * 100 / 0x3FF

        duty (%)       duty hex
        -------  = ----------------
          100       data_reg_default

          duty hex = ( duty (%) ) * (data_reg_default) / 100

        ==> duty hex = (ADC value) * (data_reg_default) / 0x3FF

    */

    // target_duty_hex = (unsigned long) ADC_ch_value * data_reg_default / 0x3FF;
    target_duty_hex = (unsigned long) ADC_ch_value * 156 / 100; ;  // 1600 / 1023 = 1.56

    #if defined (ENABLE_LOG_ADC_CVT_PWM)
    printf_tiny("VREF:%d,",vdd_Vbgr);
    // printf_tiny("ADC:0x%04X,",ADC_ch_value);
    printf_tiny("ADC:%4d,",ADC_ch_value);
    // printf_tiny("duty hex:0x%04X", target_duty_hex);
    printf_tiny("duty hex:%4d", target_duty_hex);
    // printf_tiny("(0x%03X),", data_reg_default);
    printf_tiny("(%4d),", data_reg_default);
    printf_tiny("\r\n");
    #endif

    set_TAU0_pwm_ch_duty(3,target_duty_hex);

    tmp = get_TAU0_pwm_ch_duty(3);
    set_TAU0_pwm_ch_data_reg(tmp);
    generate_TAU0_pwm_ch();

}

void LED_Default_reverse(void)
{
    cLED1 = 1;
    cLED2 = 1;
    cLED3 = 1;
    cLED4 = 1;

    cLED5= 1;
    cLED6 = 1;
    cLED7 = 1;
    cLED8 = 1;
}


void LED_Default(void)
{
    cLED1 = 0;
    cLED2 = 0;
    cLED3 = 0;
    cLED4 = 0;

    cLED5 = 0;
    cLED6 = 0;
    cLED7 = 0;
    cLED8 = 0;
}


void LED_state_machine(void)
{
    // interval : 40ms
    static unsigned char state_idx = 0;

    if (FLAG_PROJ_TIMER_PERIOD_LEDs)
    {
        FLAG_PROJ_TIMER_PERIOD_LEDs = 0;
        // printf_tiny("state:%2d\r\n",state_idx);
        switch(state_idx)
        {
            case 0:
                #if defined (ENABLE_GPIO_REVERSE)
                LED_Default_reverse();
                #else
                LED_Default();
                #endif
                break;                
            case 1:
                #if defined (ENABLE_GPIO_REVERSE)
                cLED1 = 0;
                #else
                cLED1 = 1;
                #endif
                break;                
            case 2:
                #if defined (ENABLE_GPIO_REVERSE)
                cLED2 = 0;
                #else
                cLED2 = 1;
                #endif
                break;                
            case 3:
                #if defined (ENABLE_GPIO_REVERSE)
                cLED3 = 0;
                #else
                cLED3 = 1;
                #endif
                break;                
            case 4:
                #if defined (ENABLE_GPIO_REVERSE)
                cLED4 = 0;
                #else
                cLED4 = 1;
                #endif
                break;                
            case 5:
                #if defined (ENABLE_GPIO_REVERSE)
                cLED5 = 0;
                #else
                cLED5 = 1;
                #endif
                break;                
            case 6:
                #if defined (ENABLE_GPIO_REVERSE)
                cLED6 = 0;
                #else
                cLED6 = 1;
                #endif
                break;             
            case 7:
                #if defined (ENABLE_GPIO_REVERSE)
                cLED7 = 0;
                #else
                cLED7 = 1;
                #endif
                break;             
            case 8:
                #if defined (ENABLE_GPIO_REVERSE)
                cLED8 = 0;
                #else
                cLED8 = 1;
                #endif
                break;
        }

        state_idx = ( state_idx < 8 ) ? (state_idx + 1) : (0) ;
    }
}

void Timer_1ms_IRQ(void)
{
    tick_counter();

    if ((get_tick() % 1000) == 0)
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 1;
    }

    if ((get_tick() % 100) == 0)
    {
        FLAG_PROJ_TIMER_PERIOD_ADC = 1;
    }	

    if ((get_tick() % 250) == 0)
    {
        FLAG_PROJ_TIMER_PERIOD_LEDs = 1;
    }	

    Button_Process_long_counter();
}


/*
    G15 target board
    LED1 connected to P66, LED2 connected to P67
*/
// void LED_Toggle(void)
// {
//     // PIN_WRITE(2,0) = ~PIN_READ(2,0);
//     // PIN_WRITE(2,1) = ~PIN_READ(2,1);
//     P2_bit.no0 = ~P2_bit.no0;
//     P2_bit.no1 = ~P2_bit.no1;
// }

void loop(void)
{
	// static unsigned long LOG1 = 0;

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;

        // printf_tiny("log(timer):%4d\r\n",LOG1++);
        // LED_Toggle();             
    }

    if (FLAG_PROJ_TIMER_PERIOD_ADC)
    {
        FLAG_PROJ_TIMER_PERIOD_ADC = 0;
        ADC_Process();  // P02
    }

    Button_Process_in_polling();

    LED_state_machine();
    
    // need t remove convert_ADC_to_PWM , if plan to use terminal D/A to change PWM duty
    // PWM_Process_Adjust();      

    // need t remove PWM_Process_Adjust , if plan to use ADC to change PWM duty   
    convert_ADC_to_PWM();
}


// G15 EVB , P137/INTP0 , set both edge 
void Button_Process_long_counter(void)
{
    if (FLAG_PROJ_TRIG_BTN2)
    {
        btn_tick_counter();
    }
    else
    {
        btn_set_tick(0);
    }
}

void Button_Process_in_polling(void)
{
    static unsigned char cnt = 0;

    if (FLAG_PROJ_TRIG_BTN1)
    {
        FLAG_PROJ_TRIG_BTN1 = 0;
        printf_tiny("BTN pressed(%d)\r\n",cnt);

        P2_bit.no1 = ~P2_bit.no1;

        if (cnt == 0)   //set both edge  , BTN pressed
        {
            FLAG_PROJ_TRIG_BTN2 = 1;
        }
        else if (cnt == 1)  //set both edge  , BTN released
        {
            FLAG_PROJ_TRIG_BTN2 = 0;
        }

        cnt = (cnt >= 1) ? (0) : (cnt+1) ;
    }

    if ((FLAG_PROJ_TRIG_BTN2 == 1) && 
        (btn_get_tick() > BTN_PRESSED_LONG))
    {         
        printf_tiny("BTN pressed LONG\r\n");
        btn_set_tick(0);
        FLAG_PROJ_TRIG_BTN2 = 0;
    }
}

// // G15 EVB , P137/INTP0
void Button_Process_in_IRQ(void)    
{
    FLAG_PROJ_TRIG_BTN1 = 1;
}

void UARTx_Process(unsigned char rxbuf)
{    
    if (rxbuf > 0x7F)
    {
        printf_tiny("invalid command\r\n");
    }
    else
    {
        printf_tiny("press:%c(0x%02X)\r\n" , rxbuf,rxbuf);   // %c :  C99 libraries.
        switch(rxbuf)
        {
            case 'a':
            case 'A':
                FLAG_PROJ_PWM_DUTY_INC = 1;
                break;
            case 'd':
            case 'D':
                FLAG_PROJ_PWM_DUTY_DEC = 1;
                break;

            // case '1':
            //     FLAG_PROJ_TRIG_1 = 1;
            //     break;
            // case '2':
            //     FLAG_PROJ_TRIG_2 = 1;
            //     break;
            // case '3':
            //     FLAG_PROJ_TRIG_3 = 1;
            //     break;
            // case '4':
            //     FLAG_PROJ_TRIG_4 = 1;
            //     break;
            // case '5':
            //     FLAG_PROJ_TRIG_5 = 1;
            //     break;

            // case 'X':
            // case 'x':
            //     RL78_soft_reset(7);
            //     break;
            // case 'Z':
            // case 'z':
            //     RL78_soft_reset(1);
            //     break;
        }
    }
}

/*
    Reset Control Flag Register (RESF) 
    BIT7 : TRAP
    BIT6 : 0
    BIT5 : 0
    BIT4 : WDCLRF
    BIT3 : 0
    BIT2 : 0
    BIT1 : IAWRF
    BIT0 : LVIRF
*/
// void check_reset_source(void)
// {
//     /*
//         Internal reset request by execution of illegal instruction
//         0  Internal reset request is not generated, or the RESF register is cleared. 
//         1  Internal reset request is generated. 
//     */
//     uint8_t src = RESF;
//     printf_tiny("Reset Source <0x%08X>\r\n", src);

//     #if 1   //DEBUG , list reset source
//     if (src & BIT0)
//     {
//         printf_tiny("0)voltage detector (LVD)\r\n");       
//     }
//     if (src & BIT1)
//     {
//         printf_tiny("1)illegal-memory access\r\n");       
//     }
//     if (src & BIT2)
//     {
//         printf_tiny("2)EMPTY\r\n");       
//     }
//     if (src & BIT3)
//     {
//         printf_tiny("3)EMPTY\r\n");       
//     }
//     if (src & BIT4)
//     {
//         printf_tiny("4)watchdog timer (WDT) or clock monitor\r\n");       
//     }
//     if (src & BIT5)
//     {
//         printf_tiny("5)EMPTY\r\n");       
//     }
//     if (src & BIT6)
//     {
//         printf_tiny("6)EMPTY\r\n");       
//     }
//     if (src & BIT7)
//     {
//         printf_tiny("7)execution of illegal instruction\r\n");       
//     }
//     #endif

// }

/*
    7:Internal reset by execution of illegal instruction
    1:Internal reset by illegal-memory access
*/
//perform sofware reset
// void _reset_by_illegal_instruction(void)
// {
//     static const unsigned char illegal_Instruction = 0xFF;
//     void (*dummy) (void) = (void (*)(void))&illegal_Instruction;
//     dummy();
// }

// void _reset_by_illegal_memory_access(void)
// {
//     // #if 1
//     // const unsigned char ILLEGAL_ACCESS_ON = 0x80;
//     // IAWCTL |= ILLEGAL_ACCESS_ON;            // switch IAWEN on (default off)
//     // *(__far volatile char *)0x00000 = 0x00; //write illegal address 0x00000(RESET VECTOR)
//     // #else
//     // signed char __far* a;                   // Create a far-Pointer
//     // IAWCTL |= _80_CGC_ILLEGAL_ACCESS_ON;    // switch IAWEN on (default off)
//     // a = (signed char __far*) 0x0000;        // Point to 0x000000 (FLASH-ROM area)
//     // *a = 0;
//     // #endif
// }

// void RL78_soft_reset(unsigned char flag)
// {
//     switch(flag)
//     {
//         case 7: // do not use under debug mode
//             _reset_by_illegal_instruction();        
//             break;
//         case 1:
//             _reset_by_illegal_memory_access();
//             break;
//     }
// }

// retarget printf
int __far putchar(int c)
{
    // G15 , UART0
    STMK0 = 1U;    /* disable INTST0 interrupt */
    TXD0 = (unsigned char)c;
    while(STIF0 == 0)
    {

    }
    STIF0 = 0U;    /* clear INTST0 interrupt flag */
    return c;
}

void hardware_init(void)
{
    // const unsigned char indicator[] = "hardware_init";
    BSP_EI();
    R_Config_UART0_Start();         // UART , P03 , P04
    R_Config_TAU0_1_Start();        // TIMER
    R_Config_INTC_INTP0_Start();    // BUTTON , P137 

    /*
        RL78 G15 EVB : 
        [J1]
        P21:                EVB LED2// use EVB button to toggle
        P20:                EVB LED1// TO03 PWM output
        P41:                // external LED8
        P40:TOOL0
        T_RESET
        P137 : INTP0,       button , INTP0
        P122
        P121
        GND
        VDD

        [J2]
        P22:                // external LED1
        P23:                // external LED2
        P07:                // external LED3
        P06:                // external LED4
        P05:                // external LED5
        P04:                // UART RX
        P03:                // UART TX
        P02:                // external P02/AIN1 , use ADC to change TO03 PWM output
        P01:                // external LED6
        P00:                // external LED7

    */
    LED_Default();

    /*
        EVB : LED1
    */
    P2_bit.no1 = 1;
    
    R_Config_TAU0_2_Start();        // PWM output : P20/TO03
    ADC_init();                     // ADC : P02/AIN1
    
    // check_reset_source();
    // printf("%s finish\r\n\r\n",__func__);
    printf_tiny("%s finish\r\n\r\n",__func__);
}
