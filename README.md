# RL78_G15_ADC_PWM_GPIO
 RL78_G15_ADC_PWM_GPIO
 
udpate @ 2024/08/12

1. use RL78 G15 EVB to test GPIO , ADC , PWM

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

2. toggle LED * 8 in sequence

3. press EVB button to toggle LED2

4. change ADC value and convert to PWM output

5. below is log message if enable ENABLE_LOG_ADC

![image](https://github.com/released/RL78_G15_ADC_PWM_GPIO/blob/main/log_adc.jpg)


6. below is log message if enable ENABLE_LOG_PWM

![image](https://github.com/released/RL78_G15_ADC_PWM_GPIO/blob/main/log_pwm.jpg)


7. below is log message if enable ENABLE_LOG_ADC_CVT_PWM

![image](https://github.com/released/RL78_G15_ADC_PWM_GPIO/blob/main/log_adc_cvt_pwm.jpg)

