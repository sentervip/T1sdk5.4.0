/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Battery level example for DA14580/581 SDK
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
#include <stdio.h>

#include "global_io.h"
#include "common_uart.h"
#include "user_periph_setup.h"
#include "battery.h"
#include "adc.h"
#include "pwm.h"
#include "gpio.h"
void system_init(void);
void batt_test(void);

#define ADC_RESOLUTION      1023
#define ADC_ATTENUATION    1
#define ADC_VOLTAGE_REFER  1200  //mv

void adc_test(void)
{
	char printfBuf[16];
	uint32_t adc_sample,adc_sample2;
	
	adc_calibrate();
	
	adc_sample = adc_get_vbat_sample(0);
	sprintf(printfBuf,"adc_sample0 = %d mV\r\n",adc_sample);
	printf_string(printfBuf);
	
	adc_sample2 = adc_get_vbat_sample(1);
	sprintf(printfBuf,"adc_sample1 = %d mV\r\n",adc_sample2);
	printf_string(printfBuf);	
    adc_init(GP_ADC_SE, GP_ADC_SIGN, 0);	
	
}
void read_adc(void)
{
	char printfBuf[16];
	uint16_t adc_sample, adc_sample1,adc_sample2;
	static uint16_t tmp1;
	float tmp = 0;

	
//	adc_enable_channel(ADC_CHANNEL_P00);
//	adc_sample1 = adc_get_sample();
//	printf_string("ADC_CHANNEL_P00 = ");
//	print_hword(adc_sample1);
//	printf_string("\n\r");
//	adc_usDelay(100);
	
	adc_enable_channel(ADC_CHANNEL_P01);
	adc_sample2 = adc_get_sample();
	adc_sample = (unsigned int)((float)(adc_sample2  * ADC_ATTENUATION * ADC_VOLTAGE_REFER) / ADC_RESOLUTION);
	sprintf(printfBuf,"P01 = %d mV\r\n",adc_sample);	
	printf_string(printfBuf);
	
	tmp = adc_sample * 4.378f;
	tmp -= tmp1;	
	tmp1 += tmp *0.5f;
	
	sprintf(printfBuf,"tmp = %d \r\n",tmp1);	
	printf_string(printfBuf);
	
	adc_usDelay(100);
	adc_enable_channel(ADC_CHANNEL_P02);
	adc_sample2 = adc_get_sample();
	adc_sample = (unsigned int)((float)(adc_sample2  * ADC_ATTENUATION * ADC_VOLTAGE_REFER) / ADC_RESOLUTION);
	sprintf(printfBuf,"P02 = %d mV\r\n",adc_sample);
	printf_string(printfBuf);
}
// pwm settings
#define PWM_FREQUENCY     0x1000
#define BIT_MASK          0xFFF
#define PWM2_DUTY_STEP    (PWM_FREQUENCY / 10)
#define PWM34_DUTY_STEP   0x100
#define PWM3_MAX_DUTY     0x1000
uint16_t pwm_duty,pwm_freq=200;
void timer_init(void)
{
    //Enables TIMER0,TIMER2 clock
    set_tmr_enable(CLK_PER_REG_TMR_ENABLED);

    //Sets TIMER0,TIMER2 clock division factor to 8, so TIM0 Fclk is F = 16MHz/8 = 2Mhz
    set_tmr_div(CLK_PER_REG_TMR_DIV_8);

    // initialize PWM with the desired settings
    //timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_NO_DIV);

    // initialize PWM with the desired settings (sw paused)PWM_2_3_4_SW_PAUSE_ENABLED
//    timer2_init(HW_CAN_NOT_PAUSE_PWM_2_3_4, PWM_2_3_4_SW_PAUSE_DISABLED, PWM_FREQUENCY);
    timer2_enable(TRIPLE_PWM_ENABLED);      
    timer2_set_hw_pause(0);
    timer2_set_sw_pause(0);
	timer2_set_pwm_frequency(pwm_freq);
	timer2_set_pwm2_duty_cycle(pwm_freq >> 1);//& BIT_MASK
//	timer2_stop();
//	timer2_set_pwm2_duty_cycle(50);
//	timer2_set_pwm3_duty_cycle(50);
//	timer2_set_pwm4_duty_cycle(50);
}
/**
 ****************************************************************************************
 * @brief  Main routine of the battery level example
 * 
 ****************************************************************************************
 */
uint16_t i = 0,j;
uint32_t a1 = 5000,a2 = 5000;
char buff[16];

int main (void)
{
    system_init();
    periph_init();
    batt_test();
	adc_test();
	timer_init();
    while(1)
	{
		
		//adc_usDelay(1000000);
		read_adc();
		if(i == 0)
		{
			i = 1;
			timer2_enable(TRIPLE_PWM_DISABLED);
			GPIO_SetActive( GPIO_PORT_1, GPIO_PIN_0);//led on
			adc_usDelay(a1);
		}
		else
		{
			i = 0;
			timer2_enable(TRIPLE_PWM_ENABLED);
			GPIO_SetInactive( GPIO_PORT_1, GPIO_PIN_0);//led off
			adc_usDelay(a2);
		}
		//for(j = 0 ; j < 10000 ; j++ )
		{
			//pwm_freq += 200;
			//memset(buff,0,sizeof(buff));
			//sprintf(buff,"pwm_freq = %f hz.\r\n",1000000.0f/pwm_freq);
			//printf_string(buff);
			
			//timer2_set_pwm_frequency(pwm_freq);
			// set pwm2 duty cycle
			//timer2_set_pwm2_duty_cycle(pwm_freq >> 1 );//& BIT_MASK
			
			// set pwm3 duty cycle
			//timer2_set_pwm3_duty_cycle(PWM3_MAX_DUTY - ((j * PWM34_DUTY_STEP) & BIT_MASK) + 1);
			
			// set pwm4 duty cycle
			//timer2_set_pwm4_duty_cycle((j * PWM34_DUTY_STEP) & BIT_MASK);
			
			// release sw pause to let pwm2,pwm3,pwm4 run
			//timer2_set_sw_pause(PWM_2_3_4_SW_PAUSE_DISABLED);

			// delay approx 1 second to observe change of duty cycle
			//simple_delay();
			//adc_usDelay(1000000);
			//timer2_set_sw_pause(PWM_2_3_4_SW_PAUSE_ENABLED);
		}
		//timer2_stop();
		//printf_byte_dec(adc_get_sample());
		//printf_string("000\n\r");
	}
}

 /**
 ****************************************************************************************
 * @brief System Initiialization 
 *
 * 
 ****************************************************************************************
 */

void system_init(void)
{
    SetWord16(CLK_AMBA_REG, 0x00);                 // set clocks (hclk and pclk ) 16MHz
    SetWord16(SET_FREEZE_REG,FRZ_WDOG);            // stop watch dog    
    SetBits16(SYS_CTRL_REG,PAD_LATCH_EN,1);        // open pads
    SetBits16(SYS_CTRL_REG,DEBUGGER_ENABLE,1);     // open debugger
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP,0);       // exit peripheral power down
}

/**
 ****************************************************************************************
 * @brief Battery Level Indication  example function 
 * 
 ****************************************************************************************
 */
void batt_test(void)
{
    printf_string("\n\r\n\r");
    printf_string("*******************\n\r");
    printf_string("* 3V BATTERY TEST *\n\r");
    printf_string("*******************\n\r");

#if BATT_CR2032    
    printf_string("\n\rBattery type: CR2032");  
    printf_string("\n\rCurrent battery level (%): "); 
	
    printf_byte_dec(battery_get_lvl(BATT_CR2032));
    printf_string("% left");
#else
    printf_string("\n\rBattery type unknown");  
#endif
    printf_string("\n\rEnd of test\n\r");
}
