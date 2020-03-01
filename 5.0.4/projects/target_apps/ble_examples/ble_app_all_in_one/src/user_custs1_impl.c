/**
 ****************************************************************************************
 *
 * @file user_custs1_impl.c
 *
 * @brief All in one project Custom1 Server implementation source code.
 *
 * Copyright (C) 2015. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "adc.h"
#include "app.h"
#include "app_api.h"
#include "gpio.h"
#include "pwm.h"
#include "app_bond_db.h"
#include "user_all_in_one.h"
#include "user_custs1_def.h"
#include "user_custs1_impl.h"
#include "user_periph_setup.h"
#include "wkupct_quadec.h"

struct app_proj_env_tag user_app_env __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY


//static uint16_t user_get_adc1(void);
//static uint16_t user_get_adc2(void);
static uint8_t user_hex2utf8(int16_t in,uint8_t dot,uint8_t *out);
static void user_app_get_adj_val(void);
/*
 * DEFINES
 ****************************************************************************************
 */
#define APP_PWM_ON      1000
#define APP_PWM_HIGH    500
#define APP_PWM_LOW     200

#define APP_NOTES_NUM   26

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

ke_msg_id_t timer_used = 0xFFFF;
//uint16_t waringTmp = 3880;
//const uint16_t notes[APP_NOTES_NUM] =
//{
//    1046,987,767,932,328,880,830,609,783,991,739,989,698,456,659,255,622,254,587,554,365,523,251,493,466,440
//};

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
*/

static void user_app_pwm_callback_function(void);
static void user_app_enable_pwm(void);
static void user_app_disable_pwm(void);
static void user_app_set_button_event(uint8_t);
static void user_app_disable_button(void);
static void user_app_disable_button(void);
static void user_app_button_press_cb(void);
static void user_app_get_bat_val(void);
//void user_app_disable_led(void);
//void user_app_enable_led(void);

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void user_custs1_ctrl_wr_ind_handler(ke_msg_id_t const msgid,
                                      struct custs1_val_write_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t val = 0;
    memcpy(&val, &param->value[0], param->length);

    if (val == 0)
    {
		/*if (timer_used == 0xFFFF)
		{
			timer_used = app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY, user_app_adcval1_timer_cb_handler);
		}*/
//		arch_set_deep_sleep();			
		arch_set_extended_sleep();	   // lewis 	
    }
    else if (val == 1)
    {
        /*if (timer_used != 0xFFFF)
        {
            app_easy_timer_cancel(timer_used);
            timer_used = 0xFFFF;
        }*/
		arch_set_extended_sleep();
    }
    if (val == CUSTS1_CP_CMD_PWM_ENABLE)
    {
        user_app_enable_pwm();
    }
    else if (val == CUSTS1_CP_CMD_PWM_DISABLE)
    {
        user_app_disable_pwm();
    }
    /*else if (val == CUSTS1_CP_CMD_ADC_VAL_2_ENABLE)
    {
        user_app_env.custs1_adcval2_enabled = 1;
        user_app_get_adcval2_val();
    }
    else if (val == CUSTS1_CP_CMD_ADC_VAL_2_DISABLE)
    {
        user_app_env.custs1_adcval2_enabled = 0;
    }*/
}

void user_custs1_led_wr_ind_handler(ke_msg_id_t const msgid,
                                     struct custs1_val_write_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint8_t val = param->value[0];
    //memcpy(&val, &param->value[0], param->length);

    if (val == CUSTS1_LED_ON)
    {
        user_app_enable_led();
        //arch_force_active_mode();
    }
    else if (val == CUSTS1_LED_OFF)
    {
        user_app_disable_led();
        //arch_restore_sleep_mode();
    }
}
/*
void user_custs1_long_val_cfg_ind_handler(ke_msg_id_t const msgid,
                                           struct custs1_val_write_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
}
*/

void user_custs1_long_val_wr_ind_handler(ke_msg_id_t const msgid,
                                          struct custs1_val_write_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
	char val[8];
	float valf;
	memset(val,0,sizeof(val));
	memcpy(val, &param->value[0], param->length);
	if(val[2] == '.')
	{
		valf = (val[0] - 0x30)*1000 + (val[1] - 0x30)*100 + \
			   (val[3] - 0x30)*10 + (val[4] - 0x30);//+ (val[4] - 0x30);
		user_tempadj_data.adjTemp = valf / 100.0f; 
		if(user_tempadj_data.curTemp)
		{
			user_tempadj_data.flags = 0x00;
			user_tempadj_data.valid = 0x01;
			user_tempadj_data.adjData = user_tempadj_data.adjTemp / user_tempadj_data.curTemp;

			user_config_data.flags = 0x00;	
			user_config_data.adjData1 *= user_tempadj_data.adjData;
		}	
	}
	else if(strcmp(val,"save9") == 0)
	{
		user_config_data.valid = 0x01;//工厂校准标志位
		user_config_data.flags = 0x01;	
		user_config_data.adjTemp = user_tempadj_data.adjTemp;		
		bond_usercfgdata_store_flash();		
	}	
	else if(strcmp(val,"save") == 0)
	{	
		user_tempadj_data.valid = 0x01;//用户校准标志位
		user_tempadj_data.flags = 0x01;			
		bond_useradjdata_store_flash();
	}
	else if(strcmp(val,"read") == 0)
	{
		user_app_get_adj_val();
	}
	else if(strcmp(val,"clear") == 0)
	{
		memset(&user_tempadj_data,0,sizeof(user_tempadj_data));
		bond_useradjdata_store_flash();
	}
	else if(strcmp(val,"clear9") == 0)
	{
		memset(&user_config_data,0,sizeof(user_config_data));
		user_config_data.adjData1 = 1.0f;
		bond_usercfgdata_store_flash();
	}
	
}
/*
void user_custs1_long_val_ntf_cfm_handler(ke_msg_id_t const msgid,
                                           struct custs1_val_write_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
}
*/
void user_custs1_adc_val_1_cfg_ind_handler(ke_msg_id_t const msgid,
                                            struct custs1_val_write_ind const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
}

void user_custs1_adc_val_1_ntf_cfm_handler(ke_msg_id_t const msgid,
                                            struct custs1_val_write_ind const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
}

void user_custs1_button_cfg_ind_handler(ke_msg_id_t const msgid,
                                         struct custs1_val_write_ind const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
}

void user_custs1_button_ntf_cfm_handler(ke_msg_id_t const msgid,
                                         struct custs1_val_write_ind const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
}
/*
void user_custs1_indicateable_cfg_ind_handler(ke_msg_id_t const msgid,
                                               struct custs1_val_write_ind const *param,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id)
{
}

void user_custs1_indicateable_ind_cfm_handler(ke_msg_id_t const msgid,
                                               struct custs1_val_write_ind const *param,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id)
{
}
*/
void user_app_adcval1_timer_cb_handler()
{
    struct custs1_val_ntf_req* req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_NTF_REQ,
                                                      TASK_CUSTS1,
                                                      TASK_APP,
                                                      custs1_val_ntf_req,
                                                      DEF_CUST1_ADC_VAL_1_CHAR_LEN);

    // ADC value to be sampled
    //static uint8_t kk =0;
	uint8_t data[6] = {0,0,0,0,0,0},len;
	static uint16_t tmp1 = 0;
	float tmp,fcoe = 0.2f;
	//float adjcfg = user_config_data.adjData1 / 1000.0f;
	uint16_t sample = user_get_adc1();
	if(user_config_data.adjData1)
	{
		tmp = sample * user_config_data.adjData1;//4.378f;
	}
	else
	{
		tmp = sample;
	}

//	fcoe = user_config_data.flags == 0x00 ? 0.5f : 0.1f;
	if(tmp1 == 0)
	{
		tmp1 = tmp;
	}
	tmp -= tmp1;	
	tmp1 += tmp * fcoe;
	
	user_tempadj_data.curTemp = tmp1 / 100.0f; 
	
	/*uint16_t adc_sample = 3751+kk;//user_get_adc1();
	if(++kk > 100)
		kk = 0;*/
	len = user_hex2utf8(tmp1,2,data);
	
    req->conhdl = app_env->conhdl;
    req->handle = CUST1_IDX_ADC_VAL_1_VAL;
    req->length = len;
    memcpy(req->value, data, len);

    ke_msg_send(req);

    if (ke_state_get(TASK_APP) == APP_CONNECTED)
    {
        // Set it once again until Stop command is received in Control Characteristic
        timer_used = app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY, user_app_adcval1_timer_cb_handler);
    }
}
void user_app_pwm_timer_cb_handler()
{
	static uint8_t i =0;
	if(i == 0)
	{
		i = 1;
		user_app_enable_pwm();
		GPIO_SetActive( GPIO_PORT_1, GPIO_PIN_0);//led on
	}
	else
	{
		i = 0;
		 user_app_disable_pwm();
		GPIO_SetInactive( GPIO_PORT_1, GPIO_PIN_0);//led off
	}
	app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY, user_app_pwm_timer_cb_handler);
}
/**
 ****************************************************************************************
 * @brief Disable pwm timer
 * @return void
 ****************************************************************************************
*/
static void user_app_disable_pwm(void)
{
    if(user_app_env.custs1_pwm_enabled)
    {
        user_app_env.custs1_pwm_enabled = 0;

        arch_restore_sleep_mode();
#if 0
        timer0_stop();
        set_tmr_enable(CLK_PER_REG_TMR_DISABLED);
#endif
		timer2_stop();
		timer2_enable(TRIPLE_PWM_DISABLED);	
		set_tmr_enable(CLK_PER_REG_TMR_DISABLED);		
    }
}

/**
 ****************************************************************************************
 * @brief Tiemer callback function. Update PWM settings
 * @return void
 ****************************************************************************************

static void user_app_pwm_callback_function(void)
{
    static uint8_t change_delay = 0;
    static uint8_t notes_idx = 0;

    if (change_delay == 10)
    {
        change_delay = 0;
        timer0_set_pwm_on_counter(0xFFFF);
        timer0_set_pwm_high_counter(notes[notes_idx]/3 * 2);
        timer0_set_pwm_low_counter(notes[notes_idx]/3);
        notes_idx = ++notes_idx % (APP_NOTES_NUM -1);
    }
    change_delay++;
}
*/
/**
 ****************************************************************************************
 * @brief Enable pwm timer
 * @return void
 ****************************************************************************************
*/
static void user_app_enable_pwm(void)
{
#define PWM_FREQUENCY     400	
    if(!user_app_env.custs1_pwm_enabled)
    {
        user_app_env.custs1_pwm_enabled = 1;

        // Disable sleep mode
        arch_force_active_mode();

        // Enable TIMER clock
        set_tmr_enable(CLK_PER_REG_TMR_ENABLED);
        // Set clock division
        set_tmr_div(CLK_PER_REG_TMR_DIV_8);
#if 0		
//        // Init timer 0
//        timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_NO_DIV);
//        // Set timing parameters
//        timer0_set(APP_PWM_ON, APP_PWM_HIGH, APP_PWM_LOW);
//        // Register IRQ callback
//        timer0_register_callback(user_app_pwm_callback_function);
//        // Enable IRQ
//        timer0_enable_irq();
//        // Start timer
//        timer0_start();
#endif
/**/
    timer2_enable(TRIPLE_PWM_ENABLED);      
    timer2_set_hw_pause(HW_CAN_NOT_PAUSE_PWM_2_3_4);
    timer2_set_sw_pause(HW_CAN_NOT_PAUSE_PWM_2_3_4);
	timer2_set_pwm_frequency(PWM_FREQUENCY);
	timer2_set_pwm2_duty_cycle(PWM_FREQUENCY >> 2 );//& BIT_MASK
	timer2_enable(TRIPLE_PWM_ENABLED);
    }
}
void user_app_disable_led(void)
{
	arch_restore_sleep_mode();
	GPIO_SetInactive(GPIO_LED_PORT, GPIO_LED_PIN);
}
void user_app_enable_led(void)
{
	// Disable sleep mode
	arch_force_active_mode();
	GPIO_SetActive(GPIO_LED_PORT, GPIO_LED_PIN);
}
uint8_t user_app_get_led_status(void)
{
	return GPIO_GetPinStatus(GPIO_LED_PORT, GPIO_LED_PIN);
}
/**
 ****************************************************************************************
 * @brief Set button event configuration
 * @return void
 ****************************************************************************************
*/
static void user_app_set_button_event(uint8_t next_event)
{
	
    wkupct_register_callback(user_app_button_press_cb);

    wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN),
                      WKUPCT_PIN_POLARITY(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, next_event), // polarity
                      1, // 1 event
                      10); // debouncing time = 10 ms

}

/**
 ****************************************************************************************
 * @brief Disable button action.
 * @return void
 ****************************************************************************************
*/
static void user_app_disable_button(void)
{
    wkupct_disable_irq();
}

/**
 ****************************************************************************************
 * @brief Callback function for button action. Update button state charactersitc.
 * @return void
 ****************************************************************************************
*/
static void user_app_button_press_cb(void)
{
    uint8_t next_btn_event = 0;

    // Read button state
    if(GPIO_GetPinStatus( GPIO_BUTTON_PORT, GPIO_BUTTON_PIN ))
    {
        user_app_env.custs1_btn_state = CUSTS1_BTN_STATE_RELEASED;
        next_btn_event = WKUPCT_PIN_POLARITY_LOW;
    }
    else
    {
        user_app_env.custs1_btn_state = CUSTS1_BTN_STATE_PRESSED;
        next_btn_event = WKUPCT_PIN_POLARITY_HIGH;
    }

    // Update button characterstic
    struct custs1_val_ntf_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_NTF_REQ,
                                                      TASK_CUSTS1,
                                                      TASK_APP,
                                                      custs1_val_ntf_req,
                                                      DEF_CUST1_BUTTON_STATE_CHAR_LEN);

    req->conhdl = app_env->conhdl;
    req->handle = CUST1_IDX_BUTTON_STATE_VAL;
    req->length = DEF_CUST1_BUTTON_STATE_CHAR_LEN;
    req->value[0] = user_app_env.custs1_btn_state;

    ke_msg_send(req);
    // Configure next button event
    user_app_set_button_event(next_btn_event);
	arch_set_extended_sleep();
}

/**
 ****************************************************************************************
 * @brief Read ADC val and update ADCVAL2 characteristic
 * @return void
 ****************************************************************************************
*/
static void user_app_get_bat_val(void)
{
    if(user_app_env.custs1_adcval2_enabled)
    {
        uint16_t adc_sample;
		uint8_t data[6] = {0,0,0,0,0,0},len;
        //adc_calibrate();
        //adc_sample = (uint16_t)adc_get_vbat_sample(false);
		adc_sample = user_get_adc2();
		len = user_hex2utf8(adc_sample,3,data); 

        struct custs1_val_set_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_SET_REQ,
                                                          TASK_CUSTS1,
                                                          TASK_APP,
                                                          custs1_val_set_req,
                                                          DEF_CUST1_ADC_VAL_2_CHAR_LEN);

       req->conhdl = app_env->conhdl;
       req->handle = CUST1_IDX_ADC_VAL_2_VAL;
       req->length = len;
       memcpy(req->value, data, len);

       ke_msg_send(req);
    }
}

static void user_app_get_adj_val(void)
{       
		uint16_t adj = user_tempadj_data.adjTemp * 100;
		uint8_t data[6] = {0,0,0,0,0,0},len;
        //adc_calibrate();
        //adc_sample = (uint16_t)adc_get_vbat_sample(false);
		//adc_sample = user_get_adc2();
		//len = user_hex2utf8(adj,2,data);
		
        struct custs1_val_set_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_SET_REQ,
                                                          TASK_CUSTS1,
                                                          TASK_APP,
                                                          custs1_val_set_req,
                                                          DEF_CUST1_LONG_VALUE_CHAR_LEN);

       req->conhdl = app_env->conhdl;
       req->handle = CUST1_IDX_LONG_VALUE_VAL;
       req->length = user_hex2utf8(adj,2,data);
       memcpy(req->value, data, req->length);

       ke_msg_send(req);
}
uint16_t swapInt16(uint16_t value)  
{  
	return  ((value & 0x00FF) << 8) |  
			((value & 0xFF00) >> 8) ;  
}
static void user_periph_init(void)
{
	adc_calibrate();
}
static uint16_t user_get_adc_common(uint16_t ch,uint16_t attn)
{
    uint32_t adc_sample, adc_sample2;
	uint16_t attenuation = 0;
	
	if(attn == 3)
	{
		attenuation = GP_ADC_ATTN3X;
	}
	
    adc_init(GP_ADC_SE, GP_ADC_SIGN, attenuation);
    adc_usDelay(20);

    adc_enable_channel(ch);

    adc_sample = adc_get_sample();
    adc_usDelay(1);
    adc_init(GP_ADC_SE, 0, attenuation);

    adc_enable_channel(ch);

    adc_sample2 = adc_get_sample();
    //We have to divide the following result by 2 if
    //the 10 bit accuracy is enough
    adc_sample = (adc_sample2 + adc_sample) >> 1;
    //adc_disable();

    return adc_sample;
}
static uint8_t user_hex2utf8(int16_t in,uint8_t dot,uint8_t *out)
{
	uint8_t i = 0,m,tmp[6] = {0,0,0,0,0,0};	
	tmp[i++] = in / 10000 + 0x30;
	if(dot == 4)tmp[i++] = '.';	
	tmp[i++] = in % 10000 / 1000 + 0x30;
	if(dot == 3)tmp[i++] = '.';	
	tmp[i++] = in % 1000 / 100 + 0x30;
	if(dot == 2)tmp[i++] = '.';	
	tmp[i++] = in % 100 / 10 + 0x30;
	if(dot == 1)tmp[i++] = '.';	
	tmp[i++] = in % 10 + 0x30;
	
	for(m = 0;m<5;m++)
	{
		if((tmp[m] == 0x30) && (tmp[m+1] != 0x30))
		{
			break;
		}
	}
	for(int j =m + 1,i=0;j<(6-m);j++,i++)
	{
		out[i] = tmp[j];
	}
	return i;
}
#define ADC_RESOLUTION      1023
#define ADC_ATTENUATION_1    1
#define ADC_ATTENUATION_3    3
#define ADC_VOLTAGE_REFER  1200  //mv	
uint16_t user_get_adc1(void)
{
	char printfBuf[8] = {0};
	uint16_t adc_sample, adc_sample0;

	adc_sample = user_get_adc_common(ADC_CHANNEL_P01,ADC_ATTENUATION_1);
	
	adc_sample0 = (unsigned int)((float)(adc_sample  * ADC_ATTENUATION_1 * ADC_VOLTAGE_REFER) / ADC_RESOLUTION);
//	sprintf(printfBuf,"%d",adc_sample);
	//adc_sample0 = swapInt16(adc_sample0);
	return adc_sample0;
}
uint16_t user_get_adc2(void)
{
    uint32_t adc_sample, adc_sample0;	
//	char printfBuf[8] = {0};

	adc_sample = user_get_adc_common(ADC_CHANNEL_VBAT3V,ADC_ATTENUATION_3);
	
	adc_sample0 = (unsigned int)((float)(adc_sample  * ADC_ATTENUATION_3 * ADC_VOLTAGE_REFER) / ADC_RESOLUTION);
	//adc_sample0 = swapInt16(adc_sample0);
	//	sprintf(printfBuf,"%d",adc_sample);
	return adc_sample0;	
}
/**
 ****************************************************************************************
 * @brief Enable peripherals used by application.
 * @return void
 ****************************************************************************************
*/
void user_app_enable_periphs(void)
{
	user_periph_init();
	
    // Update button state characteristic
    user_app_button_press_cb();
    // Get initial ADC value if enabled
    
 	timer_used = app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY, user_app_adcval1_timer_cb_handler);
//	app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY, user_app_pwm_timer_cb_handler);
	user_app_env.custs1_adcval2_enabled = 1;
	user_app_get_bat_val();
	user_app_get_adj_val();
	arch_set_extended_sleep(); // by aizj add for bugs: long press btn 800uA on connected 
	
	if(user_config_data.valid == 0x01) //工厂温度校准标志位
	{
		if(user_tempadj_data.valid == 0x01)//用户校准数据标志位
		{
			user_config_data.adjData1 *= user_tempadj_data.adjData;
		}
	}
	else//未校准，使用默认数据
	{
		memset(&user_config_data,0,sizeof(user_config_data));
		memset(&user_tempadj_data,0,sizeof(user_tempadj_data));
		
		user_config_data.adjData1 = 4.738f;//工厂校准数据
		user_tempadj_data.adjData = 1.0f;
		user_config_data.adjTemp = 38.00f;//用户校准温度数据-38
//		bond_usercfgdata_store_flash();
//		bond_useradjdata_store_flash();  
	}
}

/**
 ****************************************************************************************
 * @brief Disable peripherals used by application.
 * @return void
 ****************************************************************************************
*/
void user_app_disable_periphs(void)
{
    user_app_disable_pwm();
    user_app_disable_button();
	//user_app_disable_led();
}
