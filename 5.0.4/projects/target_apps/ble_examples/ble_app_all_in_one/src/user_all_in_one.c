/**
 ****************************************************************************************
 *
 * @file user_all_in_one.c
 *
 * @brief All in one project source code.
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

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "arch_api.h"
#include "app_bond_db.h"
#include "app_prf_perm_types.h"
#include "gap.h"
#include "gpio.h"
#include "user_all_in_one.h"
#include "user_custs1_def.h"
#include "user_custs1_impl.h"
#include "user_periph_setup.h"
#include "wkupct_quadec.h"
#include "app_easy_security.h"
#include "app_task.h"


#if (BLE_SPOTA_RECEIVER)
#include "app_spotar.h"
#if defined(__DA14583__) && (!SPOTAR_SPI_DISABLE)
#include "spi_flash.h"
#endif
#endif

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
static void app_button_enable(void);

// Manufacturer Specific Data ADV structure type
struct mnf_specific_data_ad_structure
{
    uint8_t ad_structure_size;
    uint8_t ad_structure_type;
    uint8_t company_id[APP_AD_MSD_COMPANY_ID_LEN];
    uint8_t proprietary_data[APP_AD_MSD_DATA_LEN];
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

uint8_t app_connection_idx                    __attribute__((section("retention_mem_area0"),zero_init)); // @RETENTION MEMORY
timer_hnd app_adv_data_update_timer_used      __attribute__((section("retention_mem_area0"),zero_init)); // @RETENTION MEMORY
timer_hnd app_param_update_request_timer_used __attribute__((section("retention_mem_area0"),zero_init)); // @RETENTION MEMORY
timer_hnd wakeup_led_ctrl_used                __attribute__((section("retention_mem_area0"),zero_init));
timer_hnd app_check_button_used               __attribute__((section("retention_mem_area0"),zero_init));
timer_hnd cali_param_update_used              __attribute__((section("retention_mem_area0"),zero_init));

// aizj add
uint8_t app_connection_flag                    __attribute__((section("retention_mem_area0"),zero_init)); // @RETENTION MEMORY
uint8_t app_connection_flag2                    __attribute__((section("retention_mem_area0"),zero_init));

// Manufacturer Specific Data
struct mnf_specific_data_ad_structure mnf_data __attribute__((section("retention_mem_area0"),zero_init)); // @RETENTION MEMORY

// Stored static random address
struct gap_bdaddr stored_addr                  __attribute__((section("retention_mem_area0"),zero_init)); // @RETENTION MEMORY


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Manufacturer Specific Data
 * @return void
 ****************************************************************************************
 */
static void mnf_data_init()
{
    mnf_data.ad_structure_size = sizeof(struct mnf_specific_data_ad_structure ) - sizeof(uint8_t); // minus the size of the ad_structure_size field
    mnf_data.ad_structure_type = GAP_AD_TYPE_MANU_SPECIFIC_DATA;
    mnf_data.company_id[0] = APP_AD_MSD_COMPANY_ID & 0xFF; // LSB
    mnf_data.company_id[1] = (APP_AD_MSD_COMPANY_ID >> 8 )& 0xFF; // MSB
    mnf_data.proprietary_data[0] = 0;
    mnf_data.proprietary_data[1] = 0;
}

/**
 ****************************************************************************************
 * @brief Update Manufacturer Specific Data
 * @return void
 ****************************************************************************************
 */
static void mnf_data_update()
{
    uint16_t data;

    data = mnf_data.proprietary_data[0] | (mnf_data.proprietary_data[1] << 8);
    data += 1;
    mnf_data.proprietary_data[0] = data & 0xFF;
    mnf_data.proprietary_data[1] = (data >> 8) & 0xFF;

    if (data == 0xFFFF) {
         mnf_data.proprietary_data[0] = 0;
         mnf_data.proprietary_data[1] = 0;
    }
}

/**
 ****************************************************************************************
 * @brief Advertisement data timer update callback function.
 * @return void
 ****************************************************************************************
*/
static void adv_data_update_timer_cb()
{
    app_easy_gap_advertise_stop();
}

/**
 ****************************************************************************************
 * @brief Parameter update request timer callback function.
 * @return void
 ****************************************************************************************
*/
static void param_update_request_timer_cb()
{
    app_easy_gap_param_update_start(app_connection_idx);
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
}

/**
 ****************************************************************************************
 * @brief Application wakeup callback function. Registerd in API message utility.
 * @return void
 ****************************************************************************************
*/
static void app_wakeup_cb(void)
{
    // If state is not idle, ignore the message
    if (ke_state_get(TASK_APP) == APP_CONNECTABLE)
    {
            user_app_adv_start();
            
    }
}
static void app_wakeup_led_ctrl_cb(void)
{
//   static uint8_t powerOnLedFinshFlag =0;
//   static uint16_t powerOffCount = 0;
    if(arch_ble_ext_wakeup_get() == false)
    {
        //if(powerOnLedFinshFlag == 0)
        {
            //powerOnLedFinshFlag = 1;
            for(int j = 0;j<20;j++)
            {
                if(user_app_get_led_status() == 0)
                {
                    user_app_enable_led();
                }else{
                    user_app_disable_led();
                }
                for(int m=0;m<200000;m++);
            }
        }
    }       
}
static void cali_param_update_cb(void)
{
    uint8_t initFlag = 0,states = 1,startCaliFalg = 0;
    float tmp1 = 0,varA = 0,varB = 100;
    uint16_t len_count = 0,sample,dataSize = 100;
    float tmp,fcoe = 0.005f;
    uint32_t stateCnt = 800000,caliCnt = 0;
    
    
    //if(initFlag == 0x00)
    {
    //  initFlag = 0x01;
        periph_init();
        user_app_enable_periphs();
        //arch_set_extended_sleep();       
        arch_force_active_mode();
        GPIO_SetActive(GPIO_POWER_PORT, GPIO_POWER_PIN);//power on
        user_tempadj_data.adjTemp = FACTORY_TEMPRATURE_CALI;
        user_tempadj_data.adjData = 1.0f;
        user_config_data.adjData1 = 1.0f;
        user_config_data.flags = 0x02;
    }
        
    while(states)
    {
        len_count++;
#if (USE_WDOG)      
//      wdg_freeze();            // Stop WDOG
        wdg_reload(WATCHDOG_DEFAULT_PERIOD);          // Reset WDOG!        
#endif      
		
        if(GPIO_GetPinStatus( GPIO_BUTTON_CALI_PORT, GPIO_BUTTON_CALI_PIN ) == 0)
        {
            startCaliFalg = 1;
            stateCnt = 400000;
        }
        
        if(startCaliFalg)
        {
            len_count = 0;
            sample = user_get_adc1();
            tmp = sample * user_config_data.adjData1;
            tmp -= tmp1;    
            tmp1 += tmp * fcoe; 
            user_tempadj_data.curTemp = tmp1 / 100.0f; 
            
            if(user_tempadj_data.curTemp != 0)
            {
                varA += (user_tempadj_data.curTemp - varA)/dataSize;
                varB = (dataSize-2)*(varB)/(dataSize-1)+(user_tempadj_data.curTemp - varA)*(user_tempadj_data.curTemp - varA)/dataSize; 
                user_tempadj_data.varA = varA;
                user_tempadj_data.varB = varB;
            }
            
            if((user_tempadj_data.varB < FACTORY_ERROR_CALI) && (states == 0x01))
            {
                stateCnt = 100000;
                caliCnt++;
                if(caliCnt > 10)
                {
                    user_config_data.adjTemp = user_tempadj_data.adjTemp;
                    user_config_data.adjData1 = user_config_data.adjTemp / user_tempadj_data.curTemp;
                    user_config_data.valid = 1;
                    user_config_data.flags = 1;
                    

                    bond_usercfgdata_store_flash();
                    
                    user_tempadj_data.valid = 1;
                    user_tempadj_data.flags = 1;
                    bond_useradjdata_store_flash();
                    
                    states = 0x02;
                }
            }
            else
            {
                caliCnt = 0;
                stateCnt = 200000;
            }
        }
        //led display
        if(user_config_data.flags == 0x01)//校准完成,LED常亮
        {
            GPIO_SetActive(GPIO_LED_PORT, GPIO_LED_PIN);            
            app_easy_timer_cancel(cali_param_update_used);
        }
        else if(user_config_data.flags == 0x02)
        {
            if(user_app_get_led_status())
            {
                //user_app_disable_led();
                GPIO_SetInactive(GPIO_LED_PORT, GPIO_LED_PIN);
            }
            else
            {
                //user_app_enable_led();
                GPIO_SetActive(GPIO_LED_PORT, GPIO_LED_PIN);
            }
            for(int m=0;m<stateCnt;m++);
        }else{ //校准失败,LED熄灭
            GPIO_SetInactive(GPIO_LED_PORT, GPIO_LED_PIN);
        }
        
        if(GPIO_GetPinStatus( GPIO_BUTTON_PORT, GPIO_BUTTON_PIN ) == 0) // 开关按键按下 重启系统
        {
            states = 0x00; 
            GPIO_SetInactive(GPIO_LED_PORT, GPIO_LED_PIN);
        }
    }//end while(1)
    NVIC_SystemReset(); 
//  if(cali_param_update_used != EASY_TIMER_INVALID_TIMER)
//  {
//      cali_param_update_used = app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY,cali_param_update_cb);
//  }   
}

static void app_check_button_cb(void)
{
    static uint16_t powerOffCount = 0;  
    static uint16_t g_check_btnCb_count = 0;
    
    g_check_btnCb_count++; 
    if(GPIO_GetPinStatus( GPIO_BUTTON_PORT, GPIO_BUTTON_PIN ) == 0 && (app_connection_flag == APP_BLE_ADV)){
        powerOffCount++;
        user_app_enable_led();  
        arch_force_active_mode();
        if(powerOffCount > 5){
            powerOffCount = 0;
            user_app_disable_led();
            if(app_adv_data_update_timer_used != EASY_TIMER_INVALID_TIMER)           
            {
                app_easy_timer_cancel(app_adv_data_update_timer_used);                  
                app_adv_data_update_timer_used = EASY_TIMER_INVALID_TIMER;      
            }               
            
            app_easy_gap_advertise_stop();
            user_app_disable_periphs();
            for(int i=0;i<10;i++)
                arch_restore_sleep_mode();          
            if(app_check_button_used != EASY_TIMER_INVALID_TIMER)
            {
                app_easy_timer_cancel(app_check_button_used);
                app_check_button_used = EASY_TIMER_INVALID_TIMER;
            }
        }// >5
        
    }else if(app_connection_flag == APP_BLE_ADV && (g_check_btnCb_count%5) == 0){ // blink led      
        powerOffCount = 0;
        if(user_app_get_led_status() == 1 || g_check_btnCb_count%3 == 0)
        {
            user_app_disable_led();
        }
        else
        {
            user_app_enable_led(); 
        }
        
    }else{
        powerOffCount = 0;
        arch_restore_sleep_mode();  // by aizj for debug
        
    }
     
    // poll now?
    if(app_check_button_used != EASY_TIMER_INVALID_TIMER)
    {
        app_check_button_used = app_easy_timer(APP_WAKEUP_LED_CTRL_TIMER_DELAY,app_check_button_cb);    
    }
    // poll btn calbrate
    if(GPIO_GetPinStatus( GPIO_BUTTON_CALI_PORT, GPIO_BUTTON_CALI_PIN ) == 0){
        //arch_force_active_mode();    
        cali_param_update_used = app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY_CALI,cali_param_update_cb); 

    }  
    
}
/*
static void app_check_button_cb(void)
{
    static uint16_t powerOffCount = 0;  
    static uint16_t g_check_btnCb_count = 0;
    
        g_check_btnCb_count++; 
        if(GPIO_GetPinStatus( GPIO_BUTTON_PORT, GPIO_BUTTON_PIN ) == 0 && (app_connection_flag == APP_BLE_ADV))
        {
            powerOffCount++;
            user_app_enable_led();  
            arch_force_active_mode();
            if(powerOffCount > 5)
            {
                powerOffCount = 0;
                user_app_disable_led();
                if(app_adv_data_update_timer_used != EASY_TIMER_INVALID_TIMER)          // lewis add
                {
                    app_easy_timer_cancel(app_adv_data_update_timer_used);                  
                    app_adv_data_update_timer_used = EASY_TIMER_INVALID_TIMER;      
                }               
                
                app_easy_gap_advertise_stop();
                user_app_disable_periphs();
                for(int i=0;i<10;i++)
                    arch_restore_sleep_mode();          
                if(app_check_button_used != EASY_TIMER_INVALID_TIMER)
                {
                    app_easy_timer_cancel(app_check_button_used);
                    app_check_button_used = EASY_TIMER_INVALID_TIMER;
                }
            }
            else
            {
                if(user_config_data.valid != 0x01)
                {
                    if(user_config_data.flags != 0x02)
                    {
                        cali_param_update_used = app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY,cali_param_update_cb);
                    }
                }
            }
        }
        else if(app_connection_flag == APP_BLE_ADV && (g_check_btnCb_count%5) == 0)
        { // blink led      
            powerOffCount = 0;
            if(user_config_data.flags == 0x01)
            {
                if(user_app_get_led_status() == 1 || g_check_btnCb_count%3 == 0)
                {
                    user_app_disable_led();
                }
                else
                {
                    user_app_enable_led(); 
                }
            }
            
        }
        else
        {
            powerOffCount = 0;
            arch_restore_sleep_mode();
        }
        // poll now?
        if(app_check_button_used != EASY_TIMER_INVALID_TIMER)
        {
            app_check_button_used = app_easy_timer(APP_WAKEUP_LED_CTRL_TIMER_DELAY,app_check_button_cb);    
        }
        
    
}
*/
/**
 ****************************************************************************************
 * @brief Button press callback function. Registered in WKUPCT driver.
 * @return void
 ****************************************************************************************
 */
static void app_button_press_cb(void)
{
    if (GetBits16(SYS_STAT_REG, PER_IS_DOWN))
    {
        periph_init();
    }

    if (arch_ble_ext_wakeup_get())
    {
        arch_set_sleep_mode(app_default_sleep_mode);
        arch_ble_force_wakeup();
        arch_ble_ext_wakeup_off();
        app_easy_wakeup();
        wakeup_led_ctrl_used = app_easy_timer(APP_WAKEUP_LED_CTRL_TIMER_DELAY,app_wakeup_led_ctrl_cb);
    }
}

/**
 ****************************************************************************************
 * @brief Sets button as wakeup trigger
 * @return void
 ****************************************************************************************
*/
static void app_button_enable(void)
{
    app_easy_wakeup_set(app_wakeup_cb);
    wkupct_register_callback(app_button_press_cb);
//    if (GPIO_GetPinStatus(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN))
        {
    wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN), // select pin (GPIO_BUTTON_PORT, GPIO_BUTTON_PIN)
                      WKUPCT_PIN_POLARITY(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, WKUPCT_PIN_POLARITY_LOW), // polarity low
                                          1, // 1 event
                                          40); // debouncing time = 0
        }
}

void user_app_init(void)
{
    app_prf_srv_perm_t svc_perm = SRV_PERM_ENABLE;
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;

    // Initialize Manufacturer Specific Data
    mnf_data_init();

    default_app_on_init();

    // Set custom service permission depending on the currently selected preset
    #if defined (USER_CFG_PAIR_METHOD_JUST_WORKS)
    svc_perm = SRV_PERM_UNAUTH;

    #elif defined (USER_CFG_PAIR_METHOD_PASSKEY)
    svc_perm = SRV_PERM_AUTH;

    #elif defined (USER_CFG_PAIR_METHOD_OOB)
    svc_perm = SRV_PERM_AUTH;

    #else
    svc_perm = SRV_PERM_ENABLE;
    #endif

    app_set_prf_srv_perm(TASK_CUSTS1, svc_perm);
#if (BLE_SPOTA_RECEIVER)
    app_set_prf_srv_perm(TASK_SPOTAR, svc_perm);
#endif

    // Fetch bond data from the external memory
    bond_db_init();
}
//void user_app_going_to_sleep(sleep_mode_t sleep_mode)
//{
//  user_app_disable_led();
//}

/**
 ****************************************************************************************
 * @brief Add an AD structure in the Advertising or Scan Response Data of the GAPM_START_ADVERTISE_CMD parameter struct
 * @param[in] cmd               GAPM_START_ADVERTISE_CMD parameter struct.
 * @param[in] ad_struct_data    AD structure buffer.
 * @param[in] ad_struct_len     AD structure length.
 * @return void.
 ****************************************************************************************
 */
static void app_add_ad_struct(struct gapm_start_advertise_cmd *cmd, void *ad_struct_data, uint8_t ad_struct_len)
{
    if ( (APP_ADV_DATA_MAX_SIZE - cmd->info.host.adv_data_len) >= ad_struct_len)
    {
        // Copy data
        memcpy(&cmd->info.host.adv_data[cmd->info.host.adv_data_len], ad_struct_data, ad_struct_len);

        // Update Advertising Data Length
        cmd->info.host.adv_data_len += ad_struct_len;
    }
    else if ( (APP_SCAN_RESP_DATA_MAX_SIZE - cmd->info.host.scan_rsp_data_len) >= ad_struct_len)
    {
        // Copy data
        memcpy(&cmd->info.host.scan_rsp_data[cmd->info.host.scan_rsp_data_len], ad_struct_data, ad_struct_len);

        // Update Scan Responce Data Length
        cmd->info.host.scan_rsp_data_len += ad_struct_len;
    }
    else
    {
        // Manufacturer Specific Data do not fit in either Advertising Data or Scan Response Data
        ASSERT_ERROR(0);
    }
}

void user_app_adv_start(void)
{
    // Schedule the next advertising data update
    app_connection_flag = APP_BLE_ADV;
    app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);

    struct gapm_start_advertise_cmd* cmd;
    cmd = app_easy_gap_undirected_advertise_get_active();

    // add manufacturer specific data dynamically
    mnf_data_update();
    app_add_ad_struct(cmd, &mnf_data, sizeof(struct mnf_specific_data_ad_structure));

#if defined (USER_CFG_PRIV_GEN_STATIC_RND)
    // Provide previously generated random address if saved
    if (stored_addr.addr_type == 0x01)
    {
        cmd->op.addr_src = GAPM_PROVIDED_RND_ADDR;
        cmd->op.addr = stored_addr.addr;
        cmd->op.renew_dur = 0;
    }
#endif

    // Enable deep sleep
//    arch_set_deep_sleep();                
    arch_set_extended_sleep();              // lewis add        

    app_easy_gap_undirected_advertise_start();
    
    app_check_button_used = app_easy_timer(APP_WAKEUP_LED_CTRL_TIMER_DELAY, app_check_button_cb);
}

#if (BLE_SPOTA_RECEIVER)
void on_spotar_status_change( const uint8_t spotar_event)
{
#if defined(__DA14583__) && (!SPOTAR_SPI_DISABLE)
    int8_t man_dev_id = 0;

    man_dev_id = spi_flash_enable(SPI_EN_GPIO_PORT, SPI_EN_GPIO_PIN);
    if (man_dev_id == SPI_FLASH_AUTO_DETECT_NOT_DETECTED)
    {
        // The device was not identified. The default parameters are used.
        // Alternatively, an error can be asserted here.
        spi_flash_init(SPI_FLASH_DEFAULT_SIZE, SPI_FLASH_DEFAULT_PAGE);
    }

    if( spotar_event == SPOTAR_END )
    {
        // Power down SPI Flash
        spi_flash_power_down();
    }
#endif
}
#endif // (BLE_SPOTA_RECEIVER)


void user_app_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param)
{
    app_connection_flag = APP_BLE_CONNECTED;
    app_connection_flag2 = 1;
    default_app_on_connection(connection_idx, param);
    if (app_env[connection_idx].conidx != GAP_INVALID_CONIDX)
    {
        app_connection_idx = connection_idx;

                if(app_adv_data_update_timer_used != EASY_TIMER_INVALID_TIMER)      // lewis add
                {
                    // Stop the advertising data update timer
                    app_easy_timer_cancel(app_adv_data_update_timer_used);
                    app_adv_data_update_timer_used = EASY_TIMER_INVALID_TIMER;
                }           
                
                if(wakeup_led_ctrl_used != EASY_TIMER_INVALID_TIMER)
                {
                    app_easy_timer_cancel(wakeup_led_ctrl_used);
                    wakeup_led_ctrl_used = EASY_TIMER_INVALID_TIMER; 
                }               
                

        // Check if the parameters of the established connection are the preferred ones.
        // If not then schedule a connection parameter update request.
        if ((param->con_interval < user_connection_param_conf.intv_min) ||
            (param->con_interval > user_connection_param_conf.intv_max) ||
            (param->con_latency != user_connection_param_conf.latency) ||
            (param->sup_to != user_connection_param_conf.time_out))
        {
            // Connection params are not these that we expect
            app_param_update_request_timer_used = app_easy_timer(APP_PARAM_UPDATE_REQUEST_TO, param_update_request_timer_cb);
        }
        user_app_enable_periphs();
        arch_set_extended_sleep();
        user_app_disable_led();
        
    }
}

void user_app_disconnect(struct gapc_disconnect_ind const *param)
{
    
        app_connection_flag2 = 0;
    
    // Cancel the parameter update request timer
    if (app_param_update_request_timer_used != EASY_TIMER_INVALID_TIMER)
    {
        app_easy_timer_cancel(app_param_update_request_timer_used);
        app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
    }

        if(app_check_button_used != EASY_TIMER_INVALID_TIMER)                           // lewis add
        {
            app_easy_timer_cancel(app_check_button_used);
            app_check_button_used = EASY_TIMER_INVALID_TIMER;
        }           
        
    uint8_t state = ke_state_get(TASK_APP);

    if ((state == APP_SECURITY) ||
        (state == APP_CONNECTED) ||
        (state == APP_PARAM_UPD))
    {
        // Restart Advertising
        user_app_disable_periphs();
        user_app_adv_start();
    }
    else
    {
        // We are not in a Connected State
        ASSERT_ERR(0);
    }
    
#if (BLE_SPOTA_RECEIVER)
    // Issue a platform reset when it is requested by the spotar procedure
    if (spota_state.reboot_requested)
    {
        // Reboot request will be served
        spota_state.reboot_requested = 0;

        // Platform reset
        platform_reset(RESET_AFTER_SPOTA_UPDATE);
    }
#endif
}

#if (BLE_APP_SEC)
void user_app_on_tk_exch_nomitm(uint8_t connection_idx,
                                    struct gapc_bond_req_ind const * param)
{
#if defined (USER_CFG_PAIR_METHOD_JUST_WORKS) || defined (USER_CFG_PAIR_METHOD_PASSKEY) || defined (USER_CFG_PAIR_METHOD_OOB)
    if (param->data.tk_type == GAP_TK_DISPLAY)
    {
        // By default we send hardcodded passkey
        uint32_t passkey = APP_SECURITY_MITM_PASSKEY_VAL;

        app_easy_security_tk_exch(connection_idx, (uint8_t*) &passkey, 4);
    }
    else if (param->data.tk_type == GAP_TK_OOB)
    {
        // By default we send hardcodded oob data
        uint8_t oob_tk[KEY_LEN] = APP_SECURITY_OOB_TK_VAL;

        app_easy_security_tk_exch(connection_idx, (uint8_t*) oob_tk, KEY_LEN);
    }
#else
    default_app_on_tk_exch_nomitm(connection_idx, param);
#endif
}

void user_app_on_encrypt_req_ind(uint8_t connection_idx,
                                    struct gapc_encrypt_req_ind const *param)
{
#if defined (USER_CFG_PAIR_METHOD_JUST_WORKS) || defined (USER_CFG_PAIR_METHOD_PASSKEY) || defined (USER_CFG_PAIR_METHOD_OOB)
    const struct bond_db_data *pbd;

    // Retrieve bond data and put it inside the app_sec_env
    pbd = bond_db_lookup_by_ediv(&param->rand_nb, param->ediv);
    if (pbd)
    {
        app_sec_env[connection_idx].auth = pbd->auth;                                               /// Authentication
        memcpy(&app_sec_env[connection_idx].ltk, &pbd->ltk.ltk, sizeof(struct gap_sec_key));        /// Long Term Key
        app_sec_env[connection_idx].ediv = pbd->ltk.ediv;                                           /// Encryption Diversifier
        memcpy(&app_sec_env[connection_idx].rand_nb, &pbd->ltk.randnb, RAND_NB_LEN);                /// Random Number
        app_sec_env[connection_idx].key_size = pbd->ltk.key_size;                                   /// Encryption key size (7 to 16)
        memcpy(&app_sec_env[connection_idx].irk, &pbd->irk, sizeof(struct gapc_irk));               /// IRK
    }
#endif
    // Validate bond data in app_sec_env
    default_app_on_encrypt_req_ind(connection_idx, param);
}

void user_app_on_pairing_succeded(void)
{
    struct bond_db_data bd;

    if (app_sec_env[app_connection_idx].auth & GAP_AUTH_BOND)
    {
        memset(&bd, 0, sizeof(bd));

        // store bond data
        bd.valid = 0xAA;

        memcpy(&bd.ltk.ltk, &app_sec_env[app_connection_idx].ltk, sizeof(struct gap_sec_key));  /// Long Term Key
        bd.ltk.ediv = app_sec_env[app_connection_idx].ediv;                                     /// Encryption Diversifier
        memcpy(&bd.ltk.randnb, &app_sec_env[app_connection_idx].rand_nb, RAND_NB_LEN);          /// Random Number
        bd.ltk.key_size = app_sec_env[app_connection_idx].key_size;                             /// Encryption key size (7 to 16)

        // Check bits for resolvable address type
        if ((app_sec_env[app_connection_idx].peer_addr_type == ADDR_RAND) &&
            (app_sec_env[app_connection_idx].peer_addr.addr[0] && GAP_RSLV_ADDR))
        {
            memcpy(&bd.irk, &app_sec_env[app_connection_idx].irk, sizeof(struct gapc_irk));     /// IRK
            bd.flags |= BOND_DB_ENTRY_IRK_PRESENT;
        }

        // Remote address
        bd.bdaddr.addr_type = app_sec_env[app_connection_idx].peer_addr_type;
        memcpy(&bd.bdaddr.addr, &app_sec_env[app_connection_idx].peer_addr, sizeof(struct bd_addr));

        bd.auth = app_sec_env[app_connection_idx].auth;                                         // authentication level

        bond_db_store(&bd);
    }
}
#endif // BLE_APP_SEC

void user_catch_rest_hndl(ke_msg_id_t const msgid,
                        void const *param,
                        ke_task_id_t const dest_id,
                        ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case CUSTS1_VAL_WRITE_IND:
        {
            struct custs1_val_write_ind const *msg_param = (struct custs1_val_write_ind const *)(param);

            switch (msg_param->handle)
            {
                case CUST1_IDX_CONTROL_POINT_VAL:
                   user_custs1_ctrl_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                   break;

                case CUST1_IDX_LED_STATE_VAL:
                    user_custs1_led_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case CUST1_IDX_ADC_VAL_1_NTF_CFG:
                    user_custs1_adc_val_1_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case CUST1_IDX_BUTTON_STATE_NTF_CFG:
                    user_custs1_button_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;
/*
                case CUST1_IDX_INDICATEABLE_IND_CFG:
                    user_custs1_long_val_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case CUST1_IDX_LONG_VALUE_NTF_CFG:
                    user_custs1_long_val_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;
*/
                case CUST1_IDX_LONG_VALUE_VAL:
                    user_custs1_long_val_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                default:
                    break;
            }
        } break;

        case CUSTS1_VAL_NTF_CFM:
        {
            struct custs1_val_ntf_cfm const *msg_param = (struct custs1_val_ntf_cfm const *)(param);

            switch (msg_param->handle)
            {
                case CUST1_IDX_ADC_VAL_1_VAL:
                    break;

                case CUST1_IDX_BUTTON_STATE_VAL:
                    break;

                case CUST1_IDX_LONG_VALUE_VAL:
                    break;

                default:
                    break;
            }
        } break;

        /*case CUSTS1_VAL_IND_CFM:
        {
            struct custs1_val_ind_cfm const *msg_param = (struct custs1_val_ind_cfm const *)(param);

            switch (msg_param->handle)
            {
                //case CUST1_IDX_INDICATEABLE_VAL:
                 //   break;

                default:
                    break;
             }
         } break;*/

        case GAPC_PARAM_UPDATED_IND:
        {
            // Cast the void pointer to the appropriate message structure
            struct gapc_param_updated_ind const *msg_param = (struct gapc_param_updated_ind const *)(param);

            // Check if updated Conn Params filled to preffered ones
            if ((msg_param->con_interval >= user_connection_param_conf.intv_min) &&
                (msg_param->con_interval <= user_connection_param_conf.intv_max) &&
                (msg_param->con_latency == user_connection_param_conf.latency) &&
                (msg_param->sup_to == user_connection_param_conf.time_out))
            {
            }
        } break;

        case GAPM_DEV_BDADDR_IND:
        {
            struct gapm_dev_bdaddr_ind const *msg_param = (struct gapm_dev_bdaddr_ind const *)(param);

            stored_addr = msg_param->addr;
        }

        default:
            break;
    }
}

void user_app_adv_undirect_complete(const uint8_t status)
{
    // Disable wakeup for BLE and timer events. Only external (GPIO) wakeup events can wakeup processor.
    app_connection_flag = APP_BLE_TIMEOUT_IDLE;
    if (status == GAP_ERR_CANCELED)
    {
        user_app_disable_periphs();
        user_app_disable_led();
        arch_ble_ext_wakeup_on();

        if(app_check_button_used != EASY_TIMER_INVALID_TIMER)                           // aizj add
        {
            app_easy_timer_cancel(app_check_button_used);
            app_check_button_used = EASY_TIMER_INVALID_TIMER;
        }   
        // Configure wakeup button
        app_button_enable();
    }
}

/// @} APP
