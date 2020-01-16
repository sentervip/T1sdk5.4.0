/**
 ****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief Peripherals setup and initialization.
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

#include "rwip_config.h"             // SW configuration
#include "user_periph_setup.h"       // peripheral configuration
#include "global_io.h"
#include "gpio.h"
#include "uart.h"                    // UART initialization

extern uint8_t app_connection_flag;

/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 *
 * @return void
 ****************************************************************************************
 */

#ifdef CFG_DEVELOPMENT_DEBUG

void GPIO_reservations(void)
{
	RESERVE_GPIO(A0,GPIO_PORT_0, GPIO_PIN_1, PID_ADC );
	RESERVE_GPIO(A1,GPIO_PORT_0, GPIO_PIN_2, PID_ADC );
	
	RESERVE_GPIO(A2,GPIO_POWER_PORT, GPIO_POWER_PIN, PID_GPIO);
	//GPIO_SetActive(GPIO_POWER_PORT, GPIO_POWER_PIN);//power on
	
    RESERVE_GPIO(A3,GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, PID_GPIO);
    RESERVE_GPIO(A4,GPIO_PWM_PORT, GPIO_PWM_PIN, PID_PWM2);
    RESERVE_GPIO(A5,GPIO_LED_PORT, GPIO_LED_PIN, PID_GPIO);
    
    // SPI FLASH
    RESERVE_GPIO(A6,SPI_EN_GPIO_PORT,  SPI_EN_GPIO_PIN, PID_SPI_EN);
    RESERVE_GPIO(A7,SPI_CLK_GPIO_PORT, SPI_CLK_GPIO_PIN, PID_SPI_CLK);
    RESERVE_GPIO(A8,SPI_DO_GPIO_PORT,  SPI_DO_GPIO_PIN, PID_SPI_DO);
    RESERVE_GPIO(A9,SPI_DI_GPIO_PORT,  SPI_DI_GPIO_PIN,  PID_SPI_DI);
}
#endif // CFG_DEVELOPMENT_DEBUG

void set_pad_functions(void)        // set gpio port function mode
{
#ifdef CFG_PRINTF_UART2
    GPIO_ConfigurePin(GPIO_UART2_TX_PORT, GPIO_UART2_TX_PIN, OUTPUT, PID_UART2_TX, false);
    GPIO_ConfigurePin(GPIO_UART2_RX_PORT, GPIO_UART2_RX_PIN, INPUT, PID_UART2_RX, false);
#endif
	GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_0, INPUT, PID_ADC, false );
	GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_2, INPUT, PID_ADC, false );
	
	if(app_connection_flag == APP_BLE_CONNECTED)
		GPIO_ConfigurePin(GPIO_POWER_PORT, GPIO_POWER_PIN, OUTPUT, PID_GPIO, true);
	else
		GPIO_ConfigurePin(GPIO_POWER_PORT, GPIO_POWER_PIN, OUTPUT, PID_GPIO, false);		
//	GPIO_SetActive(GPIO_POWER_PORT, GPIO_POWER_PIN);//power on
	
	
    GPIO_ConfigurePin(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, INPUT_PULLUP, PID_GPIO, false);
    GPIO_ConfigurePin(GPIO_PWM_PORT, GPIO_PWM_PIN, OUTPUT, PID_PWM2, false);
    GPIO_ConfigurePin(GPIO_LED_PORT, GPIO_LED_PIN, OUTPUT, PID_GPIO, false);
    
    // I2C EEPROM
/*    GPIO_ConfigurePin(I2C_GPIO_PORT, I2C_SCL_PIN, INPUT, PID_I2C_SCL, false);
    GPIO_ConfigurePin(I2C_GPIO_PORT, I2C_SDA_PIN, INPUT, PID_I2C_SDA, false);
*/
    // SPI FLASH
    GPIO_ConfigurePin(SPI_EN_GPIO_PORT,  SPI_EN_GPIO_PIN,  OUTPUT, PID_SPI_EN,  true);
    GPIO_ConfigurePin(SPI_CLK_GPIO_PORT, SPI_CLK_GPIO_PIN, OUTPUT, PID_SPI_CLK, false);
    GPIO_ConfigurePin(SPI_DO_GPIO_PORT,  SPI_DO_GPIO_PIN,  OUTPUT, PID_SPI_DO,  false);
    GPIO_ConfigurePin(SPI_DI_GPIO_PORT,  SPI_DI_GPIO_PIN,  INPUT,  PID_SPI_DI,  false);
}

void periph_init(void)
{
    // Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));

    SetBits16(CLK_16M_REG, XTAL16_BIAS_SH_ENABLE, 1);

    // Rom patch
    patch_func();

    // Init pads
    set_pad_functions();

    // (Re)Initialize peripherals
    // i.e.
    // uart_init(UART_BAUDRATE_115K2, 3);

#ifdef CFG_PRINTF_UART2
    SetBits16(CLK_PER_REG, UART2_ENABLE, 1);
    uart2_init(UART_BAUDRATE_115K2, 3);
#endif

    // Enable the pads
    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}
