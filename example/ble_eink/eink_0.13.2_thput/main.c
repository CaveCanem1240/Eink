/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Motion-Powered EinkTag"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                3200                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(300, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(750, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

/***********************************************************************************************************
	* include
	*/
#include "ble_err.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "app_button.h"
#include "nrf_delay.h"

#include "eink\e154.h"
#include "eink\e154font.h"

#include "nrf_nvmc.h"

#include "nrf_sdh_soc.h"

#include "fds.h"

#include "GUI\GUI_Paint.h"
#include "Qrcode\qrcodegen.h"

#include "nrf_drv_saadc.h"
#include "nrfx_saadc.h"

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define gImage_num_CONCAT_2_(p1, p2)      p1 ## p2
#define pixel_size(p1)      DOT_PIXEL_ ## p1 ## X ## p1
int i,flag1,flag2;

#define ble_debug 0
#define key_LOG 0
#define key_LED 1
#define key_ADC 1
#define key_GUI 0
#define key_Qrcode 0 
#define key_ADC_LOG 0
/***********************************************************************************************************
	* APPTIMER DEFINITIONS
	*/
#define LED_TOGGLE_INTERVAL         APP_TIMER_TICKS(500)  //ms              // sending interval setting, unit ms 
#define BUSY_TOGGLE_INTERVAL         APP_TIMER_TICKS(100)  //ms              // sending interval setting, unit ms 
#define EPD_INIT_BUSY_TOGGLE_INTERVAL         APP_TIMER_TICKS(20)  //ms              // sending interval setting, unit ms 

APP_TIMER_DEF(m_adc_timer_id);
APP_TIMER_DEF(m_busy_timer_id); 
APP_TIMER_DEF(m_epd_rst_busy_timer_id); 

static bool eink_busy = 0;

static bool Init_Ready = 0;
static bool Init_waiting = 0;
static int Init_Process = 0;

/***********************************************************************************************************
	* SAADC DEFINITIONS
	*/

#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                 /**< Size of timer operation queues. */

#define ADC_SAMPLE_INTERVAL         APP_TIMER_TICKS(500)  //ms              // sending interval setting, unit ms 

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                 /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                   /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  1000                    /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                /**< Maximum digital value for 10-bit ADC conversion. */

#define SAMPLES_IN_BUFFER               1

bool adcflag = 0;
bool finished_flag = 0;
bool Energy_flag = 0;
bool bleflag = 0;
static void adcCallbackFunc(nrf_drv_saadc_evt_t const *pEvent);
static nrf_saadc_value_t s_bufferPool[SAMPLES_IN_BUFFER];

uint32_t  addr;
uint32_t *  pAddr;
uint32_t  offset;

static uint16_t adc_threshold;
bool sysoff_flag = 1;


/***********************************************************************************************************
	* FDS DEFINITIONS
	*/
#define FILE_ID         0x0001  /* The ID of the file to write the records into. */
//#define RECORD_KEY_1    0x0001  /* A key for the first record. */
#define RECORD_KEY_LAST_PIC    0x0001  /* A key for the last picture record. */
#define RECORD_KEY_BLE_STR    0x0002  /* A key for the last picture record. */
static fds_record_t        new_record;
static fds_record_desc_t   record_desc;
static fds_find_token_t    ftok;

/***********************************************************************************************************
	* EINK DEFINITIONS
	*/
static unsigned char base_map[5000];

static int ble_data_length;
static char ble_data[200];
UBYTE *BlackImage;


/***********************************************************************************************************
	* QRCODE DEFINITIONS
	*/
static int qrsize = 0;
static int qrsize_display = 1;
uint8_t qrcode[qrcodegen_BUFFER_LEN_MAX];
uint8_t tempBuffer[qrcodegen_BUFFER_LEN_MAX];



/***********************************************************************************************************
	* PUBLIC FUNCTIONS
	*/


/***********************************************************************************************************
	* ADC
	*/

void ADC_Init(void)
{
    ret_code_t err_code;
    //
    err_code = nrf_drv_saadc_init(NULL, adcCallbackFunc);
    APP_ERROR_CHECK(err_code);
    //
    nrf_saadc_channel_config_t channelConfig = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    //
    err_code = nrf_drv_saadc_channel_init(0, &channelConfig);
    APP_ERROR_CHECK(err_code);
    //
    err_code = nrf_drv_saadc_buffer_convert(s_bufferPool, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

void ADC_Read(void)
{
		
    ret_code_t err_code;
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
		
}

void ADC_Enable(void)
{
    ADC_Init();
}

void ADC_Disable(void)
{
    nrfx_saadc_uninit();
}

void ADC_start(void)
{
    app_timer_start(m_adc_timer_id, ADC_SAMPLE_INTERVAL, NULL);
}

static void adcCallbackFunc(nrf_drv_saadc_evt_t const *pEvent)
{
		//bsp_board_led_invert(BSP_BOARD_LED_1);	
    if(pEvent->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adcResult;
        ret_code_t errCode;
        errCode = nrf_drv_saadc_buffer_convert(pEvent->data.done.p_buffer, SAMPLES_IN_BUFFER);                  
        APP_ERROR_CHECK(errCode);
        adcResult = pEvent->data.done.p_buffer[0];
				#if key_LOG
					#if key_ADC_LOG
						printf("\r\ADC: %d\r\n", adcResult);
					#endif
				#endif
				if(adcflag == 0){
					if(adcResult>adc_threshold){
						//bsp_board_led_invert(BSP_BOARD_LED_1);	
						#if key_LED
							#if ble_debug
								bsp_board_led_on(BSP_BOARD_LED_2);	
							#endif
						#endif
						adcflag = 1;
					}
				}
				else{
					if(adcResult<180){
						#if key_LED
							#if ble_debug
								bsp_board_led_off(BSP_BOARD_LED_2);
							#endif
						#endif
						adcflag = 0;
						finished_flag = 0;
					}
				}
				ADC_Disable();
    }
}

static void adc_threshold_update(void)
{
		uint8_t value = pAddr[0];
		switch(value){
			case 0x00:
				adc_threshold = 370;//600;
				break;
			case 0x1A:
				adc_threshold = 370;//1600;
				break;
			default:
				adc_threshold = 275;//1250;
				break;
		}
}

/***********************************************************************************************************
	* APPLICATION TIMER
	*/

/**@brief 
 * function when timer overflowed
 * @details
 *
 * @param[in]  
 */

static void adc_timer_timeout_handler(void *p_context)
{		
		ADC_Enable();
		#if key_LED
			#if ble_debug
				bsp_board_led_invert(BSP_BOARD_LED_3);	
			#endif
		#endif
    UNUSED_PARAMETER(p_context);
    
    ADC_Read();
		
}

static void busy_timer_timeout_handler(void *p_context)
{
		if(nrf_gpio_pin_read(27)==0){
				nrf_gpio_pin_clear(4);
				eink_busy = 0;
				SPI_EP_unInit();
				
				app_timer_stop(m_busy_timer_id);
				
		}
}

static void epd_rst_busy_timer_timeout_handler(void *p_context)
{
		
		if(nrf_gpio_pin_read(27)==0){
				//epd_rst_eink_busy = 0;
				app_timer_stop(m_epd_rst_busy_timer_id);
				switch(Init_Process){
					case 0:
						Init_Process = 1;
						break;
					case 1:
						Init_Process = 2;
						break;
					default:
						break;
				}
				Init_waiting = 0;
		}
}

static void apptimer_create(void)
{		
		ret_code_t err_code;
		
		err_code = app_timer_create(&m_adc_timer_id, APP_TIMER_MODE_REPEATED, adc_timer_timeout_handler);
		APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_busy_timer_id, APP_TIMER_MODE_REPEATED, busy_timer_timeout_handler);
		APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_epd_rst_busy_timer_id, APP_TIMER_MODE_REPEATED, epd_rst_busy_timer_timeout_handler);
		APP_ERROR_CHECK(err_code);
}

/***********************************************************************************************************
	* EINK FUNCTION
	*/
static void EPD_Init(void)
{
		
		if(Init_Ready == 0){
			//Init_waiting = 1;
			//bsp_board_led_on(BSP_BOARD_LED_0);
			if(Init_Process == 0 & Init_waiting == 0){
				SPI_EP_Init();
				nrf_gpio_pin_set(4);
				Init_waiting = 1;
				EPD_HW_Init1();
				#if key_LED
					//bsp_board_led_on(BSP_BOARD_LED_1);
				#endif
				app_timer_start(m_epd_rst_busy_timer_id, EPD_INIT_BUSY_TOGGLE_INTERVAL, NULL);
			}
			if(Init_Process == 1 & Init_waiting == 0){
				EPD_HW_Init2();
				#if key_LED
					//bsp_board_led_on(BSP_BOARD_LED_2);
				#endif
				Init_waiting = 1;
				app_timer_start(m_epd_rst_busy_timer_id, EPD_INIT_BUSY_TOGGLE_INTERVAL, NULL);
			}
			if(Init_Process == 2 & Init_waiting == 0){
				Init_waiting = 1;
				#if key_LED
					//bsp_board_led_on(BSP_BOARD_LED_3);
				#endif
				EPD_HW_Init3();
				Init_Ready = 1;
				Init_waiting = 0;
				//app_timer_start(m_epd_rst_busy_timer_id, EPD_INIT_BUSY_TOGGLE_INTERVAL, NULL);
			}
		}
}

static void gen_basemap(const unsigned char * datas)
{
		for(i=0;i<5000;i++)
		{
				base_map[i] = *datas;
				datas++;
		} 
}

static void update_basemap_ramvalue(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_COLUMN,unsigned int PART_LINE)
{
		
		unsigned int addr;
		

		for(i=0;i<PART_COLUMN*PART_LINE/8;i++)
		{
				addr = (200-y_start+(i*8)/PART_COLUMN)*25 + (x_start/8) + i%5;
				//addr = (y_start-PART_LINE+(i*8)/PART_COLUMN)*25 + (x_start/8) + i%5;
				base_map[addr] = *datas;
				datas++;
		} 
}

static int update_basemap()
{
		uint8_t value = pAddr[0];
		
		if(value==0x01){return 0;}
		update_basemap_ramvalue(0,200,gImage_sht11,40,40);
		if(value==0x02){return 0;}
		update_basemap_ramvalue(0,160,gImage_sht12,40,40);
		if(value==0x03){return 0;}
		update_basemap_ramvalue(0,120,gImage_sht13,40,40);
		if(value==0x04){return 0;}
		update_basemap_ramvalue(0,80,gImage_sht14,40,40);
		if(value==0x05){return 0;}
		update_basemap_ramvalue(0,40,gImage_sht15,40,40);
		if(value==0x06){return 0;}
		update_basemap_ramvalue(40,200,gImage_sht21,40,40);
		if(value==0x07){return 0;}
		update_basemap_ramvalue(40,160,gImage_sht22,40,40);
		if(value==0x08){return 0;}
		update_basemap_ramvalue(40,120,gImage_sht23,40,40);
		if(value==0x09){return 0;}
		update_basemap_ramvalue(40,80,gImage_sht24,40,40);
		if(value==0x0A){return 0;}
		update_basemap_ramvalue(40,40,gImage_sht25,40,40);
		if(value==0x0B){return 0;}
		update_basemap_ramvalue(80,200,gImage_sht31,40,40);
		if(value==0x0C){return 0;}
		update_basemap_ramvalue(80,160,gImage_sht32,40,40);
		if(value==0x0D){return 0;}
		update_basemap_ramvalue(80,120,gImage_sht33,40,40);
		if(value==0x0E){return 0;}
		update_basemap_ramvalue(80,80,gImage_sht34,40,40);
		if(value==0x0F){return 0;}
		update_basemap_ramvalue(80,40,gImage_sht35,40,40);
		if(value==0x10){return 0;}
		update_basemap_ramvalue(120,200,gImage_sht41,40,40);
		if(value==0x11){return 0;}
		update_basemap_ramvalue(120,160,gImage_sht42,40,40);
		if(value==0x12){return 0;}
		update_basemap_ramvalue(120,120,gImage_sht43,40,40);
		if(value==0x13){return 0;}
		update_basemap_ramvalue(120,80,gImage_sht44,40,40);
		if(value==0x14){return 0;}
		update_basemap_ramvalue(120,40,gImage_sht45,40,40);
		if(value==0x15){return 0;}
		update_basemap_ramvalue(160,200,gImage_sht51,40,40);
		if(value==0x16){return 0;}
		update_basemap_ramvalue(160,160,gImage_sht52,40,40);
		if(value==0x17){return 0;}
		update_basemap_ramvalue(160,120,gImage_sht53,40,40);
		if(value==0x18){return 0;}
		update_basemap_ramvalue(160,80,gImage_sht54,40,40);
		return 0;
}

static void check_pwroff(uint8_t value)
{
		switch(value){
				case 0x00:
						break;
				case 0x01:
						EPD_SetRAMValue(gImage_white_full);
						break;
				case 0x02:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x03:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x04:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x05:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x06:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x07:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x08:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x09:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x0A:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x0B:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x0C:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x0D:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x0E:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x0F:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x10:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x11:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x12:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x13:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x14:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x15:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x16:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x17:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x18:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x19:
						gen_basemap(gImage_white_full);
						update_basemap();
						EPD_SetRAMValue(base_map);
						break;
				case 0x1A:
						break;
		}
}

static void figure_refresh(uint8_t value)
{
		switch(value){
				case 0x00:
						EPD_SetRAMValue_BaseMap(gImage_white_full);
						break;
				case 0x01:
						EPD_Dis_Part(0,200,gImage_sht11,40,40);  //11
						break;
				case 0x02:
						EPD_Dis_Part(0,160,gImage_sht12,40,40);  //12
						break;
				case 0x03:
						EPD_Dis_Part(0,120,gImage_sht13,40,40);  //13
						break;
				case 0x04:
						EPD_Dis_Part(0,80,gImage_sht14,40,40);  //14
						break;
				case 0x05:
						EPD_Dis_Part(0,40,gImage_sht15,40,40);  //15
						break;
				case 0x06:
						EPD_Dis_Part(40,200,gImage_sht21,40,40);  //21
						break;
				case 0x07:
						EPD_Dis_Part(40,160,gImage_sht22,40,40);  //22
						break;
				case 0x08:
						EPD_Dis_Part(40,120,gImage_sht23,40,40);  //23
						break;
				case 0x09:
						EPD_Dis_Part(40,80,gImage_sht24,40,40);  //24
						break;
				case 0x0A:
						EPD_Dis_Part(40,40,gImage_sht25,40,40);  //25
						break;
				case 0x0B:
						EPD_Dis_Part(80,200,gImage_sht31,40,40);  //31
						break;
				case 0x0C:
						EPD_Dis_Part(80,160,gImage_sht32,40,40);  //32
						break;
				case 0x0D:
						EPD_Dis_Part(80,120,gImage_sht33,40,40);  //33
						break;
				case 0x0E:
						EPD_Dis_Part(80,80,gImage_sht34,40,40);  //34
						break;
				case 0x0F:
						EPD_Dis_Part(80,40,gImage_sht35,40,40);  //35
						break;
				case 0x10:
						EPD_Dis_Part(120,200,gImage_sht41,40,40);  //41
						break;
				case 0x11:
						EPD_Dis_Part(120,160,gImage_sht42,40,40);  //42
						break;
				case 0x12:
						EPD_Dis_Part(120,120,gImage_sht43,40,40);  //43
						break;
				case 0x13:
						EPD_Dis_Part(120,80,gImage_sht44,40,40);  //44
						break;
				case 0x14:
						EPD_Dis_Part(120,40,gImage_sht45,40,40);  //45
						break;
				case 0x15:
						EPD_Dis_Part(160,200,gImage_sht51,40,40);  //51
						break;
				case 0x16:
						EPD_Dis_Part(160,160,gImage_sht52,40,40);  //52
						break;
				case 0x17:
						EPD_Dis_Part(160,120,gImage_sht53,40,40);  //53
						break;
				case 0x18:
						EPD_Dis_Part(160,80,gImage_sht54,40,40);  //54
						break;
				case 0x19:
						EPD_Dis_Part(160,40,gImage_sht55,40,40);  //55
						break;
				case 0x1A:
						EPD_SetRAMValue_BaseMap(gImage_metal);
						break;
		}
}

/***********************************************************************************************************
	* FDS
	*/
/**@brief Function for initializing FDS handler.
 */
static void fds_evt_handler(fds_evt_t const * p_fds_evt)
{
		ret_code_t err_code;
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result == NRF_SUCCESS)
            {
								fds_flash_record_t  flash_record;
								fds_record_desc_t   record_desc;
								fds_find_token_t    ftok;
								
								memset(&ftok, 0x00, sizeof(fds_find_token_t));
							
								if (fds_record_find(FILE_ID, RECORD_KEY_LAST_PIC, &record_desc, &ftok) == NRF_SUCCESS){
										err_code = fds_record_open(&record_desc, &flash_record);
										APP_ERROR_CHECK(err_code);
											
											
										pAddr = (uint32_t*)(flash_record.p_data);
											
										err_code = fds_record_close(&record_desc);
										APP_ERROR_CHECK(err_code);
										
										
								}
								else{
										fds_record_t        init_record;
										fds_record_desc_t   init_record_desc;
												
										// Set up record.
										init_record.file_id           = FILE_ID;
										init_record.key               = RECORD_KEY_LAST_PIC;
										init_record.data.p_data       = &m_figure0;
										init_record.data.length_words = 1;
												
										err_code = fds_record_write(&init_record_desc, &init_record);
										APP_ERROR_CHECK(err_code);
										pAddr = (uint32_t*)(init_record.data.p_data);
								}
								
								adc_threshold_update();
            }
            break;
				case FDS_EVT_UPDATE:
            if (p_fds_evt->result == NRF_SUCCESS)
            {
								
            }
            break;
						
        default:
            break;
    }
}



static void FDS_init(void)
{
		ret_code_t err_code;
		
		err_code = fds_register(fds_evt_handler);
		APP_ERROR_CHECK(err_code);
		
		err_code = fds_init();
		APP_ERROR_CHECK(err_code);
}

static void find_last_record(void)
{
		new_record.file_id           = FILE_ID;
		new_record.key               = RECORD_KEY_LAST_PIC;
		new_record.data.length_words = 1;
				
		memset(&ftok, 0x00, sizeof(fds_find_token_t));
		fds_record_find(FILE_ID, RECORD_KEY_LAST_PIC, &record_desc, &ftok);
			
}
static void record_update(uint8_t value)
{
		ret_code_t err_code;
		switch(value){
				case 0x00:
						new_record.data.p_data = &m_figure1;
						break;
				case 0x01:
						new_record.data.p_data = &m_figure2;
						break;
				case 0x02:
						new_record.data.p_data = &m_figure3;
						break;
				case 0x03:
						new_record.data.p_data = &m_figure4;
						break;
				case 0x04:
						new_record.data.p_data = &m_figure5;
						break;
				case 0x05:
						new_record.data.p_data = &m_figure6;
						break;
				case 0x06:
						new_record.data.p_data = &m_figure7;
						break;
				case 0x07:
						new_record.data.p_data = &m_figure8;
						break;
				case 0x08:
						new_record.data.p_data = &m_figure9;
						break;
				case 0x09:
						new_record.data.p_data = &m_figure10;
						break;
				case 0x0A:
						new_record.data.p_data = &m_figure11;
						break;
				case 0x0B:
						new_record.data.p_data = &m_figure12;
						break;
				case 0x0C:
						new_record.data.p_data = &m_figure13;
						break;
				case 0x0D:
						new_record.data.p_data = &m_figure14;
						break;
				case 0x0E:
						new_record.data.p_data = &m_figure15;
						break;
				case 0x0F:
						new_record.data.p_data = &m_figure16;
						break;
				case 0x10:
						new_record.data.p_data = &m_figure17;
						break;
				case 0x11:
						new_record.data.p_data = &m_figure18;
						break;
				case 0x12:
						new_record.data.p_data = &m_figure19;
						break;
				case 0x13:
						new_record.data.p_data = &m_figure20;
						break;
				case 0x14:
						new_record.data.p_data = &m_figure21;
						break;
				case 0x15:
						new_record.data.p_data = &m_figure22;
						break;
				case 0x16:
						new_record.data.p_data = &m_figure23;
						break;
				case 0x17:
						new_record.data.p_data = &m_figure24;
						break;
				case 0x18:
						new_record.data.p_data = &m_figure25;
						break;
				case 0x19:
						new_record.data.p_data = &m_figure26;
						break;
				case 0x1A:
						new_record.data.p_data = &m_figure0;
						break;
		}
		
		err_code = fds_record_update(&record_desc, &new_record);
		APP_ERROR_CHECK(err_code);
						
		err_code = fds_gc();
		APP_ERROR_CHECK(err_code);
}

void Rx_to_Flash(int ble_data_length, char * ble_data)
{

}

/***********************************************************************************************************
	* QRCODE
	*/
DOT_PIXEL pixel_type(int qrsize_display)
{
		switch(qrsize_display)
		{
			case 1: 
				return DOT_PIXEL_1X1;
			case 2: 
				return DOT_PIXEL_2X2;
			case 3: 
				return DOT_PIXEL_3X3;
			case 4: 
				return DOT_PIXEL_4X4;
			case 5: 
				return DOT_PIXEL_5X5;
			case 6: 
				return DOT_PIXEL_6X6;
			case 7: 
				return DOT_PIXEL_7X7;
			case 8: 
				return DOT_PIXEL_8X8;
			default:
				return DOT_PIXEL_8X8;
		}
}

static void Paint_Gen_Qrcode(DOT_PIXEL qr_pixel_size)
{
		int x_addr,y_addr;
		for(uint32_t x = 0; x < qrsize; x++){
				for(uint32_t y = 0; y < qrsize; y++){
						x_addr = (x+1) * qrsize_display;
						y_addr = (y+1) * qrsize_display;
						if(qrcodegen_getModule(qrcode,x,y)){
								Paint_DrawPoint(x_addr, y_addr, BLACK, qr_pixel_size, DOT_STYLE_DFT);
						}
						else{
								Paint_DrawPoint(x_addr, y_addr, WHITE, qr_pixel_size, DOT_STYLE_DFT);
						}
				}
		}
}

/***********************************************************************************************************
	* BUTTON
	*/
/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
						#if key_LOG
							NRF_LOG_INFO("Send button state change.");
						#endif
            break;
				case BSP_BUTTON_1:
						#if key_LOG
							NRF_LOG_INFO("Button2 pressed.");
						#endif
            break;
				case BSP_BUTTON_2:
            break;
				case BSP_BUTTON_3:
            break;
        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler},
				{BSP_BUTTON_1, false, BUTTON_PULL, button_event_handler},
				{BSP_BUTTON_2, false, BUTTON_PULL, button_event_handler},
				{BSP_BUTTON_3, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}



/***********************************************************************************************************
	* NUS
	*/


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
				#if key_LOG
					NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
					NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
				#endif
        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {		
						#if key_LOG
							do
							{
									
									err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
									if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
									{
											NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
											APP_ERROR_CHECK(err_code);
											
									}
									
									
							} while (err_code == NRF_ERROR_BUSY);
						#endif
						ble_data[i] = p_evt->params.rx_data.p_data[i];
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
						#if key_LOG
							while (app_uart_put('\n') == NRF_ERROR_BUSY);
						#endif
        }
				ble_data_length = p_evt->params.rx_data.length;
				bleflag = 1;
				#if key_LOG
					printf("\nReceived: %d bytes", ble_data_length);
				#endif
				//Rx_to_Flash(ble_data_length, ble_data);
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
						ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						#if key_LOG
							NRF_LOG_INFO("Connected");
							printf("\nConnected");
						#endif
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
						#if key_LOG
							NRF_LOG_INFO("Disconnected");
							printf("\nDisconnected");
						#endif
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
						#if key_LOG
							NRF_LOG_DEBUG("PHY update request.");
						#endif
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
				#if key_LOG
					NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
				#endif
    }
		#if key_LOG
			NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
										p_gatt->att_mtu_desired_central,
										p_gatt->att_mtu_desired_periph);
		#endif
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            //UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
										#if key_LOG
											NRF_LOG_DEBUG("Ready to send data over BLE NUS");
											NRF_LOG_HEXDUMP_DEBUG(data_array, index);
										#endif
                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void state_handle(void)
{
		ret_code_t err_code;
	
		if(adcflag == 1 & finished_flag == 0 & eink_busy == 0 & Init_Ready == 0 & Init_waiting == 0){
				EPD_Init();
		}
		if(adcflag == 1 & finished_flag == 0 & eink_busy == 0 & Init_Ready == 1){
				uint8_t value = pAddr[0];
			
				find_last_record();
				record_update(value);
				
				if(sysoff_flag == 1){
						check_pwroff(value);
				}
				figure_refresh(value);

				pAddr = (uint32_t*)(new_record.data.p_data);
				
				adc_threshold_update();
				sysoff_flag = 0;
				#if key_LED
					bsp_board_led_on(BSP_BOARD_LED_0);
				#endif
				
				finished_flag = 1;
				eink_busy = 1;
				Init_Ready = 0;
				app_timer_start(m_busy_timer_id, BUSY_TOGGLE_INTERVAL, NULL);
		
		}
		if(bleflag==2){
				bleflag = 0;
		}
		if(bleflag==1){
				nrf_gpio_pin_set(4);
				SPI_EP_Init();
				char display_data[ble_data_length];
				bleflag = 0;
				Paint_Clear(WHITE);
				/*
				Paint_DrawString_EN(5, 10, "ShanghaiTech", &Font20, WHITE, BLACK);
				Paint_DrawString_EN(5, 35, "Metal", &Font20, WHITE, BLACK);
				Paint_DrawString_EN(5, 60, "ViPSN-Eink", &Font20, BLACK, WHITE);
				Paint_DrawString_EN(5, 85, "Motion-Powered", &Font16, WHITE, BLACK);
				Paint_DrawString_EN(5, 105, "Eink Tag", &Font16, WHITE, BLACK);
				*/
				
				for(uint32_t i = 0; i < ble_data_length; i++){
						display_data[i] = ble_data[i];
				}
				
				//const char *text = display_data;                // User-supplied text
				enum qrcodegen_Ecc errCorLvl = qrcodegen_Ecc_LOW;  // Error correction level
				
				bool ok = qrcodegen_encodeText(display_data, tempBuffer, qrcode, errCorLvl, qrcodegen_VERSION_MIN, qrcodegen_VERSION_MAX, qrcodegen_Mask_AUTO, true);
				if(ok){
						qrsize = qrcodegen_getSize(qrcode);
						qrsize_display = 200 / qrsize;
						if(qrsize_display>=8){
								qrsize_display = 8;
						}
				}
				DOT_PIXEL qr_pixel_size = pixel_type(qrsize_display/2+1);
				//bsp_board_led_on(BSP_BOARD_LED_1);	
				Paint_Gen_Qrcode(qr_pixel_size);
				
				find_last_record();
				new_record.data.p_data = &m_figure0;
				err_code = fds_record_update(&record_desc, &new_record);
				APP_ERROR_CHECK(err_code);
								
				err_code = fds_gc();
				APP_ERROR_CHECK(err_code);
				
				pAddr = (uint32_t*)(new_record.data.p_data);
				adc_threshold_update();
				/*
				Paint_DrawString_EN(5, 125, "Rx_len: ", &Font16, WHITE, BLACK);
				Paint_DrawNum(85, 125, ble_data_length, &Font16, BLACK, WHITE);
				Paint_DrawString_EN(5, 145, display_data, &Font16, WHITE, BLACK);
				*/
				EPD_HW_Init();
				EPD_SetRAMValue_BaseMap(BlackImage);
				app_timer_start(m_busy_timer_id, BUSY_TOGGLE_INTERVAL, NULL);
		}
    else
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
		
    // Initialize.
		#if key_LOG
			uart_init();
			log_init();
		#endif
    timers_init();
	
    #if key_LED
				//leds_init();
				buttons_leds_init(&erase_bonds);
		#endif
		
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
		
		 // Start execution.
    //printf("\r\nUART started.\r\n");
    //NRF_LOG_INFO("Debug logging for UART over RTT started.");
    advertising_start();
		//bsp_board_led_on(BSP_BOARD_LED_1);
		//bsp_board_led_on(BSP_BOARD_LED_1);	
		
		#if key_LOG
				ble_gap_addr_t		addr;
				uint32_t err_code = sd_ble_gap_addr_get( &addr);
				APP_ERROR_CHECK(err_code);
				printf("\nstart");
				printf("\nMac Addr: %02X%02X%02X%02X%02X%02X",addr.addr[5],addr.addr[4],addr.addr[3],addr.addr[2],addr.addr[1],addr.addr[0]);
				
		#endif
		nrf_gpio_cfg_output(4);
		
		apptimer_create();
		
		FDS_init();

		UWORD Imagesize = 5000;
		if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        return -1;
		}
		Paint_NewImage(BlackImage, 200, 200, 270, WHITE);
		Paint_SelectImage(BlackImage);
		Paint_Clear(WHITE);
		
		#if key_Qrcode
				const char *text = "https://egate.shanghaitech.edu.cn/new/index.html";                // User-supplied text
				enum qrcodegen_Ecc errCorLvl = qrcodegen_Ecc_LOW;  // Error correction level
				
				bool ok = qrcodegen_encodeText(text, tempBuffer, qrcode, errCorLvl, qrcodegen_VERSION_MIN, qrcodegen_VERSION_MAX, qrcodegen_Mask_AUTO, true);
				if(ok){
						qrsize = qrcodegen_getSize(qrcode);
						qrsize_display = 200 / qrsize;
						if(qrsize_display>=8){
								qrsize_display = 8;
						}
				}
				DOT_PIXEL qr_pixel_size = pixel_type(qrsize_display/2+1);
		#endif
		
		#if key_GUI
			SPI_EP_Init();
			char pic_num[1];
			
			//pic_num[0] = pAddr[0];
			//Paint_DrawPoint(190, 190, BLACK, qr_pixel_size, DOT_STYLE_DFT);
			//Paint_DrawString_EN(5, 125, pic_num, &Font16, BLACK, WHITE);
				
			Paint_DrawString_EN(5, 10, "ShanghaiTech", &Font20, WHITE, BLACK);
			Paint_DrawString_EN(5, 35, "Metal", &Font20, WHITE, BLACK);
			Paint_DrawString_EN(5, 60, "ViPSN-Eink", &Font20, BLACK, WHITE);
			Paint_DrawString_EN(5, 85, "Motion-Powered", &Font16, WHITE, BLACK);
			Paint_DrawString_EN(5, 105, "Eink Tag", &Font16, WHITE, BLACK);
			Paint_DrawNum(5, 125, adc_threshold, &Font16, BLACK, WHITE);
			
			//Paint_Gen_Qrcode(qr_pixel_size);
			EPD_HW_Init();
			EPD_SetRAMValue_BaseMap(BlackImage);
		#endif 
		
		#if key_ADC
			ADC_start();
		#endif
		/**/
    // Enter main loop.
    for (;;)
    {
        state_handle();
				//nrf_pwr_mgmt_run();
    }
}


/**
 * @}
 */
