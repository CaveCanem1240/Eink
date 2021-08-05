/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
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
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "ble_err.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"

#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "nrf_drv_gpiote.h"




#include "nrf_delay.h"
#include "eink\e154.h"
#include "eink\e154font.h"

#include "nrf_nvmc.h"

#include "nrf_sdh_soc.h"

#include "fds.h"

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "Motion-Powered EinkTag"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                3200                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(1500, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1500, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as e
rror code on stack dump, can be used to identify stack location on stack unwind. */


#define gImage_num_CONCAT_2_(p1, p2)      p1 ## p2
int i,flag1,flag2;

#define key_LED 0
#define key_ADC 1
#define ble_debug 0



/*********************************************************************
 * DEFINITIONS
 */
#include "nrf_drv_saadc.h"
#include "nrfx_saadc.h"
#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                 /**< Size of timer operation queues. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                 /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                   /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  1000                    /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                /**< Maximum digital value for 10-bit ADC conversion. */

// VP = (RESULT * REFERENCE / 2^10) * 6
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) 
 
#define SAMPLES_IN_BUFFER               1

bool adcflag = 0;
bool finished_flag = 0;
bool Energy_flag = 0;
static void adcCallbackFunc(nrf_drv_saadc_evt_t const *pEvent);
static nrf_saadc_value_t s_bufferPool[SAMPLES_IN_BUFFER];

uint32_t  addr;
uint32_t*  pAddr;
uint32_t  offset;

static uint16_t adc_threshold;
static uint32_t last_pic;
bool sysoff_flag = 1;

#define FILE_ID         0x0001  /* The ID of the file to write the records into. */
#define RECORD_KEY_1    0x0001  /* A key for the first record. */
//#define RECORD_KEY_2    0x2222  /* A key for the second record. */

/*
static char const m_figure[] = "Metal";
static char next_figure[];
*/
static unsigned char base_map[5000];

static uint32_t    const m_figure0 = 0x00000000;
static uint32_t    const m_figure1 = 0x00000001;
static uint32_t    const m_figure2 = 0x00000002;
static uint32_t    const m_figure3 = 0x00000003;
static uint32_t    const m_figure4 = 0x00000004;
static uint32_t    const m_figure5 = 0x00000005;
static uint32_t    const m_figure6 = 0x00000006;
static uint32_t    const m_figure7 = 0x00000007;
static uint32_t    const m_figure8 = 0x00000008;
static uint32_t    const m_figure9 = 0x00000009;
static uint32_t    const m_figure10 = 0x0000000A;
static uint32_t    const m_figure11 = 0x0000000B;
static uint32_t    const m_figure12 = 0x0000000C;
static uint32_t    const m_figure13 = 0x0000000D;
static uint32_t    const m_figure14 = 0x0000000E;
static uint32_t    const m_figure15 = 0x0000000F;
static uint32_t    const m_figure16 = 0x00000010;
static uint32_t    const m_figure17 = 0x00000011;
static uint32_t    const m_figure18 = 0x00000012;
static uint32_t    const m_figure19 = 0x00000013;
static uint32_t    const m_figure20 = 0x00000014;
static uint32_t    const m_figure21 = 0x00000015;
static uint32_t    const m_figure22 = 0x00000016;
static uint32_t    const m_figure23 = 0x00000017;
static uint32_t    const m_figure24 = 0x00000018;
static uint32_t    const m_figure25 = 0x00000019;
static uint32_t    const m_figure26 = 0x0000001A;
static uint32_t    const m_figure27 = 0x0000001B;


/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void ADC_Init(void)
{
    ret_code_t err_code;
    //
    err_code = nrf_drv_saadc_init(NULL, adcCallbackFunc);
    APP_ERROR_CHECK(err_code);
    //
    nrf_saadc_channel_config_t channelConfig = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);   //
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

static void adcCallbackFunc(nrf_drv_saadc_evt_t const *pEvent)
{
    if(pEvent->type == NRF_DRV_SAADC_EVT_DONE)                                                                  // ????
    {
        nrf_saadc_value_t adcResult;
        ret_code_t errCode;
        errCode = nrf_drv_saadc_buffer_convert(pEvent->data.done.p_buffer, SAMPLES_IN_BUFFER);                  
        APP_ERROR_CHECK(errCode);
        adcResult = pEvent->data.done.p_buffer[0];
				if(adcflag == 0){
					if(adcResult>adc_threshold){
						#if key_LED
							#if ble_debug
								bsp_board_led_on(BSP_BOARD_LED_2);	
							#endif
						#endif
						adcflag = 1;
					}
				}
				else{
					if(adcResult<400){
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
				adc_threshold = 1600;
				break;
			case 0x1A:
				adc_threshold = 1600;
				break;
			default:
				adc_threshold = 1250;
				break;
		}
}

/*****************************************************************************************/

#define LED_TOGGLE_INTERVAL         APP_TIMER_TICKS(500)  //ms              // sending interval setting, unit ms 
#define BUSY_TOGGLE_INTERVAL         APP_TIMER_TICKS(100)  //ms              // sending interval setting, unit ms 
#define EPD_INIT_BUSY_TOGGLE_INTERVAL         APP_TIMER_TICKS(20)  //ms              // sending interval setting, unit ms 

APP_TIMER_DEF(m_adc_timer_id);
APP_TIMER_DEF(m_busy_timer_id); 
APP_TIMER_DEF(m_epd_rst_busy_timer_id); 
static bool eink_busy = 0;
static bool busy_flag = 0;


static bool epd_rst_eink_busy = 0;
static bool epd_rst_busy_flag = 0;

static bool Init_Ready = 0;
static bool Init_waiting = 0;
static int Init_Process = 0;
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
/*
static void busy_timer_timeout_handler(void *p_context)
{
		if(busy_flag==0){
			if(nrf_gpio_pin_read(27)==0){
				busy_flag = 1;
			}
		}
		else{
			if(nrf_gpio_pin_read(27)==0){
				nrf_gpio_pin_clear(4);
				eink_busy = 0;
				#if key_LED
					bsp_board_led_on(BSP_BOARD_LED_1);
				#endif
				SPI_EP_unInit();
				
				app_timer_stop(m_busy_timer_id);
				
			}
			busy_flag = 0;
		}
}
*/
static void busy_timer_timeout_handler(void *p_context)
{
		if(nrf_gpio_pin_read(27)==0){
				#if key_LED
					bsp_board_led_on(BSP_BOARD_LED_1);
				#endif
				nrf_gpio_pin_clear(4);
				eink_busy = 0;
				SPI_EP_unInit();
				
				app_timer_stop(m_busy_timer_id);
				
		}
}
/*************************************************************************************************************/

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
					bsp_board_led_on(BSP_BOARD_LED_3);
				#endif
				EPD_HW_Init3();
				Init_Ready = 1;
				Init_waiting = 0;
				//app_timer_start(m_epd_rst_busy_timer_id, EPD_INIT_BUSY_TOGGLE_INTERVAL, NULL);
			}
		}
}
/*
static void epd_rst_busy_timer_timeout_handler(void *p_context)
{
		
		if(epd_rst_busy_flag==0){
			if(nrf_gpio_pin_read(27)==0){
				//
				epd_rst_busy_flag = 1;
			}
		}
		else{
			if(nrf_gpio_pin_read(27)==0){
				//nrf_gpio_pin_clear(4);
				epd_rst_eink_busy = 0;
				app_timer_stop(m_epd_rst_busy_timer_id);
				switch(Init_Process){
					case 0:
						Init_Process = 1;
						//bsp_board_led_on(BSP_BOARD_LED_0);
						break;
					case 1:
						Init_Process = 2;
						//bsp_board_led_on(BSP_BOARD_LED_0);
						break;
					default:
						break;
				}
				//SPI_EP_unInit();
				Init_waiting = 0;
				//epd_rst_busy_flag = 0;
				//bsp_board_led_on(BSP_BOARD_LED_0);
				//EPD_Init();
			}
			epd_rst_busy_flag = 0;
			
		}
}
*/
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

/*************************************************************************************************************/
/**@brief 
 * Initialize timer for led_toggle_timeout_handler function with APP_TIMER_MODE_REPEATED mode.
 * @details    
 * First, it Initializes app_timer. Then create timer event when timer is out. Here the event is led_toggle_timeout_handler function.

static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_create(&m_adc_timer_id, APP_TIMER_MODE_SINGLE_SHOT, adc_timer_timeout_handler);
	  APP_ERROR_CHECK(err_code);
// it has two types of timer, one is being ticked repeating with the interval set before 
 
//    err_code = app_timer_create(&m_led_toggle_timer_id,
//                                APP_TIMER_MODE_REPEATED,
//                                led_toggle_timeout_handler);
// the other is being ticked singlely
}
 */
/**************************************************************************/


BLE_LBS_DEF(m_lbs);                                                             /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */



/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
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


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
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


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
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

static void set_figure(const uint32_t * next_figure)
{
		ret_code_t    err_code;
		
		fds_record_t        new_record;
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok;
			
		new_record.file_id           = FILE_ID;
		new_record.key               = RECORD_KEY_1;
		new_record.data.length_words = 1;
		new_record.data.p_data = next_figure;
				
		memset(&ftok, 0x00, sizeof(fds_find_token_t));
		fds_record_find(FILE_ID, RECORD_KEY_1, &record_desc, &ftok);
				
		err_code = fds_record_update(&record_desc, &new_record);
		APP_ERROR_CHECK(err_code);
						
		err_code = fds_gc();
		APP_ERROR_CHECK(err_code);
				
		pAddr = (uint32_t*)(new_record.data.p_data);
}
/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
		ret_code_t    err_code;
    if (led_state)
    {
				#if key_LED
					#if ble_debug
						bsp_board_led_on(LEDBUTTON_LED);
					#endif
				#endif
				
				//set_figure(&m_figure1);
				
        NRF_LOG_INFO("Received LED ON!");
    }
    else
    {
				#if key_LED
					#if ble_debug
						bsp_board_led_off(LEDBUTTON_LED);	
					#endif
				#endif
        NRF_LOG_INFO("Received LED OFF!");
				
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_lbs_init_t     init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize LBS.
    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
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
    ret_code_t             err_code;
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


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
		#if key_LED
			#if ble_debug
				bsp_board_led_on(ADVERTISING_LED);	
			#endif
		#endif
    
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
						#if key_LED
							#if ble_debug
								bsp_board_led_on(CONNECTED_LED);
								bsp_board_led_off(ADVERTISING_LED);
							#endif
						#endif
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
						set_figure(&m_figure26);
						adc_threshold_update();
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
						#if key_LED
							#if ble_debug
								bsp_board_led_off(CONNECTED_LED);
							#endif
						#endif
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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

uint32_t ble_button2_send(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t val)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(val);
    memset(&params, 0, sizeof(params));
    params.type   = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_lbs->button_char_handles.value_handle;  //Button characteristic value handle
    params.p_data = &val;
    params.p_len  = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);   
}

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
            NRF_LOG_INFO("Send button state change.");
            err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;
				case BSP_BUTTON_1:
            NRF_LOG_INFO("Button2 pressed.");
            ble_button2_send(m_conn_handle, &m_lbs, 5);
						#if key_LED
								bsp_board_led_on(BSP_BOARD_LED_2);
						#endif
            break;
				case BSP_BUTTON_2:
						
						flag1=1;
            break;
				case BSP_BUTTON_3:
						
						flag2=1;
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


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/*************************************************************************************************************/
//EPD_Dis_Part(0,120,gImage_sht13,40,40);
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

/*************************************************************************************************************/
static fds_record_t        new_record;
static fds_record_desc_t   record_desc;
static fds_find_token_t    ftok;

static void find_last_record(void)
{
		new_record.file_id           = FILE_ID;
		new_record.key               = RECORD_KEY_1;
		new_record.data.length_words = 1;
				
		memset(&ftok, 0x00, sizeof(fds_find_token_t));
		fds_record_find(FILE_ID, RECORD_KEY_1, &record_desc, &ftok);
			
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
							
								if (fds_record_find(FILE_ID, RECORD_KEY_1, &record_desc, &ftok) == NRF_SUCCESS){
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
										init_record.key               = RECORD_KEY_1;
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


/*************************************************************************************************************/

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the state (main loop).
 *
 * @details If there is no pending operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
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
    else
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
	
		// Initialize.
		#if key_LED
				leds_init();
    #endif

    nrf_gpio_cfg_output(4);
		//nrf_gpio_cfg_input(27,NRF_GPIO_PIN_NOPULL);
		
    timers_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    advertising_start();
		
		
		#if key_ADC
			ADC_Init();
			ADC_Disable();
			err_code = app_timer_create(&m_adc_timer_id, APP_TIMER_MODE_REPEATED, adc_timer_timeout_handler);
			APP_ERROR_CHECK(err_code);
		#endif
		
		err_code = app_timer_create(&m_busy_timer_id, APP_TIMER_MODE_REPEATED, busy_timer_timeout_handler);
		APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_epd_rst_busy_timer_id, APP_TIMER_MODE_REPEATED, epd_rst_busy_timer_timeout_handler);
		APP_ERROR_CHECK(err_code);

		/*********************************************/
		
		err_code = fds_register(fds_evt_handler);
		APP_ERROR_CHECK(err_code);
		
		err_code = fds_init();
		APP_ERROR_CHECK(err_code);
		
		#if key_ADC
			app_timer_start(m_adc_timer_id, LED_TOGGLE_INTERVAL, NULL);
		#endif 
		
		
		
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
