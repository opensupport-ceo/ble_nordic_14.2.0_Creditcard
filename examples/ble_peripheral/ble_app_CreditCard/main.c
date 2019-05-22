/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
 * Copyright (c) 2017 - 2019, APMATE
 *
 * All rights reserved.
 *
 * Author: Jaehong Park, jaehong1972@gmail.com
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
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "boards.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"

#include "nrf_temp.h"

#include "app_button.h"
#include "nrf_delay.h"
#include "ble_dfu.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "peer_manager.h"
#include "fds.h"
#include "bsp.h" //bonds

//#define DEV_TEMP
#if defined(BATT_ADC)
#define SAMPLES_IN_BUFFER 5
#ifdef USE_ADC_TIMER
#define SAMPLES 1
#endif
#if defined(BATT_POWEROFF)
#define VOLT_MAX    0x32B //4.2V
#define CUTOFF_VAL  0x281 //3.3V
#endif

/* Soft_device use #0 timer for another purpose,
** So we have to use another timer block exept for #0 timer. 
*/
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);

static nrf_saadc_value_t adc_buf[2][SAMPLES_IN_BUFFER];
#ifdef USE_ADC_TIMER
static nrf_saadc_value_t batt_adc_buf[2][SAMPLES];
#endif
static uint8_t 		uart_send_data[20];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;
#endif

#if defined(TEMP_ONBOARD_ADC)
// This function contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31.
static int16_t volatile onboard_temperature = 0;
static bool temp_read_start = false;
static bool temp_read_done = false;
#endif
static uint8_t paired_connection = 0;

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "APMATE-00:11:22:33:44:55"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL               MSEC_TO_UNITS(1000, UNIT_0_625_MS)           /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                           /**< The advertising timeout (in units of seconds). */

#define SECURITY_REQUEST_DELAY          APP_TIMER_TICKS(400)                        /**< Delay after connection until Security Request is sent, if necessary (ticks). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#if defined(STATIC_KEY)
#define SEC_PARAM_MITM                  1                                           /**< Man In The Middle protection required (applicable when display module is detected). */
#else
#define SEC_PARAM_MITM                  1                                           /**< Man In The Middle protection required (applicable when display module is detected). */
#endif
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#if defined(STATIC_KEY)
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_DISPLAY_ONLY                /**< Display I/O capabilities. */
#else
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_DISPLAY_ONLY 
#endif
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define PASSKEY_TXT                     "Passkey:"                                  /**< Message to be displayed together with the pass-key. */
#define PASSKEY_TXT_LENGTH              8                                           /**< Length of message to be displayed together with the pass-key. */
#define PASSKEY_LENGTH                  6                                           /**< Length of pass-key received by the stack for display. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus);                                                                 /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

#if defined(USE_NEWLED)
APP_TIMER_DEF(m_led_idle_timer_id);
#endif
APP_TIMER_DEF(m_led_timer_id);
APP_TIMER_DEF(m_btn_timer_id);
#if defined(APP_BTN)
APP_TIMER_DEF(m_app_btn_timer_id);
APP_TIMER_DEF(m_app_btn_poweroff_timer_id);
APP_TIMER_DEF(m_app_btn_double_timer_id);
#endif
#if defined(OLD_BATT_ADC)
APP_TIMER_DEF(m_battery_timer_id);
#endif
#ifdef USE_ADC_TIMER
APP_TIMER_DEF(m_batt_timer_id);
#endif
#if defined(BATT_POWEROFF)
APP_TIMER_DEF(m_batt_adc_timer_id);
#endif
APP_TIMER_DEF(m_batt_adc_send_timer_id);

#if defined(TEMP_ONBOARD_ADC)
APP_TIMER_DEF(m_temperature_timer_id);
#endif
#if defined(WAKEUP_NOPAIRED)
APP_TIMER_DEF(m_wakeup_nopaired_timer_id);
#endif
APP_TIMER_DEF(m_sec_req_timer_id);                                                  /**< Security Request timer. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(10)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */


#if defined(BTN_PWR_ON)
#define BTN1_INTERVAL1          APP_TIMER_TICKS(1000)
#define BTN1_INTERVAL2          APP_TIMER_TICKS(3000) //3sec time.
#define BTN1_POWEROFF_INTERVAL2 APP_TIMER_TICKS(5000)
#define BTN1_DOUBLE_INTERVAL    APP_TIMER_TICKS(1000)
#else
#define BTN1_INTERVAL                                             APP_TIMER_TICKS(100)
#endif
#if defined(USE_CARD_LED)
#define LED_INTERVAL0                                             APP_TIMER_TICKS(100)
#define LED_INT_250MS     APP_TIMER_TICKS(250)
#define LED_INT_500MS     APP_TIMER_TICKS(500)
#define LED_INT_1SEC      APP_TIMER_TICKS(1000)
#else
#define LED_INTERVAL1                                             APP_TIMER_TICKS(100)
#define LED_ALERT_INTERVAL                                            APP_TIMER_TICKS(100)
#endif
#if defined(USE_NEWLED)
#define LED_IDLE_INTERVAL                                             APP_TIMER_TICKS(1000)
#endif
#if defined(OLD_BATT_ADC)
#define BATTERY_INTERVAL                                          APP_TIMER_TICKS(60000)
#endif
#ifdef USE_ADC_TIMER
#ifdef DEV_TEMP
#define BATT_READ_START_INTERVAL_FIRST                        APP_TIMER_TICKS(1000*10)
#define BATT_READ_START_INTERVAL                               APP_TIMER_TICKS(1000*60*60)
#else
#define BATT_READ_START_INTERVAL_FIRST                        APP_TIMER_TICKS(1000*10)
#define BATT_READ_START_INTERVAL                              APP_TIMER_TICKS(1000*60*60)
#endif
#endif
#if defined(BATT_POWEROFF)
#ifdef DEV_TEMP
#define BATTERY_READ_START_INTERVAL_FIRST                        APP_TIMER_TICKS(1000*10)
#define BATTERY_READ_START_INTERVAL                               APP_TIMER_TICKS(1000*60*60)
#else
#define BATTERY_READ_START_INTERVAL_FIRST                        APP_TIMER_TICKS(1000*10)
#define BATTERY_READ_START_INTERVAL                              APP_TIMER_TICKS(1000*60*60)
#endif
#endif
#define SEND_BATT_INTERVAL  APP_TIMER_TICKS(10000)

#if defined(TEMP_ONBOARD_ADC)
#define TEMPERATURE_INTERVAL                                      APP_TIMER_TICKS(10000)
#endif
#if defined(WAKEUP_NOPAIRED)
#define WAKEUP_NOPAIRED_INTERVAL                                      APP_TIMER_TICKS(60000*10)
#endif

#if defined(WAKEUP_NOPAIRED)
static bool paired_once = false;
#endif
#if defined(BATT_POWEROFF)
static bool check_batt_adc = false;
#endif

#if defined(BTN_LED1)
static uint32_t pressed_time, released_time, pressed_duration = 0;
static uint8_t pressed_cnt, click_cnt, double_cnt = 0;
#endif

#ifndef USE_CARD_LED
static uint32_t alert_cnt;
static bool alert_on = false;
static bool alert_btn_on = false;
static bool btn_release = false;
static bool m_led_timer_f = false;
#endif

static bool over5sec_f = false;
static bool over3sec_f = false;
static bool over1sec_f = false;
static bool under1sec_f = false;

static void send_to_phoneapp_selfcamera(void);

static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
#if (0)
    ,{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
#endif
};

static void advertising_start(bool erase_bonds);

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

uint8_t hex_to_ascii(bool up_down,uint8_t hex_data)
{
  uint8_t temp_hex_data;
  if(up_down){
    temp_hex_data = hex_data >> 4;
  }else{
    temp_hex_data = hex_data & 0x0f;
  }
  if(temp_hex_data<10){
    return (0x30+temp_hex_data);
  }else{
    return (55+temp_hex_data);
  }
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code;

    nrf_gpio_cfg_sense_input(APMATE_BTN_1,NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_SENSE_LOW);

#if (0)
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
#endif

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code= sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

#if defined(BTN_PWR_ON)
static void power_off_instant(void)
{
  NRF_LOG_INFO("System down...");
  nrf_gpio_pin_clear(APMATE_P_CTL);
  sleep_mode_enter();
}

static void led321_onoff_500ms_2cnt_poweroff(void)
{
#if defined(BOARD_R11)  
  nrf_gpio_pin_set(APMATE_LED_3);
  nrf_delay_ms(500);
  nrf_gpio_pin_clear(APMATE_LED_3);
  nrf_gpio_pin_set(APMATE_LED_2);
  nrf_delay_ms(500);
  nrf_gpio_pin_clear(APMATE_LED_2);
  nrf_gpio_pin_set(APMATE_LED_1);
  nrf_delay_ms(500);
  nrf_gpio_pin_clear(APMATE_LED_1);
  nrf_gpio_pin_set(APMATE_LED_3);
  nrf_delay_ms(500);
  nrf_gpio_pin_clear(APMATE_LED_3);
  nrf_gpio_pin_set(APMATE_LED_2);
  nrf_delay_ms(500);
  nrf_gpio_pin_clear(APMATE_LED_2);
  nrf_gpio_pin_set(APMATE_LED_1);
  nrf_delay_ms(500);
  nrf_gpio_pin_clear(APMATE_LED_1);
#endif
}
static void power_off(void)
{
  NRF_LOG_INFO("Powering off...");
  led321_onoff_500ms_2cnt_poweroff();
  power_off_instant();
}
#endif

#if defined(TEMP_ONBOARD_ADC)
#if defined(USE_SD_TEMP_API)
static int32_t read_temperature(void)
{
  uint32_t                err_code;
  int32_t temperature;

  //This function will block until the temperature measurement is done.
  //It takes around 50 us from call to return.
  err_code = sd_temp_get(&temperature);
  if (err_code == NRF_SUCCESS){
    temp_read_done = true;
    return temperature;
  }else{
    temp_read_done = false;
    return temperature;
  }
}
#else
static int32_t read_temperature_complete(void)
{
  int32_t temperature;
  
  NRF_TEMP->EVENTS_DATARDY = 0;

  /**@note Workaround for PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register. */
  temperature = (nrf_temp_read() / 4);

  /**@note Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. */
  NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */

  return temperature;
}

static bool is_read_temperature_done(void)
{
  if (NRF_TEMP->EVENTS_DATARDY == 0){
    temp_read_done = false;
  }else {
    temp_read_done = true;
  }

  return temp_read_done;
}

static bool read_temperature_start(void)
{
  NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

  if (NRF_TEMP->EVENTS_DATARDY == 0){
     temp_read_done = false;
  }else{
     temp_read_done = true;
  }
  return temp_read_done;
}
#endif

#if defined(USE_REGISTER)
static int32_t read_temperature(void)
{
  // This function contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31.
  int32_t volatile temp;

  NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

  /* Busy wait while temperature measurement is not finished, you can skip waiting if you enable interrupt for DATARDY event and read the result in the interrupt. */
  /*lint -e{845} // A zero has been given as right argument to operator '|'" */
  while (NRF_TEMP->EVENTS_DATARDY == 0)
  {
      // Do nothing.
  }
  NRF_TEMP->EVENTS_DATARDY = 0;

  /**@note Workaround for PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register. */
  temp = (nrf_temp_read() / 4);

  /**@note Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. */
  NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */

  NRF_LOG_INFO("Actual temperature: %d", (int)temp);
  nrf_delay_ms(500);
}
#endif//#if defined(USE_REGISTER)
#endif//#if defined(TEMP_ONBOARD_ADC)

#if defined(BATT_ADC)
static nrf_saadc_value_t saved_batt_adc = 0;
static void send_to_phoneapp_batt(int16_t batt_adc);
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
    
      	nrf_saadc_value_t tmp_batt_adc = 0; //int16_t
        nrf_saadc_value_t tmp_adc_sum = 0;
        uint32_t          err_code;
        
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
		
        int i;
        //NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            //NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
            tmp_adc_sum += p_event->data.done.p_buffer[i];
        }
        m_adc_evt_counter++;

        tmp_batt_adc = tmp_adc_sum/SAMPLES_IN_BUFFER;

#if defined(BATT_POWEROFF)
        if(check_batt_adc){//BATTERY_READ_START_INTERVAL
          NRF_LOG_INFO("Batt ADC: 0x%X(%d)", tmp_batt_adc, tmp_batt_adc);
          saved_batt_adc = tmp_batt_adc;
        }
#else
        NRF_LOG_INFO("Batt ADC: 0x%X(%d)", tmp_batt_adc, tmp_batt_adc);
#endif

#ifdef BATT_POWEROFF
        if(check_batt_adc){
          //send_to_phoneapp_batt(tmp_batt_adc);
          check_batt_adc = false;
          if(tmp_batt_adc <= CUTOFF_VAL){ // When under 3.3V.
              //nrf_gpio_pin_clear(APMATE_BAT_V);
              //nrf_gpio_pin_clear(APMATE_P_CTL);
              NRF_LOG_INFO("Batt adc under 3.3V...");
        #ifdef DEV_TEMP
              NRF_LOG_INFO("Batt ADC read-check stop...");
              check_batt_adc = false;
              nrf_gpio_pin_clear(APMATE_BAT_V);
              err_code = app_timer_start(m_batt_adc_timer_id,BATTERY_READ_START_INTERVAL,0);
              APP_ERROR_CHECK(err_code);
        #else
              power_off();
        #endif
          }else{
            NRF_LOG_INFO("Batt ADC read-check stop...");
            check_batt_adc = false;
            nrf_gpio_pin_clear(APMATE_BAT_V);
            err_code = app_timer_start(m_batt_adc_timer_id,BATTERY_READ_START_INTERVAL,0);
            APP_ERROR_CHECK(err_code);
          }
        }
#endif
    }
}
#endif

#ifdef USE_ADC_TIMER
void saadc_batt_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
	nrf_saadc_value_t tmp_batt_adc = 0;
        nrf_saadc_value_t tmp_adc_sum = 0;
        uint32_t          err_code;
        //tmp_batt_adc = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES);
        APP_ERROR_CHECK(err_code);
		
        int i;
        //NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

        for (i = 0; i < SAMPLES; i++)
        {
            //NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
            tmp_adc_sum += p_event->data.done.p_buffer[i];
        }
        m_adc_evt_counter++;

        //batt_adc = p_event->data.done.p_buffer[0];
        tmp_batt_adc = tmp_adc_sum/SAMPLES;

#if defined(BATT_POWEROFF)
        if(check_batt_adc){//BATT_READ_START_INTERVAL
          NRF_LOG_INFO("Batt ADC: 0x%X(%d)", tmp_batt_adc, tmp_batt_adc);
          saved_batt_adc = tmp_batt_adc;
        }
#else
        NRF_LOG_INFO("Batt ADC: 0x%X(%d)", tmp_batt_adc, tmp_batt_adc);
#endif

#ifdef BATT_POWEROFF
        if(check_batt_adc){
          //send_to_phoneapp_batt(tmp_batt_adc);
          check_batt_adc = false;
          if(tmp_batt_adc <= CUTOFF_VAL){ // When under 3.3V.
              //nrf_gpio_pin_clear(APMATE_BAT_V);
              //nrf_gpio_pin_clear(APMATE_P_CTL);
              NRF_LOG_INFO("Batt adc under 3.3V...");
        #ifdef DEV_TEMP
              NRF_LOG_INFO("Batt ADC read-check stop...");
              check_batt_adc = false;
              nrf_gpio_pin_clear(APMATE_BAT_V);
              err_code = app_timer_start(m_batt_timer_id,BATT_READ_START_INTERVAL,0);
              APP_ERROR_CHECK(err_code);
        #else
              power_off();
        #endif
          }else{
            NRF_LOG_INFO("Batt ADC read-check stop...");
            check_batt_adc = false;
            nrf_gpio_pin_clear(APMATE_BAT_V);
            err_code = app_timer_start(m_batt_timer_id,BATT_READ_START_INTERVAL,0);
            APP_ERROR_CHECK(err_code);
          }
        }
#endif
    }
}
#endif

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
#if defined(STATIC_KEY)
    ble_opt_t               ble_opt;
#endif
    uint8_t temp_name[]=DEVICE_NAME;
    ble_gap_addr_t temp_ad;
    sd_ble_gap_addr_get(&temp_ad);
   
    for(uint8_t i=0; i<6;i++){
      temp_name[(i+2)*3+1]=hex_to_ascii(1,temp_ad.addr[5-i]);
      temp_name[(i+2)*3+2]=hex_to_ascii(0,temp_ad.addr[5-i]);
    }

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) temp_name,
                                          strlen(temp_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT; 

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

#if defined(STATIC_KEY)
    ble_opt.gap_opt.passkey.p_passkey = "000000";

    err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &ble_opt);
    APP_ERROR_CHECK(err_code);
#endif
}
     
#if defined(USE_CARD_LED)
#define LED_STATE 3 //led1, led2, led3

#define LED_INIT_V    0
#define LED123_500MS_REPEATED 1
#define LED123_500MS  2
#define LED123_1S     3
#define LED321_500MS  4
#define LED321_1S     5
#define LEDALL_ONOFF  6
#define LED123_500MS_F   7
#define LED123_250MS  8

#define CONTINUED     0
static uint8_t alert_led_totalcnt = CONTINUED; //0:continuous

static uint8_t led_type = LED_INIT_V;
//command received
static void led1_led2_led3_onoff_500ms_repeatedly(void)
{
  uint32_t err_code;

  led_type = LED123_500MS_REPEATED;
  alert_led_totalcnt = CONTINUED;
  err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
  APP_ERROR_CHECK(err_code);
}

//paring mode
static void led1_led2_led3_onoff_1s_5cnt_paring_mode(void)
{
  uint32_t err_code;

  led_type = LED123_1S;
  alert_led_totalcnt = 5*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INT_1SEC,0);
  APP_ERROR_CHECK(err_code);
}

//paring mode
static void led1_led2_led3_onoff_1s_5cnt_paring_mode_forced(void)//500ms
{
  uint32_t err_code;

  led_type = LED123_500MS_F;
  alert_led_totalcnt = 5*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
  APP_ERROR_CHECK(err_code);
}

//power-on
static void led1_led2_led3_onoff_500ms_1cnt_poweron(void)
{
  uint32_t err_code;

  led_type = LED123_500MS;
  alert_led_totalcnt = 1*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
  APP_ERROR_CHECK(err_code);
}

//paring connection
static void led1_led2_led3_onoff_500ms_2cnt_paired_connectioned(void)
{
  uint32_t err_code;

  led_type = LED123_500MS;
  alert_led_totalcnt = 2*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
  APP_ERROR_CHECK(err_code);
}

/*
//button_power off
static void led3_led2_led1_onoff_500ms_2cnt_button_power_off(void)
{
  uint32_t err_code;

  led_type = LED321_500MS;
  alert_led_totalcnt = 2*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
  APP_ERROR_CHECK(err_code);
}
*/

//phonepp -> tracker
static void led1_led2_led3_onoff_500ms_15cnt_phoneapp_to_tracker(void)
{
  uint32_t err_code;

  led_type = LED123_500MS;
  alert_led_totalcnt = 15*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
  APP_ERROR_CHECK(err_code);
}

//phonepp -> tracker
static void led1_led2_led3_onoff_250ms_15cnt_phoneapp_to_tracker(void)
{
  uint32_t err_code;

  led_type = LED123_250MS;
  alert_led_totalcnt = 15*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INT_250MS,0);
  APP_ERROR_CHECK(err_code);
}

//tracker -> phoneapp
static void led3_led2_led1_onoff_1s_15cnt_tracker_to_phoneapp(void)
{
  uint32_t err_code;

  led_type = LED321_1S;
  alert_led_totalcnt = 15*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INT_1SEC,0);
  APP_ERROR_CHECK(err_code);
}


//link loss
static void led1_led2_led3_500ms_15cnt_linkloss(void)
{
  uint32_t err_code;

  led_type = LED123_500MS;
  alert_led_totalcnt = 15*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
  APP_ERROR_CHECK(err_code);
}

//finished to search tracker
static void led1_led2_led3_500ms_15cnt_finished_to_search_tracker(void)
{
  uint32_t err_code;

  led_type = LED123_500MS;
  alert_led_totalcnt = 15*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
  APP_ERROR_CHECK(err_code);
}

void led_alert_start(void)
{
  uint32_t err_code;

  led_type = LEDALL_ONOFF;
  alert_led_totalcnt = 2*LED_STATE;
  err_code = app_timer_start(m_led_timer_id,LED_INTERVAL0,0);
  APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
#ifdef USE_CARD_LED
static bool led_alert_stop = false;
#endif

#define ID_OFF  0
#define ID_ON   1
#define ID_ONOFF  2
static uint8_t id_config = ID_OFF; //init value: off

static void id_config_set(uint8_t idconfig)
{
  uint32_t err_code;

  NRF_LOG_INFO("id_config: %d", idconfig);
  switch(idconfig){
    case ID_OFF:
      nrf_gpio_pin_clear(APMATE_ID_CONTROL);
    break;

    case ID_ON:
      nrf_gpio_pin_set(APMATE_ID_CONTROL);
    break;

    case ID_ONOFF:
    {
#if (1)
      if(paired_connection){
        //if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
          nrf_gpio_pin_set(APMATE_ID_CONTROL);
        //}else{
        //  nrf_gpio_pin_clear(APMATE_ID_CONTROL);
        //}
      }else{
          nrf_gpio_pin_clear(APMATE_ID_CONTROL);
      }
#else
      if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
        nrf_gpio_pin_set(APMATE_ID_CONTROL);
      }else{
        nrf_gpio_pin_clear(APMATE_ID_CONTROL);
      }
#endif
    }
    break;

    default:
    break;
  }
}

//0xCCB4, 0xCCB3
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
        NRF_LOG_INFO("Rx hex-string: %x %x %x",p_evt->params.rx_data.p_data[0],p_evt->params.rx_data.p_data[1],p_evt->params.rx_data.p_data[2]);
        if ((p_evt->params.rx_data.p_data[0] == 0xCC) && (p_evt->params.rx_data.p_data[1] == 0xB4)){
          switch(p_evt->params.rx_data.p_data[2]){
            case 0xDD: //
        #if (1)
              led_alert_stop = false;
              led1_led2_led3_onoff_250ms_15cnt_phoneapp_to_tracker();
        #else    
            #ifdef USE_CARD_LED
              led1_led2_led3_onoff_500ms_repeatedly();
            #endif
        #endif
              break;
            case 0xBB: //LED stop.
            #ifdef USE_CARD_LED
                led_alert_stop = true;
            #endif
              break;
            case 0xAA: //OTA start.
              bootloader_start();
              break;

            case 0xCC: //phoneapp -> tracker
              led_alert_stop = false;
              led1_led2_led3_onoff_250ms_15cnt_phoneapp_to_tracker();
              break;
          }
        }
        if ((p_evt->params.rx_data.p_data[0] == 0xCC) && (p_evt->params.rx_data.p_data[1] == 0xB3)){
          switch(p_evt->params.rx_data.p_data[2]){
  #if (1) //ID config
            case 0x01: //ID off
              id_config = ID_OFF;
              id_config_set(id_config);
              break;

            case 0x02: //ID on
              id_config = ID_ON;
              id_config_set(id_config);
              break;
          
            case 0x03: // ID on,off according to connection.
              id_config = ID_ONOFF;
              id_config_set(id_config);
              break;
  #endif
            default:
            break;
          }
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

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
           
            break;
        case BLE_ADV_EVT_IDLE:
          #if (0)      
            sleep_mode_enter();
          #endif 
            break;
        default:
            break;
    }
}

static uint32_t get_num_of_peers(void)
{
    uint32_t count = 0;
    count = pm_peer_count();
    NRF_LOG_INFO("Number of peer: %d", count);
    return count;
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
    #if (1)
          if(get_num_of_peers() >= 2){
            err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Number of peers, over 1, disconnecting");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                               BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
          }else
    #endif
          {

            NRF_LOG_INFO("Connected");
            m_peer_to_be_deleted = PM_PEER_ID_INVALID;

#if defined(BOARD_R11)
            //nrf_gpio_pin_set(APMATE_ID_CONTROL);
#endif
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      #if defined(PEER_MNG)
            // Start Security Request timer.
            err_code = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
      #endif

#if defined(OLD_BATT_ADC)
            err_code = app_timer_start(m_battery_timer_id,BATTERY_INTERVAL,0);
            APP_ERROR_CHECK(err_code);
#endif

#if defined(TEMP_ONBOARD_ADC)
            err_code = app_timer_start(m_temperature_timer_id, TEMPERATURE_INTERVAL,0);
            APP_ERROR_CHECK(err_code);
#endif
            if(id_config == ID_ONOFF){
              id_config_set(ID_ONOFF);
            }

            err_code = app_timer_start(m_batt_adc_send_timer_id,SEND_BATT_INTERVAL,0);
            APP_ERROR_CHECK(err_code);
          }
          break;

        case BLE_GAP_EVT_DISCONNECTED:
          {
            NRF_LOG_INFO("Disconnected");
            paired_connection = 0;
            if(id_config == ID_ONOFF){
              id_config_set(ID_ONOFF);
            }
            switch(p_ble_evt->evt.gap_evt.params.disconnected.reason){
              case BLE_HCI_CONNECTION_TIMEOUT:
                led1_led2_led3_500ms_15cnt_linkloss();
              break;

              default:
              break;
            }

            // LED indication will be changed when advertising starts.
#if defined(BOARD_R11)
            //nrf_gpio_pin_clear(APMATE_ID_CONTROL);
#endif
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
      #if defined(PEER_MNG)
            // Check if the last connected peer had not used MITM, if so, delete its bond information.
            if (m_peer_to_be_deleted != PM_PEER_ID_INVALID)
            {
                err_code = pm_peer_delete(m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_DEBUG("Collector's bond deleted");
                m_peer_to_be_deleted = PM_PEER_ID_INVALID;
            }
      #endif

#if defined(OLD_BATT_ADC)
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
#endif

#if defined(TEMP_ONBOARD_ADC)
            err_code = app_timer_stop(m_temperature_timer_id);
            APP_ERROR_CHECK(err_code);
#endif
            
            if(id_config == ID_ONOFF){
              id_config_set(ID_ONOFF);
            }
        #if (0) //Useless.
            err_code = app_timer_stop(m_batt_adc_send_timer_id);
            APP_ERROR_CHECK(err_code);
        #endif
          }
          break;

#ifndef S140
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
#endif

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      #if defined(PEER_MNG)
            NRF_LOG_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
      #else
        #if (0) //PEER_MNG
            NRF_LOG_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
        #else
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
        #endif
      #endif
            break;

#if defined(PEER_MNG)
        case BLE_GAP_EVT_PASSKEY_DISPLAY:
        {
            char passkey[PASSKEY_LENGTH + 1];
            memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
            passkey[PASSKEY_LENGTH] = 0;
            // Don't send delayed Security Request if security procedure is already in progress.
            err_code = app_timer_stop(m_sec_req_timer_id);
            APP_ERROR_CHECK(err_code);
        #if (1)
            NRF_LOG_INFO("Passkey: %s", nrf_log_push(passkey));
        #else
            NRF_LOG_INFO("Passkey: %s", passkey);
        #endif
        } break;
#endif

#if defined(STATIC_KEY)
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        {
         
           NRF_LOG_INFO("GAP Passkey request.");
           uint8_t passkey[] = "000000"; 
           if (p_ble_evt->evt.gap_evt.params.auth_key_request.key_type == BLE_GAP_AUTH_KEY_TYPE_PASSKEY)
           {
               err_code = sd_ble_gap_auth_key_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_AUTH_KEY_TYPE_PASSKEY, passkey);
               APP_ERROR_CHECK(err_code);
           }
         
        } break;
#endif

#if !defined (S112)
         case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            NRF_LOG_INFO("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST");
            ble_gap_data_length_params_t dl_params;

            // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        } break;
#endif //!defined (S112)
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            NRF_LOG_INFO("BLE_GATTS_EVT_SYS_ATTR_MISSING");
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

#if (1) //Later, should be verified.
        case BLE_GATTC_EVT_TIMEOUT:
            NRF_LOG_INFO("BLE_GATTC_EVT_TIMEOUT");
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_INFO("BLE_GATTS_EVT_TIMEOUT");
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
#endif
        case BLE_EVT_USER_MEM_REQUEST:
            NRF_LOG_INFO("BLE_EVT_USER_MEM_REQUEST");
        #if (1)//Used as ported.
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
        #else    
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
        #endif    
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            NRF_LOG_INFO("BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST");
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (0)
        case BLE_HCI_CONNECTION_TIMEOUT: //ble_hci.h, ble_gap.h
          break;
#endif
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
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

#if defined(APP_BTN)
static void app_button_init_click_cnt(void);
static void send_to_phoneapp_when_tracker_phone(void)
{
  uint32_t err_code;
  
  if(!paired_connection) return;
  if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
      NRF_LOG_INFO("send to phoneapp: tracker-> search phone ");
      uart_send_data[0] = 0xCC;
      uart_send_data[1] = 0xB5;
      uart_send_data[2] = 0x02;
      uint16_t length = 3;
      err_code = ble_nus_string_send(&m_nus, uart_send_data, &length);
      //app_button_init_click_cnt();
  }
}

static void send_to_phoneapp_click_cnt(void)
{
  uint32_t err_code;

  if(!paired_connection) return;
  if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
      NRF_LOG_INFO("send to phoneapp: click count ");
      //uart_send_data[0]=0x30+click_cnt/2;
      uart_send_data[0] = 0xCC;
      uart_send_data[1] = 0xB5;
      uart_send_data[2] = 0xDD;
      uint16_t length = 3;
      err_code = ble_nus_string_send(&m_nus, uart_send_data, &length);
      //app_button_init_click_cnt();
  }
}

static void send_to_phoneapp_power_off(void)
{
  uint32_t err_code;

  if(!paired_connection) return;
  if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
      NRF_LOG_INFO("send to phoneapp: power off info. ");
      uart_send_data[0] = 0xCC;
      uart_send_data[1] = 0xB5;
      uart_send_data[2] = 0xBB;
      uint16_t length = 3;
      err_code = ble_nus_string_send(&m_nus, uart_send_data, &length);
  }
}

static void send_to_phoneapp_temperature(uint8_t temp)
{
  uint32_t err_code;

  if(!paired_connection) return;
  if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
      NRF_LOG_INFO("send to phoneapp: temperature: %d", temp);
      uart_send_data[0] = 0xCC;
      uart_send_data[1] = 0xB7;
      uart_send_data[2] = temp;
      uint16_t length = 3;

      err_code = ble_nus_string_send(&m_nus, uart_send_data, &length);
  }
}

static void send_to_phoneapp_batt(int16_t batt_adc)
{
  uint32_t err_code;
  uint8_t batt[2] = {0,0};
  
  if(!paired_connection) return;
  batt[0] = (uint8_t)((0xFF00 & batt_adc)>>8);
  batt[1] = (uint8_t)((0x00FF & batt_adc));

  //if(!paired_connection) return;
  if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
      NRF_LOG_INFO("send to phoneapp: batt adc: %x %x", batt[0], batt[1]);
      uart_send_data[0] = 0xCC;
      uart_send_data[1] = 0xB6;
      uart_send_data[2] = batt[0];
      uart_send_data[3] = batt[1];
      uint16_t length = 4;
      err_code = ble_nus_string_send(&m_nus, uart_send_data, &length);
  }
}

//static bool btn_poweroff_flag_after_paring_mode = false;
static bool btn_paring_mode = false;
static bool btn_double_tracker = false;
static void app_btn_double_tracker_vars_init(void)
{
  btn_double_tracker = false;
  double_cnt = 0;
}
static void app_btn_tracker_timeout_handler(void * p_context)// 1sec timer
{
  uint32_t err_code;
  UNUSED_PARAMETER(p_context);

  NRF_LOG_INFO("1sec btn timer-> double_cnt: %d, btn_double_tracker: %d", (int)double_cnt, (int)btn_double_tracker);
  {//tracker
    if ((double_cnt == 2) && (btn_double_tracker == true)){
      if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
        NRF_LOG_INFO("button tracker->search phone...");
        send_to_phoneapp_when_tracker_phone();
    #if (0)
        led3_led2_led1_onoff_1s_15cnt_tracker_to_phoneapp();
    #endif
      }
    }
  
    //app_btn_double_tracker_vars_init();
  }

  {//selfcamera
    if ((double_cnt == 1) && (btn_double_tracker == true)){
      if(m_conn_handle != BLE_CONN_HANDLE_INVALID){// When paired only, send selfcamera command.
        NRF_LOG_INFO("button self-camerea...");
        send_to_phoneapp_selfcamera();
      }
    }
    //app_btn_double_tracker_vars_init();
  }

  app_btn_double_tracker_vars_init();
}

static bool btn_poweroff_mode = false;
static void app_btn_poweroff_timeout_handler(void * p_context) //5sec timer
{
    uint32_t err_code;
    UNUSED_PARAMETER(p_context);

    NRF_LOG_INFO("5sec btn timer-> pressed_cnt: %d, click_cnt: %d, btn_poweroff_mode: %d", (int)pressed_cnt, (int)click_cnt, (int)btn_poweroff_mode);
    //if(btn_poweroff_flag_after_paring_mode){
      //if ((pressed_cnt == 1) && (click_cnt == 1) && (btn_poweroff_mode == true)){
      if ((pressed_cnt == 1) && (btn_poweroff_mode == true)){
          if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
            send_to_phoneapp_power_off();
          }
        
          btn_poweroff_mode = false;
          //btn_poweroff_flag_after_paring_mode = false;
          NRF_LOG_INFO("button power off...");
          power_off();
      }
    //}
}

static void do_advertising_start_between_3sec_5sec(void)
{
  if((over3sec_f == true) && (over5sec_f == false)){//When 3sec < time < 5sec,
        advertising_start(true);
  }
}

static void app_btn_timeout_handler(void * p_context) //3sec timer handler.
{
    uint32_t err_code;
    UNUSED_PARAMETER(p_context);
  
    NRF_LOG_INFO("3sec btn timer-> click_cnt: %d, btn_paring_mode: %d", (int)click_cnt, (int)btn_paring_mode);

#ifdef USE_NEW_TRACKER_SEARCH_PHONE
#else
    if (click_cnt == 2){ //tracker -> phoneapp
      if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
        NRF_LOG_INFO("button 2 click: tracker->search phone...");
        send_to_phoneapp_when_tracker_phone();
        led3_led2_led1_onoff_1s_15cnt_tracker_to_phoneapp();
      }
    }  
#endif
    NRF_LOG_INFO("3sec btn timer-> pressed_cnt: %d", (int)pressed_cnt);
    //NRF_LOG_INFO("3sec btn timer-> pressed_cnt: %d, btn_poweroff_flag: %d", (int)pressed_cnt, (int)btn_poweroff_flag_after_paring_mode);
    //if((pressed_cnt == 1) && (btn_poweroff_flag_after_paring_mode == false) && (btn_paring_mode == true)) // paring mode
    if((pressed_cnt == 1) && (click_cnt == 1) && (btn_paring_mode == true)) // paring mode
    {
      NRF_LOG_INFO("button paring mode...");
      //btn_poweroff_flag_after_paring_mode = true;
      btn_paring_mode = false;
      //led1_led2_led3_onoff_1s_5cnt_paring_mode();
      led1_led2_led3_onoff_1s_5cnt_paring_mode_forced();
#if (0)
      //do_advertising_start_between_3sec_5sec();
      if((over3sec_f == true) && (over5sec_f == false)){//When 3sec < time < 5sec,
        advertising_start(true);
      else{
      }
#endif
      app_button_init_click_cnt();
      return;
    }
 
#if (0)//Not defined!

#else
    if((click_cnt >= 2)) //click counter
    {
  #if (0)
      if ((click_cnt >= 2) && (click_cnt <= 5)){//0xDD
  #else
      if ((click_cnt >= 2) && (click_cnt <= 5)){
  #endif
        send_to_phoneapp_click_cnt(); 
      }
      app_button_init_click_cnt();
      return;
    }
#endif
     return;
}
#endif

static void led123_onoff_500ms_1cnt_poweron(void)
{
  nrf_gpio_pin_set(APMATE_LED_1);
  nrf_gpio_pin_clear(APMATE_LED_2);
  nrf_gpio_pin_clear(APMATE_LED_3);
  nrf_delay_ms(500);
  nrf_gpio_pin_clear(APMATE_LED_1);
  nrf_gpio_pin_set(APMATE_LED_2);
  nrf_gpio_pin_clear(APMATE_LED_3);
  nrf_delay_ms(500);
  nrf_gpio_pin_clear(APMATE_LED_1);
  nrf_gpio_pin_clear(APMATE_LED_2);
  nrf_gpio_pin_set(APMATE_LED_3);
  nrf_delay_ms(500);
  nrf_gpio_pin_clear(APMATE_LED_3);
}

#if defined(BTN_PWR_ON)
static void btn_power_on_timeout_handler(void * p_context)
{
    uint32_t err_code;
    UNUSED_PARAMETER(p_context);

    if(nrf_gpio_pin_read(APMATE_BTN_1)==0){
#if defined(BTN_LED1)
      nrf_gpio_pin_set(APMATE_LED_1);
#endif
      nrf_gpio_pin_set(APMATE_P_CTL); //Power on board.
      NRF_LOG_INFO("Power on already setted!");
      
#if (1)
      led123_onoff_500ms_1cnt_poweron();
#else
      led1_led2_led3_onoff_500ms_1cnt_poweron();
#endif



#ifdef USE_ADC_TIMER
      err_code = app_timer_start(m_batt_timer_id,BATT_READ_START_INTERVAL_FIRST,0);
      APP_ERROR_CHECK(err_code);
#else
#if defined(BATT_POWEROFF)
      err_code = app_timer_start(m_batt_adc_timer_id,BATTERY_READ_START_INTERVAL_FIRST,0);
      APP_ERROR_CHECK(err_code);
#endif
#endif

#if defined(WAKEUP_NOPAIRED)
    err_code = app_timer_start(m_wakeup_nopaired_timer_id, WAKEUP_NOPAIRED_INTERVAL,0);
    APP_ERROR_CHECK(err_code);
#endif
    }else{
#if defined(BTN_LED1)
      nrf_gpio_pin_clear(APMATE_LED_1);
#endif
      power_off_instant();
    }
}
#endif//#if defined(BTN_PWR_ON)

#if defined(BATT_POWEROFF)
static void batt_power_off_timeout_handler(void * p_context)//BATTERY_READ_START_INTERVAL
{
  NRF_LOG_INFO("Batt ADC read-check starts...");
  nrf_gpio_pin_set(APMATE_BAT_V);
  nrf_delay_us(100);
  check_batt_adc = true;
}
#endif

static void battery_send_timeout_handler(void * p_context)//10sec timer
{
  uint32_t err_code;

  if(saved_batt_adc != 0){
    send_to_phoneapp_batt(saved_batt_adc);
  }

  if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
    err_code = app_timer_start(m_batt_adc_send_timer_id,SEND_BATT_INTERVAL,0);
    APP_ERROR_CHECK(err_code);
  }else{
    //err_code = app_timer_stop(m_batt_adc_send_timer_id);
    //APP_ERROR_CHECK(err_code);
  }
}


#if defined(USE_NEWLED)
static void led_idle_timer_handler(void * p_context)
{
  uint32_t err_code;
  static bool onoff = true;

  UNUSED_PARAMETER(p_context);

  //Temporary LED senario. 
  if (onoff){
    nrf_gpio_pin_set(APMATE_LED_1);
    nrf_gpio_pin_clear(APMATE_LED_2);
    onoff = false;
  }else{
    nrf_gpio_pin_clear(APMATE_LED_1);
    nrf_gpio_pin_set(APMATE_LED_2);
    onoff = true;
  }

}
#endif

#if defined(USE_CARD_LED)
static void send_to_phoneapp_when_led_off(void)
{
    uint32_t err_code;

    if(!paired_connection) return;
    uart_send_data[0]=0xCC;
    uart_send_data[1]=0xB5;
    if(( pressed_cnt != 0) || (click_cnt > 0 )){ //button pressed.
      uart_send_data[2]=0xEE;
    }else{ //button Not pressed.			
      uart_send_data[2]=0xFF;
    }
    if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
        NRF_LOG_INFO("send to phoneapp: led off info. ");
        uint16_t length = 3;
        err_code = ble_nus_string_send(&m_nus, uart_send_data, &length);
    }
}

static void led_all_clear(void)
{
    nrf_gpio_pin_clear(APMATE_LED_1);
    nrf_gpio_pin_clear(APMATE_LED_2);
    nrf_gpio_pin_clear(APMATE_LED_3);
}

static void led_all_set(void)
{
    nrf_gpio_pin_set(APMATE_LED_1);
    nrf_gpio_pin_set(APMATE_LED_2);
    nrf_gpio_pin_set(APMATE_LED_3);
}

#define INIT_CNT  0
static uint32_t change_led_stat = INIT_CNT;
static void led_timer_alert_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
  uint32_t err_code;
  
  if(led_type == LED123_500MS_F){//2
      switch (change_led_stat%3){
          case 0:
              nrf_gpio_pin_set(APMATE_LED_1);
              nrf_gpio_pin_clear(APMATE_LED_2);
              nrf_gpio_pin_clear(APMATE_LED_3);
          
              break;
          case 1:
              nrf_gpio_pin_clear(APMATE_LED_1);
              nrf_gpio_pin_set(APMATE_LED_2);
              nrf_gpio_pin_clear(APMATE_LED_3);
              break;
          case 2:
              nrf_gpio_pin_clear(APMATE_LED_1);
              nrf_gpio_pin_clear(APMATE_LED_2);
              nrf_gpio_pin_set(APMATE_LED_3);
              break;

          default:
              break;
      }

      if ( change_led_stat < alert_led_totalcnt){
          err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
          APP_ERROR_CHECK(err_code);
          change_led_stat++;
      }else{
          change_led_stat = INIT_CNT;
          send_to_phoneapp_when_led_off();
          led_all_clear();
      }

      return;
  }

  NRF_LOG_INFO("presssed_cnt: %d, click_cnt: %d", pressed_cnt, click_cnt);
  //if(( pressed_cnt != 0) || (click_cnt > 0 ))
  if(( pressed_cnt != 0))
  {
    change_led_stat = INIT_CNT;
    alert_led_totalcnt = 0;
    //app_timer_stop();

    //nrf_gpio_pin_clear(APMATE_LED_1);//LED1 paired woth button pressed.
    nrf_gpio_pin_clear(APMATE_LED_2);
    nrf_gpio_pin_clear(APMATE_LED_3);

    send_to_phoneapp_when_led_off();

  }
  else //When pressed_cnt == 0
  {
      if(led_type == LEDALL_ONOFF){//5
        switch (change_led_stat%2){
            case 0:
                nrf_gpio_pin_set(APMATE_LED_1);
                nrf_gpio_pin_set(APMATE_LED_2);
                nrf_gpio_pin_set(APMATE_LED_3);
            
                break;
            case 1:
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_3);
                break;

            default:
                break;
        }

        if (led_alert_stop == false){
          change_led_stat++;
          //if (change_led_stat < alert_led_totalcnt){
            err_code = app_timer_start(m_led_timer_id,LED_INTERVAL0,0);
            APP_ERROR_CHECK(err_code);
          //}
        }else{
          change_led_stat = INIT_CNT;
          led_all_clear();
          led_alert_stop = false;
        }
      }else if(led_type == LED123_500MS_REPEATED ){//1
        switch (change_led_stat%3){
            case 0:
                nrf_gpio_pin_set(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_3);
            
                break;
            case 1:
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_set(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_3);
                break;
            case 2:
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_set(APMATE_LED_3);
                break;

            default:
                break;
        }

        if (led_alert_stop == false){
          if (alert_led_totalcnt == CONTINUED){ // repeatedly.
            err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
            APP_ERROR_CHECK(err_code);
          }
          change_led_stat++;
        }else{
          change_led_stat = INIT_CNT;
          led_all_clear();
          led_alert_stop = false;
        }
      }else if(led_type == LED123_500MS){//
        switch (change_led_stat%3){
            case 0:
                nrf_gpio_pin_set(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_3);
            
                break;
            case 1:
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_set(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_3);
                break;
            case 2:
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_set(APMATE_LED_3);
                break;

            default:
                break;
        }
        
          if (( change_led_stat < alert_led_totalcnt) && (led_alert_stop == false)){
              err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
              APP_ERROR_CHECK(err_code);
              change_led_stat++;
          }else{
              change_led_stat = INIT_CNT;
              send_to_phoneapp_when_led_off();
              led_all_clear();
              led_alert_stop = false;
          }
      }else if(led_type == LED123_250MS){//LED123_250MS
        switch (change_led_stat%3){
            case 0:
                nrf_gpio_pin_set(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_3);
            
                break;
            case 1:
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_set(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_3);
                break;
            case 2:
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_set(APMATE_LED_3);
                break;

            default:
                break;
        }
        
          if (( change_led_stat < alert_led_totalcnt) && (led_alert_stop == false)){
              err_code = app_timer_start(m_led_timer_id,LED_INT_250MS,0);
              APP_ERROR_CHECK(err_code);
              change_led_stat++;
          }else{
              change_led_stat = INIT_CNT;
              send_to_phoneapp_when_led_off();
              led_all_clear();
              led_alert_stop = false;
          }
      }else if(led_type == LED123_1S){//2
        switch (change_led_stat%3){
            case 0:
                nrf_gpio_pin_set(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_3);
            
                break;
            case 1:
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_set(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_3);
                break;
            case 2:
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_set(APMATE_LED_3);
                break;

            default:
                break;
        }

        if ( change_led_stat < alert_led_totalcnt){
            err_code = app_timer_start(m_led_timer_id,LED_INT_1SEC,0);
            APP_ERROR_CHECK(err_code);
            change_led_stat++;
        }else{
            change_led_stat = INIT_CNT;
            send_to_phoneapp_when_led_off();
            led_all_clear();
        }

      }else if(led_type == LED321_500MS){//3
        switch (change_led_stat%3){
            case 0:
                nrf_gpio_pin_set(APMATE_LED_3);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_1);
            
                break;
            case 1:
                nrf_gpio_pin_clear(APMATE_LED_3);
                nrf_gpio_pin_set(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_1);
                break;
            case 2:
                nrf_gpio_pin_clear(APMATE_LED_3);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_set(APMATE_LED_1);
                break;

            default:
                break;
        }

        if ( change_led_stat < alert_led_totalcnt){
            err_code = app_timer_start(m_led_timer_id,LED_INT_500MS,0);
            APP_ERROR_CHECK(err_code);
            change_led_stat++;
        }else{
            change_led_stat = INIT_CNT;
            send_to_phoneapp_when_led_off();
            led_all_clear();
        }

      }else if(led_type == LED321_1S){//4
        switch (change_led_stat%3){
            case 0:
                nrf_gpio_pin_set(APMATE_LED_3);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_1);
            
                break;
            case 1:
                nrf_gpio_pin_clear(APMATE_LED_3);
                nrf_gpio_pin_set(APMATE_LED_2);
                nrf_gpio_pin_clear(APMATE_LED_1);
                break;
            case 2:
                nrf_gpio_pin_clear(APMATE_LED_3);
                nrf_gpio_pin_clear(APMATE_LED_2);
                nrf_gpio_pin_set(APMATE_LED_1);
                break;

            default:
                break;
        }

        if ( change_led_stat < alert_led_totalcnt){
            err_code = app_timer_start(m_led_timer_id,LED_INT_1SEC,0);
            APP_ERROR_CHECK(err_code);
            change_led_stat++;
        }else{
            change_led_stat = INIT_CNT;
            send_to_phoneapp_when_led_off();
            led_all_clear();
        }
      }
  }
}
#endif

#if defined(OLD_BATT_ADC)
static uint32_t sec_cnt;
static void battery_timeout_handler(void * p_context)
{
	uint32_t err_code;
        if(sec_cnt++>=60){
          sec_cnt=0;
          err_code = nrf_drv_saadc_sample();
          APP_ERROR_CHECK(err_code);
        }
	
}
#endif
#ifdef USE_ADC_TIMER
static void batt_timeout_handler(void * p_context)
{
  uint32_t err_code;
  UNUSED_PARAMETER(p_context);

  NRF_LOG_INFO("Batt ADC read-check starts...");
  nrf_gpio_pin_set(APMATE_BAT_V);
  nrf_delay_us(100);
  check_batt_adc = true;

  err_code = nrf_drv_saadc_sample();
  APP_ERROR_CHECK(err_code);
}
#endif

#if defined(TEMP_ONBOARD_ADC)
static void temperature_timeout_handler(void * p_context)//10sec timer
{
	uint32_t err_code;

#if defined(USE_SD_TEMP_API)
        if (temp_read_start == false){//First read?
          temp_read_start = true;
          onboard_temperature = read_temperature();
          if (temp_read_done == true){ //Read success.
            temp_read_start = false;
            temp_read_done = false;
            NRF_LOG_INFO("Onboard temperature: %d", (int)onboard_temperature);
            send_to_phoneapp_temperature((uint8_t)onboard_temperature);

          }else{ //Read fail.
            NRF_LOG_INFO("Onboard temperature: not avaiable.");
          }
        }else{//Under reading process.
          NRF_LOG_INFO("Onboard temperature: Under reading...");
        }
#else
        if (temp_read_start == false){
          temp_read_start = true;
          read_temperature_start();
          NRF_LOG_INFO("Onboard temperature: start to read.");
        }else{
          if (is_read_temperature_done()){ //Get temperature.
            temp_read_start = false;
            onboard_temperature =  read_temperature_complete();
            NRF_LOG_INFO("Onboard temperature: %d", (int)onboard_temperature);
          }else{//Skip next time and do anything.
            NRF_LOG_INFO("Onboard temperature: not avaiable.");
          }
        }
#endif
}
#endif

#if defined(WAKEUP_NOPAIRED)
static void wakeup_nopaired_timeout_handler(void * p_context)
{
  uint32_t err_code;

  NRF_LOG_INFO("nopaired_timeout...timer stop...");

  err_code = app_timer_stop(m_wakeup_nopaired_timer_id);
  APP_ERROR_CHECK(err_code);

  if (paired_once){
  }else{
    NRF_LOG_INFO("nopaired_timeout...");
    power_off();
  }
}
#endif

/**@brief Function for handling the Security Request timer timeout.
 *
 * @details This function will be called each time the Security Request timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    ret_code_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Initiate bonding.
        NRF_LOG_DEBUG("Start encryption");
        err_code = pm_conn_secure(m_conn_handle, false);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

void timer_init(void)
{
    uint32_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_btn_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            btn_power_on_timeout_handler);
    APP_ERROR_CHECK(err_code);

#ifndef USE_ADC_TIMER
#if defined(BATT_POWEROFF)
    err_code = app_timer_create(&m_batt_adc_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            batt_power_off_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif
#endif
    err_code = app_timer_create(&m_batt_adc_send_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                battery_send_timeout_handler);
    APP_ERROR_CHECK(err_code);

#if defined(APP_BTN)
    err_code = app_timer_create(&m_app_btn_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT, /* SINGLE_SHOT*/
                            app_btn_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif
    err_code = app_timer_create(&m_app_btn_poweroff_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT, /* SINGLE_SHOT*/
                            app_btn_poweroff_timeout_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_app_btn_double_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT, /* SINGLE_SHOT*/
                            app_btn_tracker_timeout_handler);
    APP_ERROR_CHECK(err_code);

#if defined(USE_NEWLED)
    err_code = app_timer_create(&m_led_idle_timer_id,
                            APP_TIMER_MODE_REPEATED, 
                            led_idle_timer_handler); // temp.
    APP_ERROR_CHECK(err_code);

#endif

#if defined(USE_CARD_LED)
    err_code = app_timer_create(&m_led_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT, 
                            led_timer_alert_handler);
    APP_ERROR_CHECK(err_code);
#endif

#if defined(OLD_BATT_ADC)
    err_code = app_timer_create(&m_battery_timer_id,
                            APP_TIMER_MODE_REPEATED,
                            battery_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif

#ifdef USE_ADC_TIMER
    err_code = app_timer_create(&m_batt_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            batt_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif

#if defined(TEMP_ONBOARD_ADC)
    err_code = app_timer_create(&m_temperature_timer_id,
                            APP_TIMER_MODE_REPEATED,
                            temperature_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif

#if defined(WAKEUP_NOPAIRED)
    err_code = app_timer_create(&m_wakeup_nopaired_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            wakeup_nopaired_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif

    // Create Security Request timer.
    err_code = app_timer_create(&m_sec_req_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sec_req_timeout_handler);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
#if defined(BTN_LED1)
#define SEC5  5000
#define SEC3  3000
#define SEC1  1000
#define MSEC500 500
#define MSEC100 100
#define MSEC10  10

static void app_button_init_time_flag_vars(void)
{
  over5sec_f = false;
  over3sec_f = false;
  over1sec_f = false;
  under1sec_f = false;
}
static void app_button_init_click_cnt(void)
{
  click_cnt = 0;
}

static void app_button_init_pressed_cnt(void)
{
  pressed_cnt = 0;
}
static void app_button_init_time_variable(void)
{
  pressed_duration = 0;
  released_time = 0;
  pressed_time = 0;
  app_button_init_time_flag_vars();
}

static void send_to_phoneapp_selfcamera(void)
{
  uint32_t err_code;

  if(!paired_connection) return;
  if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
    NRF_LOG_INFO("send to phoneapp: self-camera shutter ");
    uart_send_data[0] = 0xCC;
    uart_send_data[1] = 0xB5;
    uart_send_data[2] = 0x01;
    uint16_t length = 3;
    err_code = ble_nus_string_send(&m_nus, uart_send_data, &length);
  }
}

static void app_button_event_generator(void)
{
  pressed_duration = released_time - pressed_time;
  if (pressed_cnt == 1){
    if(pressed_duration)
    {
      if(pressed_duration >= SEC5){// btn poweroff.
        NRF_LOG_INFO("Over 5 secs, pressed...");
        over5sec_f = true;

#ifndef USE_NEW_POWEROFF
        if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
          send_to_phoneapp_power_off();
        }

        NRF_LOG_INFO("button power off...");
        power_off();
#endif

      }else if(pressed_duration >= SEC3){// go into pairing mode.
        NRF_LOG_INFO("Over 3 secs, pressed...");
        {
          over3sec_f = true;
          //led1_led2_led3_onoff_1s_5cnt_paring_mode_forced();
#if (0)
        //if(paired_connection){
          NRF_LOG_INFO("button paring mode...");
          btn_poweroff_flag_after_paring_mode = true;
          led1_led2_led3_onoff_1s_5cnt_paring_mode();
          advertising_start(true);
        //}
#endif
        }
      }else if(pressed_duration >= SEC1){// tracker->search phone.
        NRF_LOG_INFO("Over 1 secs, pressed...");
        over1sec_f = true;
#if defined(USE_NEW_TRACKER_SEARCH_PHONE)
    #if (0)
        if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
          NRF_LOG_INFO("button tracker->search phone...");
          send_to_phoneapp_when_tracker_phone();
          led3_led2_led1_onoff_1s_15cnt_tracker_to_phoneapp();
        }
    #endif
#endif
      }
#if (0)
      else if(pressed_duration >= MSEC500){// Do nothing.
        NRF_LOG_INFO("Over 0.5 secs, pressed...");
      
      }else if(pressed_duration >= MSEC100){// Self-camera.
        NRF_LOG_INFO("Under 0.3 secs, pressed...");
#ifndef USE_NEW_SELFCAMERA
        if(m_conn_handle != BLE_CONN_HANDLE_INVALID){// When paired only, send selfcamera command.
          NRF_LOG_INFO("button self-camerea...");
          send_to_phoneapp_selfcamera();
        }
#endif
      }else if(pressed_duration >= MSEC10){// Self-camera.
        NRF_LOG_INFO("Over 0.1 secs, pressed...");
#ifdef USE_NEW_SELFCAMERA        
        if(m_conn_handle != BLE_CONN_HANDLE_INVALID){// When paired only, send selfcamera command.
          NRF_LOG_INFO("button self-camerea...");
          send_to_phoneapp_selfcamera();
        }
#endif
      }
#else //#if (0)
      else {// // Self-camera.
        NRF_LOG_INFO("Under 1 secs, pressed...");
        under1sec_f = true;
#if (0)
#ifdef USE_NEW_SELFCAMERA        
        if(m_conn_handle != BLE_CONN_HANDLE_INVALID){// When paired only, send selfcamera command.
          NRF_LOG_INFO("button self-camerea...");
          send_to_phoneapp_selfcamera();
        }
#endif
#endif
      }
#endif//#if (0)
    }
  }
  do_advertising_start_between_3sec_5sec();
  app_button_init_time_variable();
  app_button_init_pressed_cnt();

}

static void app_button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
      case APMATE_BTN_1:
        switch (button_action)
        {
          case APP_BUTTON_PUSH:
          {
            NRF_LOG_INFO("Button pressed...");
            nrf_gpio_pin_set(APMATE_LED_1);
            pressed_time = app_timer_cnt_get()*1000/32768;//milli-sec
            pressed_cnt++;
            click_cnt++; // 3sec timer purpose.
            btn_poweroff_mode = true;
            btn_double_tracker = true;
            double_cnt++;// 1sec timer purpose.
            btn_paring_mode = true;
            NRF_LOG_INFO("pressed_time: %d, pressed_cnt: %d, click_cnt: %d, double_cnt: %d", (int)pressed_time, (int)pressed_cnt, (int)click_cnt, (int)double_cnt);

#ifdef USE_CARD_LED
       #if defined(APP_BTN)
            err_code = app_timer_start(m_app_btn_timer_id,BTN1_INTERVAL2,0);//3sec-click count&paring mode
            APP_ERROR_CHECK(err_code);
            
            //if (btn_poweroff_flag_after_paring_mode){
              err_code = app_timer_start(m_app_btn_poweroff_timer_id,BTN1_POWEROFF_INTERVAL2,0);//5sec-poweroff
              APP_ERROR_CHECK(err_code);
            //}

            err_code = app_timer_start(m_app_btn_double_timer_id,BTN1_DOUBLE_INTERVAL,0);//1sec-tracker&self-camera
            APP_ERROR_CHECK(err_code);
       #else
            err_code = app_timer_start(m_btn_timer_id,BTN1_INTERVAL2,0);
            APP_ERROR_CHECK(err_code);
       #endif
#endif
           }
           break;

           case APP_BUTTON_RELEASE:
           {  
              btn_poweroff_mode = false;
              if(!btn_poweroff_mode){
                err_code = app_timer_stop(m_app_btn_poweroff_timer_id);
                APP_ERROR_CHECK(err_code);
              }

              btn_paring_mode = false;
          #if (0)
              if(!btn_paring_mode){
                err_code = app_timer_stop(m_app_btn_poweroff_timer_id);
                APP_ERROR_CHECK(err_code);
              }
          #endif
              if (pressed_cnt == 1){//bugfix, omit log when poweron button.
                NRF_LOG_INFO("Button releaed...");
              }
              
              nrf_gpio_pin_clear(APMATE_LED_1);

              if (pressed_cnt == 1){//when released pressed fisrt, bugfix.
                released_time = app_timer_cnt_get()*1000/32768; //milli-sec
                NRF_LOG_INFO("released_time: %d, pressed_cnt: %d, click_cnt: %d, double_cnt: %d", (int)released_time, (int)pressed_cnt, (int)click_cnt, (int)double_cnt);
              }
           }
           break;
#if (0)
           case BSP_BUTTON_ACTION_LONG_PUSH:
           {
            //Do nothing here.
           }
           break;
#endif
         }   
      break;

      default:
         APP_ERROR_HANDLER(pin_no);
      break;
    }
    
    if(pressed_time == 0){//when released pressed fisrt, bugfix.
      app_button_init_time_variable();
      app_button_init_pressed_cnt();
      app_button_init_click_cnt();
    }

    if((pressed_time != 0) && (released_time != 0) && (pressed_cnt == 1)){
      app_button_event_generator();
    }else{
    }
}
#endif

/**@brief Function for initializing the button handler module.
 */
static void app_buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {APMATE_BTN_1, false, BUTTON_PULL, app_button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}

void gpio_init(void)
{

        nrf_gpio_cfg_output(APMATE_BAT_V);
        nrf_gpio_cfg_output(APMATE_P_CTL);
#if defined(BOARD_R11)
        nrf_gpio_cfg_output(APMATE_ID_CONTROL);
#endif
#if defined(BOARD_R7)
        nrf_gpio_cfg_output(APMATE_OUT_CHECK);
#endif
	nrf_gpio_pin_clear(APMATE_BAT_V);
#if defined(BOARD_R11)
        nrf_gpio_pin_clear(APMATE_ID_CONTROL);
#endif
#if defined(BOARD_R7)        
        nrf_gpio_pin_set(APMATE_OUT_CHECK);
#endif
	nrf_gpio_cfg_input(APMATE_BTN_1,NRF_GPIO_PIN_PULLUP);
	
        nrf_gpio_cfg_output(APMATE_LED_1);
	nrf_gpio_cfg_output(APMATE_LED_2);
#if defined(BOARD_R11)
        nrf_gpio_cfg_output(APMATE_LED_3);
#endif
        nrf_gpio_pin_clear(APMATE_LED_1);
	nrf_gpio_pin_clear(APMATE_LED_2);
#if defined(BOARD_R11)
        nrf_gpio_pin_clear(APMATE_LED_3);
#endif
}

#if defined(BTN_PWR_ON)
static void power_on(void)
{
  uint32_t err_code;

  if(nrf_gpio_pin_read(APMATE_BTN_1)==0){
    NRF_LOG_INFO("Power on timer start...");
#if defined(BTN_LED1)
    nrf_gpio_pin_set(APMATE_LED_1);
#endif
    err_code = app_timer_start(m_btn_timer_id,BTN1_INTERVAL1,0);
    APP_ERROR_CHECK(err_code);
  }else{
#if defined(BTN_LED1)
    nrf_gpio_pin_clear(APMATE_LED_1);
#endif
    power_off_instant();
  }
}
#endif

#if defined(BATT_ADC)
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
  NRF_LOG_INFO("timer_handler() occurs!");
}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
#ifdef DEV_TEMP
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 1000*1);
#else
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 1000*10);
#endif
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

static void saadc_init(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

#if defined(BOARD_R7)
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
#elif defined(BOARD_R11)
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
#endif
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(adc_buf[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(adc_buf[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

static void adc_configure(void)
{
    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
}
#endif //#if defined(BATT_ADC)

#ifdef USE_ADC_TIMER
static void batt_adc_configure(void)
{
      ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_batt_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(batt_adc_buf[0], SAMPLES);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(batt_adc_buf[1], SAMPLES);
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

#if defined(USE_TEST)
static void do_test(void)
{
  uint32_t err_code;

#if defined(TEMP_ONBOARD_ADC)
  err_code = app_timer_start(m_temperature_timer_id, TEMPERATURE_INTERVAL,0);
  APP_ERROR_CHECK(err_code);
#endif

#if defined(USE_NEWLED)
  err_code = app_timer_start(m_led_idle_timer_id, LED_IDLE_INTERVAL,0);
  APP_ERROR_CHECK(err_code);
#endif

}
#endif

static void do_send_batt_adc(void)
{
  uint32_t err_code;

  err_code = app_timer_start(m_batt_adc_send_timer_id, SEND_BATT_INTERVAL,0);
  APP_ERROR_CHECK(err_code);

}

#if defined(USE_UART)
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
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
           #if defined(USE_NOT_BUS_TX)
                NRF_LOG_DEBUG("Recieved data from UART");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);
           #else
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);
          
                do
                {
                    uint16_t length = (uint16_t)index;
                    err_code = ble_nus_string_send(&m_nus, data_array, &length);
                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_BUSY);
          #endif
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        case APP_UART_TX_EMPTY:
            //Data has been successfully transmitted on the UART
            break;
        case APP_UART_DATA:	
            //Data is ready on the UART					
            break;
#if (0)
        case APP_UART_DATA_READY:
            //Data is ready on the UART FIFO		
            break;
#endif
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
        .baud_rate    = NRF_UART_BAUDRATE_115200
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
#endif//#if defined(USE_UART)

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
    #if (1)
          if(get_num_of_peers() >= 2){
            err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Number of peers, over 1, disconnecting");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                               BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
          }else
    #endif
          {
            NRF_LOG_INFO("Connected to a previously bonded device.");
            // Start Security Request timer.
            err_code = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
          }
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
          pm_conn_sec_status_t conn_sec_status;
        
          // Check if the link is authenticated (meaning at least MITM).
          err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
          APP_ERROR_CHECK(err_code);
      #if (1)
          if(get_num_of_peers() >= 2){
            err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Number of peers, over 1, disconnecting");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                               BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
          }else
      #endif
          {
            if (conn_sec_status.mitm_protected)
            {
                NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d",
                             ble_conn_state_role(p_evt->conn_handle),
                             p_evt->conn_handle,
                             p_evt->params.conn_sec_succeeded.procedure);
                paired_connection = 1;
        #if defined(WAKEUP_NOPAIRED)
                paired_once = true;
        #endif
                if(id_config == ID_ONOFF){
                  id_config_set(ID_ONOFF);
                }
                led1_led2_led3_onoff_500ms_2cnt_paired_connectioned();
            }
            else
            {
                // The peer did not use MITM, disconnect.
                NRF_LOG_INFO("Collector did not use MITM, disconnecting");
                err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
         }
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            NRF_LOG_INFO("Failed to secure connection. Disconnecting.");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            
#if (1)
            NRF_LOG_INFO("PM_EVT_PEERS_DELETE_SUCCEEDED");
#else
            advertising_start(false);
#endif
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

static void get_bonds_config(bool * p_erase_bonds)
{
  ret_code_t err_code;
  bsp_event_t startup_event;

  startup_event = BSP_EVENT_NOTHING;//startup_event_extract
  //startup_event = BSP_EVENT_CLEAR_BONDING_DATA;//startup_event_extract

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    int32_t temperature = 0;
    bool     erase_bonds = false;

    // Initialize.
    gpio_init();
    nrf_delay_ms(200);
    timer_init();
#if (0)
    get_bonds_config(&erase_bonds);//buttons_leds_init
#endif
#if defined(USE_UART)
    uart_init();
#endif
    log_init();
    
#if defined(BATT_ADC)
    err_code = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(err_code);

    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);
#endif

    app_buttons_init();

#ifdef USE_ADC_TIMER
    batt_adc_configure();
#else
#if defined(BATT_ADC)
    adc_configure();
#endif
#endif

#if defined(TEMP_ONBOARD_ADC)
    nrf_temp_init();
#endif

    ble_stack_init();
#if defined(BTN_PWR_ON)
    power_on();
#endif

    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
#if defined(PEER_MNG)
    peer_manager_init();
#endif

#if defined(USE_TEST)
    do_test();
    do_send_batt_adc();
#endif
   
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);

#if defined(USE_UART) 
    printf("\r\nAPMATE Start!\r\n");
#endif
    NRF_LOG_INFO("APMATE Start!");

#if defined(PEER_MNG)
    advertising_start(erase_bonds);
#else
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
#endif

    // Enter main loop.
    while(true)
    {
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        power_manage();
    }
}

/**
 * @}
 */


