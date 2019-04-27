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

#if defined(BATT_ADC)
#define SAMPLES_IN_BUFFER 5
#if defined(BATT_POWEROFF)
#define CUTOFF_VAL  0x357
#endif

/* Soft_device use #0 timer for another purpose,
** So we have to use another timer block exept for #0 timer. 
*/
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);

static nrf_saadc_value_t adc_buf[2][SAMPLES_IN_BUFFER];
static uint8_t 		uart_send_data[20];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;
#endif

#if defined(TEMP_ONBOARD_ADC)
// This function contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31.
static int32_t volatile onboard_temperature = 0;
static bool temp_read_start = false;
static bool temp_read_done = false;
#endif


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "APMATE-00:11:22:33:44:55"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL               MSEC_TO_UNITS(1000, UNIT_0_625_MS)                                           /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

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
#endif
#if defined(OLD_BATT_ADC)
APP_TIMER_DEF(m_battery_timer_id);
#endif
#if defined(BATT_POWEROFF)
APP_TIMER_DEF(m_batt_adc_timer_id);
#endif
#if defined(TEMP_ONBOARD_ADC)
APP_TIMER_DEF(m_temperature_timer_id);
#endif

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */


#if defined(BTN_PWR_ON)
#define BTN1_INTERVAL1    APP_TIMER_TICKS(1000)
#define BTN1_INTERVAL2    APP_TIMER_TICKS(100)
#else
#define BTN1_INTERVAL                                             APP_TIMER_TICKS(100)
#endif
#define LED_INTERVAL1                                             APP_TIMER_TICKS(100)
#define LED_ALERT_INTERVAL                                            APP_TIMER_TICKS(100)
#if defined(USE_NEWLED)
#define LED_IDLE_INTERVAL                                             APP_TIMER_TICKS(1000)
#endif
#if defined(OLD_BATT_ADC)
#define BATTERY_INTERVAL                                          APP_TIMER_TICKS(60000)
#endif
#if defined(BATT_POWEROFF)
#define BATTERY_READ_START_INTERVAL                               APP_TIMER_TICKS(60000)
#endif
#if defined(TEMP_ONBOARD_ADC)
#define TEMPERATURE_INTERVAL                                      APP_TIMER_TICKS(10000)
#endif

#if defined(BATT_POWEROFF)
static bool check_batt_adc = false;
#endif
static uint32_t alert_cnt;
static bool alert_on = false;
static bool alert_btn_on = false;
static bool btn_release = false;
static bool m_led_timer_f = false;

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

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
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
    
      	nrf_saadc_value_t tmp_batt_adc = 0;
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

        //batt_adc = p_event->data.done.p_buffer[0];
        tmp_batt_adc = tmp_adc_sum/SAMPLES_IN_BUFFER;
#if defined(BATT_POWEROFF)
        if(check_batt_adc){
          NRF_LOG_INFO("Batt ADC: 0x%X(%d)", tmp_batt_adc, tmp_batt_adc);
        }
#else
        NRF_LOG_INFO("Batt ADC: 0x%X(%d)", tmp_batt_adc, tmp_batt_adc);
#endif

#ifdef BATT_POWEROFF
        if(check_batt_adc){
          if(tmp_batt_adc <= CUTOFF_VAL){ // When under 2.5V.
              //nrf_gpio_pin_clear(APMATE_BAT_V);
              nrf_gpio_pin_clear(APMATE_P_CTL);
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
}
     

void led_alert_start(void)
{
    uint32_t err_code;
    alert_cnt = 0;

#if defined(USE_NEWLED)
    err_code = app_timer_stop(m_led_idle_timer_id);
    APP_ERROR_CHECK(err_code);
#endif

    if(!m_led_timer_f){
        alert_on=true;	
        err_code = app_timer_start(m_led_timer_id,LED_ALERT_INTERVAL,0);
        APP_ERROR_CHECK(err_code);
    }
}


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
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
        switch(p_evt->params.rx_data.p_data[0]){
          case 0xDD: //LED start.
            led_alert_start();
            break;
          case 0xBB: //LED stop.
            alert_cnt = 100;
            break;
          case 0xAA: //OTA start.
            bootloader_start();
            break;
          default:
          break;
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

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code;

#if defined(OLD_SENARIO)
    nrf_gpio_cfg_sense_input(APMATE_BTN_1,NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_SENSE_LOW);
#endif

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
static void power_off(void)
{
  NRF_LOG_INFO("Powering off...");
  nrf_gpio_pin_clear(APMATE_P_CTL);
  sleep_mode_enter();
}
#endif

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
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            if(m_led_timer_f){
               alert_cnt = 100;
             }
#if defined(OLD_BATT_ADC)
            err_code = app_timer_start(m_battery_timer_id,BATTERY_INTERVAL,0);
            APP_ERROR_CHECK(err_code);
#endif

#if defined(TEMP_ONBOARD_ADC)
            err_code = app_timer_start(m_temperature_timer_id, TEMPERATURE_INTERVAL,0);
            APP_ERROR_CHECK(err_code);
#endif
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
#if defined(OLD_BATT_ADC)
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
#endif

#if defined(TEMP_ONBOARD_ADC)
            err_code = app_timer_stop(m_temperature_timer_id);
            APP_ERROR_CHECK(err_code);
#endif
            led_alert_start();
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
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
#if !defined (S112)
         case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t dl_params;

            // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        } break;
#endif //!defined (S112)
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

#if (0) //Later, should be verified.
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
#endif
        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
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
#if !defined(BTN_PWR_ON)
static uint32_t click_cnt, total_cnt = 0;
#endif
static void app_btn_timeout_handler(void * p_context)
{
    uint32_t err_code;
    UNUSED_PARAMETER(p_context);

#if !defined(BTN_PWR_ON)
    //Do anything.
    if(click_cnt<2){

    }
    if(total_cnt<30){
        total_cnt++;
    }else{
     if(click_cnt>=2){
      if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
          uart_send_data[0]=0x30+click_cnt/2;
          uint16_t length = 1;
          err_code = ble_nus_string_send(&m_nus, uart_send_data, &length);
        }
      }
    }

    click_cnt=0;
    total_cnt=0;
#endif
}
#endif

#if defined(BTN_PWR_ON)
static void btn_power_on_timeout_handler(void * p_context)
{
    uint32_t err_code;
    UNUSED_PARAMETER(p_context);

    if(nrf_gpio_pin_read(APMATE_BTN_1)==0){
      nrf_gpio_pin_set(APMATE_P_CTL); //Power on board.
      NRF_LOG_INFO("Power on already setted!");
#if defined(BATT_POWEROFF)
      err_code = app_timer_start(m_batt_adc_timer_id,BATTERY_READ_START_INTERVAL,0);
      APP_ERROR_CHECK(err_code);
#endif
    }else{
      power_off();
    }
}
#else
static uint32_t hold_cnt, total_cnt, click_cnt, connec_cnt;
static void btn_power_on_timeout_handler(void * p_context)
{
    uint32_t err_code;
    UNUSED_PARAMETER(p_context);
    if(click_cnt<2){
        if(nrf_gpio_pin_read(APMATE_BTN_1)==0){
                hold_cnt++;
                if(hold_cnt==30){
                        nrf_gpio_pin_clear(APMATE_LED_1);
                        nrf_gpio_pin_clear(APMATE_LED_2);
                        btn_release=true;
                }
        }else{
                hold_cnt=0;
                btn_release=true;
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
        }
    }
    if(btn_release){
        if(nrf_gpio_pin_read(APMATE_BTN_1)==0){
           err_code = app_timer_start(m_btn_timer_id,BTN1_INTERVAL,0);
           APP_ERROR_CHECK(err_code);
           return;
        }else{

         sleep_mode_enter();
        }
    }
    if(total_cnt<30){
        total_cnt++;
        if(nrf_gpio_pin_read(APMATE_BTN_1)==1){
                nrf_gpio_pin_clear(APMATE_LED_1);
                nrf_gpio_pin_clear(APMATE_LED_2);
        }
        err_code = app_timer_start(m_btn_timer_id,BTN1_INTERVAL,0);
        APP_ERROR_CHECK(err_code);
            
    }else{
        if(click_cnt>=2){
                if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
                        uart_send_data[0]=0x30+click_cnt/2;
                        uint16_t length = 1;
                        err_code = ble_nus_string_send(&m_nus, uart_send_data, &length);
                }
        }
        
        click_cnt=0;
        total_cnt=0;
        hold_cnt=0;
    }
}
#endif//#if defined(BTN_PWR_ON)

#if defined(BATT_POWEROFF)
static void batt_power_off_timeout_handler(void * p_context)
{
  check_batt_adc = true;
  NRF_LOG_INFO("Batt ADC read-check starts...");
}
#endif

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

static void led_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint32_t err_code;
    if(alert_cnt==100){
        alert_cnt=0;
        alert_on=false;
        nrf_gpio_pin_clear(APMATE_LED_1);
        nrf_gpio_pin_clear(APMATE_LED_2);
       
        if(alert_btn_on){
                alert_btn_on=false;
                uart_send_data[0]=0xEE;
        }else{			
                uart_send_data[0]=0xFF;
        }
        if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
            uint16_t length = 1;
            err_code = ble_nus_string_send(&m_nus, uart_send_data, &length);
        }
        
        return;
    }

    switch (alert_cnt++%10){
        case 0:
            nrf_gpio_pin_set(APMATE_LED_1);
            nrf_gpio_pin_set(APMATE_LED_2);
            
            break;
        case 2:
            nrf_gpio_pin_clear(APMATE_LED_1);
            nrf_gpio_pin_clear(APMATE_LED_2);
            break;

        default:
            break;
    }
    err_code = app_timer_start(m_led_timer_id,LED_INTERVAL1,0);
    APP_ERROR_CHECK(err_code);

}

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

#if defined(TEMP_ONBOARD_ADC)
static void temperature_timeout_handler(void * p_context)
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

void timer_init(void)
{
    uint32_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_btn_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            btn_power_on_timeout_handler);
    APP_ERROR_CHECK(err_code);

#if defined(BATT_POWEROFF)
    err_code = app_timer_create(&m_batt_adc_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            batt_power_off_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif

#if defined(APP_BTN)
    err_code = app_timer_create(&m_app_btn_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT, /* SINGLE_SHOT*/
                            app_btn_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif

#if defined(USE_NEWLED)
    err_code = app_timer_create(&m_led_idle_timer_id,
                            APP_TIMER_MODE_REPEATED, 
                            led_idle_timer_handler); // temp.
    APP_ERROR_CHECK(err_code);

#endif
    err_code = app_timer_create(&m_led_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT, 
                            led_timer_handler);
    APP_ERROR_CHECK(err_code);


#if defined(OLD_BATT_ADC)
    err_code = app_timer_create(&m_battery_timer_id,
                            APP_TIMER_MODE_REPEATED,
                            battery_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif

#if defined(TEMP_ONBOARD_ADC)
    err_code = app_timer_create(&m_temperature_timer_id,
                            APP_TIMER_MODE_REPEATED,
                            temperature_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif

}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void app_button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case APMATE_BTN_1:
            NRF_LOG_INFO("Button pressed...");
            nrf_gpio_pin_set(APMATE_LED_1);
            nrf_gpio_pin_set(APMATE_LED_2);
#if (1)
            if(alert_on){
                alert_cnt=100;
                alert_btn_on=true;
            }else{
              #if defined(APP_BTN)
                err_code = app_timer_start(m_app_btn_timer_id,BTN1_INTERVAL2,0);
                APP_ERROR_CHECK(err_code);
              #else
                err_code = app_timer_start(m_btn_timer_id,BTN1_INTERVAL2,0);
                APP_ERROR_CHECK(err_code);
              #endif

              #if defined(APP_BTN)
                #if !defined(BTN_PWR_ON)
                click_cnt++;
                #endif
              #else
                click_cnt++;        
                btn_release=false;
              #endif
            }
#else
            if(alert_on){
                alert_cnt=100;
                alert_btn_on=true;
            }else{
                err_code = app_timer_start(m_btn_timer_id,BTN1_INTERVAL,0);
                APP_ERROR_CHECK(err_code);
                click_cnt++;
                btn_release=false;
            }
#endif
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


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
        nrf_gpio_cfg_output(APMATE_OUT_CHECK);
       
	nrf_gpio_pin_set(APMATE_BAT_V);
        //nrf_gpio_pin_set(APMATE_P_CTL);
        nrf_gpio_pin_set(APMATE_OUT_CHECK);

	nrf_gpio_cfg_input(APMATE_BTN_1,NRF_GPIO_PIN_PULLUP);
	
        nrf_gpio_cfg_output(APMATE_LED_1);
	nrf_gpio_cfg_output(APMATE_LED_2);
	
        nrf_gpio_pin_clear(APMATE_LED_1);
	nrf_gpio_pin_clear(APMATE_LED_2);
}

#if defined(BTN_PWR_ON)
static void power_on(void)
{
  uint32_t err_code;

  if(nrf_gpio_pin_read(APMATE_BTN_1)==0){
    NRF_LOG_INFO("Power on timer start...");
    err_code = app_timer_start(m_btn_timer_id,BTN1_INTERVAL1,0);
    APP_ERROR_CHECK(err_code);
  }else{
    power_off();
  }
}
#else
void boot_init(void)
{
	

	//nrf_delay_ms(100);
	uint32_t start_btn_cnt;
	start_btn_cnt=0;
	while(nrf_gpio_pin_read(APMATE_BTN_1)==0){
		nrf_gpio_pin_set(APMATE_LED_1);
		nrf_gpio_pin_set(APMATE_LED_2);
		start_btn_cnt++;
		nrf_delay_ms(100);
		if(start_btn_cnt > 30){
			start_btn_cnt++;
			break;
		}
	}
	nrf_gpio_pin_clear(APMATE_LED_1);
	nrf_gpio_pin_clear(APMATE_LED_2);

	if(start_btn_cnt > 30){
              nrf_delay_ms(100);
              nrf_gpio_pin_set(APMATE_LED_1);
              nrf_gpio_pin_set(APMATE_LED_2);
              nrf_delay_ms(300);
              nrf_gpio_pin_clear(APMATE_LED_1);
              nrf_gpio_pin_clear(APMATE_LED_2);
              nrf_delay_ms(700);
              nrf_gpio_pin_set(APMATE_LED_1);
              nrf_gpio_pin_set(APMATE_LED_2);
              nrf_delay_ms(300);
              nrf_gpio_pin_clear(APMATE_LED_1);
              nrf_gpio_pin_clear(APMATE_LED_2);
              nrf_delay_ms(700);
              nrf_gpio_pin_set(APMATE_LED_1);
              nrf_gpio_pin_set(APMATE_LED_2);
              nrf_delay_ms(300);
              nrf_gpio_pin_clear(APMATE_LED_1);
              nrf_gpio_pin_clear(APMATE_LED_2);
	}else{
                nrf_gpio_pin_clear(APMATE_BAT_V);
             
		sleep_mode_enter();
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
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
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

    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);

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

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    int32_t temperature = 0;
    bool     erase_bonds;

    // Initialize.
    gpio_init();
    nrf_delay_ms(200);
    timer_init();
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

#if defined(BATT_ADC)
    adc_configure();
#endif

#if defined(TEMP_ONBOARD_ADC)
    nrf_temp_init();
#endif

    ble_stack_init();
#if defined(BTN_PWR_ON)
    power_on();
#else
    boot_init();//power up
#endif

    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

#if defined(USE_TEST)
    do_test();
#endif

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);

#if defined(USE_UART) 
    printf("\r\nAPMATE Start!\r\n");
#endif
    NRF_LOG_INFO("APMATE Start!");

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

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


