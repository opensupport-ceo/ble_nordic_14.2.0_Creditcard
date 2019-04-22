/**
 * Copyright (c) 2017 - 2019, APMATE
 *
 * All rights reserved.
 *
 * Author: Jaehong Park, jaehong1972@gmail.com
 */
#ifndef CUSTOM_FEATURE_H
#define CUSTOM_FEATURE_H

/* Currently, applied to Board R7 schematic */
#define BOARD_R7

#if defined(BOARD_R7)
#define APMATE_LED_1 0
#define APMATE_LED_2 1

#define APMATE_BTN_1 5
#define APMATE_P_CTL 15
#define APMATE_BAT_V 8
#define APMATE_OUT_CHECK 18

#define APMATE_BAT_VC 30 //AIN6
#define APMATE_V_TEMP 29 //AIN5
#else

#define APMATE_LED_1 17
#define APMATE_LED_2 18

#define APMATE_BTN_1 13
#define APMATE_BAT_V 19
#define APMATE_OUT_CHECK 20
#endif//#if defined(BOARD_R7)

/* Temp feature for debug*/
#define USE_TEMP_DEBUG
#if defined(USE_TEMP_DEBUG)
//#define DEBUG //Used only in app_error_weak.c
#define DEBUG_NRF_USER //Used only in nrf_assert.h
#endif

/* Power-on senario is applied using only one button on board */
#define BTN_PWR_ON

/* We shares only one button on our board to the button for ble-app senario
** adoped by nordic, by default 
*/
#define APP_BTN

/* Feature for new led senario using two leds */
#define USE_NEWLED

/* For reading battery adc*/
#define BATT_ADC
#if defined(BATT_ADC)
/* Currently, don't use old mechanism */
//#define OLD_BATT_ADC
#undef OLD_BATT_ADC

/* Power off on under 2.5V */
#define BATT_POWEROFF
#endif

#if defined(BATT_ADC)
/* UARTE0 over-shared with SAADC for reading battery ADC 
** So, shoud be disabled currently.
*/
#undef USE_UART
#if defined(USE_UART)
#define USE_NOT_BUS_TX
#endif
#endif

/* Onboard temerature sensor's ADC reading */
#define TEMP_ONBOARD_ADC
#if defined(TEMP_ONBOARD_ADC)
/* temporay feature for bug-fix for example project */
//#define TEMP_BUG_FIXED
#undef TEMP_BUG_FIXED

/* Currently, use SoftDdevice API for reading ADC */
#define USE_SD_TEMP_API
#endif

/* Use uart's debug log together with RTT log */
#define USE_UART_LOG

/* Temporary feature for testing custom functions. 
** Used only under development.
*/
#define USE_TEST

#endif