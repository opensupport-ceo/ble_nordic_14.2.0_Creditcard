/**
 * Copyright (c) 2017 - 2019, APMATE
 *
 * All rights reserved.
 *
 * Author: Jaehong Park, jaehong1972@gmail.com
 */
#ifndef CUSTOM_FEATURE_H
#define CUSTOM_FEATURE_H

/* Currently, not applied to Board R7 schematic */
#define BOARD_R7
#undef BOARD_R7

/* Currently, applied to Board R11 schematic */
#define BOARD_R11

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
#undef USE_NEWLED

/* Feature for led1 senario paired wirh button */
#define BTN_LED1

/* For reading battery adc*/
#define BATT_ADC
#if defined(BATT_ADC)
/* Currently, don't use old mechanism */
//#define OLD_BATT_ADC
#undef OLD_BATT_ADC

/* Power off on under 2.5V */
#define BATT_POWEROFF
//#undef BATT_POWEROFF
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
#undef USE_TEST

/* When op paired for 10 min, after wakeup */
#define WAKEUP_NOPAIRED

/* Power off when button pressed for 5 sec */
#define BTN_POWEROFF

/* peer manager */
#define PEER_MNG
#undef PEER_MNG
#if defined(PEER_MNG)
/* Use static pass-key */
#define STATIC_KEY
#endif

//#define STATIC_KEY

#define USE_CARD_LED

#define USE_NEW_TRACKER_SEARCH_PHONE
#define USE_NEW_SELFCAMERA

#define USE_NEW_POWEROFF
#endif