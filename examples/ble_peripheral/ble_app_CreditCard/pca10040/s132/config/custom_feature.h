/**
 * Copyright (c) 2017 - 2019, APMATE
 *
 * All rights reserved.
 *
 * Author: Jaehong Park, jaehong1972@gmail.com
 */
#ifndef CUSTOM_FEATURE_H
#define CUSTOM_FEATURE_H

#define BOARD_R7

//#define DEBUG //used in app_error_weak.c
#define DEBUG_NRF_USER //used in nrf_assert.h

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

#define BTN_PWR_ON
#define APP_BTN

#define USE_NEWLED

#define BATT_ADC
#if defined(BATT_ADC)
//#define OLD_BATT_ADC
#undef OLD_BATT_ADC
#endif

#define TEMP_ONBOARD_ADC
#if defined(TEMP_ONBOARD_ADC)
//#define TEMP_BUG_FIXED
#undef TEMP_BUG_FIXED

#define USE_SD_TEMP_API
#endif

#define USE_UART
#if defined(BATT_ADC)
#undef USE_UART
#endif
#if defined(USE_UART)
#define USE_NOT_BUS_TX
#endif

#define USE_UART_LOG

#define USE_TEST

#endif