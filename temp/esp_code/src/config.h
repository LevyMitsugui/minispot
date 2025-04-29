/* 
 * -----------------------------------------------------------------------------
 * Example: Two way communication between ESP32 and Python using WIFI
 * -----------------------------------------------------------------------------
 * Author: Radhi SGHAIER: https://github.com/Rad-hi
 * -----------------------------------------------------------------------------
 * Date: 07-05-2023 (7th of May, 2023)
 * -----------------------------------------------------------------------------
 * License: Do whatever you want with the code ...
 *          If this was ever useful to you, and we happened to meet on 
 *          the street, I'll appreciate a cup of dark coffee, no sugar please.
 * -----------------------------------------------------------------------------
 */

#ifndef __CONFG_H___
#define __CONFG_H___

#define CONFIG_FREERTOS_USE_TRACE_FACILITY              1
#define CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS  1

/* Pins definitions */
#define LED_PIN                     33
#define BTN_PIN                     25

/* Communication params */
#define ACK                         "A" // acknowledgment packet
#define QUEUE_LEN                   5
#define QUEUE_LEN_RECV              10
#define MAX_BUFFER_LEN              256

/* WiFi params */
//#define WIFI_SSID                   "TP-Link_AA13"
//#define WIFI_PASSWORD               "Benfica60"

//#define WIFI_SSID                   "Gustavo"
//#define WIFI_PASSWORD               "password"

#define WIFI_SSID                   "Vodafone-324810"
#define WIFI_PASSWORD               "3tHW44E5v2hz8DJn"

/* Socket */
#define SERVER_ADDRESS              "192.168.1.99"
#define SERVER_PORT                 11111

#define  CONFIG_FREERTOS_USE_TRACE_FACILITY 1
#define  CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS 1

#endif // __CONFG_H___
