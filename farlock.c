/**
 * Copyright (c) 2013 - 2019, Nordic Semiconductor ASA
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

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "boards.h"
#include "nordic_common.h"
#include "app_scheduler.h"
#include "sdk_config.h"
#include "app_timer.h"
#include "app_button.h"
#include "lwip/init.h"
#include "mqtt.h"
#include "lwip/timers.h"
#include "utf8.h"
#include "nrf_platform_port.h"
#include "app_util_platform.h"
#include "iot_timer.h"
#include "ipv6_medium.h"
#include "app_pwm.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static const uint8_t hexbytes[] =
        {0x00, 0x01, 0x02, 0x03,
         0x04, 0x05, 0x06, 0x07,
         0x08, 0x09, 0x0a, 0x0b,
         0x0c, 0x0d, 0x0e, 0x0f};

static const char hexdigits[] = "0123456789abcdefABCDEF";

typedef enum
{
    APP_IPV6_IF_UP,
    APP_IPV6_IF_DOWN
} app_ipv6_state_t;

typedef enum
{
    APP_MQTT_STATE_IDLE,
    APP_MQTT_STATE_CONNECTING,
    APP_MQTT_STATE_CONNECTED,
    APP_MQTT_STATE_SUBSCRIBING,
    APP_MQTT_STATE_SUBSCRIBED
} app_mqtt_state_t;

typedef enum
{
    LEDS_INACTIVE = 0,
    LEDS_CONNECTABLE_MODE,
    LEDS_IPV6_IF_DOWN,
    LEDS_IPV6_IF_UP,
    LEDS_CONNECTED_TO_BROKER,
    LEDS_SUBSCRIBED_TO_TOPIC
} display_state_t;

typedef enum
{
    LOCK_STATE_UNLOCKED,
    LOCK_STATE_LOCKING,
    LOCK_STATE_UNLOCKING,
    LOCK_STATE_LOCKED
} lock_state_t;

typedef enum
{
    LOCK_DIRECTION_LEFT,
    LOCK_DIRECTION_RIGHT
} lock_direction_t;

typedef struct
{
    mqtt_client_t *  	p_client;
    uint8_t	*			p_utf8_name;
    uint16_t			utf8_name_len;
    app_mqtt_state_t    state;
    uint8_t				subscriber;
} mqtt_worker_t;

typedef struct {
    mqtt_worker_t *		p_worker;
    uint8_t				topic_uuid[16];
} worker_pub_param_t;

typedef struct {
    mqtt_worker_t *		p_worker;
    uint16_t			message_id;
} worker_ack_param_t;

typedef struct {
    mqtt_worker_t *		p_worker;
    uint8_t				clean_session;
} worker_con_param_t;

typedef struct {
    mqtt_worker_t *		p_worker;
} worker_sub_param_t;

typedef struct {
    mqtt_worker_t *		p_worker;
} worker_discon_param_t;

#define LED_DBG                          	BSP_LED_0_MASK
#define LED_CXN                             BSP_LED_1_MASK
#define LED_ACCESS_GRANT                    BSP_LED_2_MASK
#define LED_ACCESS_REJECT                   BSP_LED_3_MASK
#define ALL_APP_LED                        (BSP_LED_0_MASK | BSP_LED_1_MASK | \
                                            BSP_LED_2_MASK | BSP_LED_3_MASK)

#define BTN_DBG								BUTTON_1
#define BTN_PRG								BUTTON_2
#define SW_LOCK_POS							BUTTON_3
#define SW_LOCK_DIR							BUTTON_4

#define MOTOR_PWM							28
#define MOTOR_IN1							29
#define MOTOR_IN2							30
#define MOTOR_STBY							31

#define LWIP_SYS_TICK_MS                    10

#define LED_BLINK_CXN_MULT1					4
#define LED_BLINK_CXN_MULT2					2
#define LED_BLINK_INTERVAL_MS               150
#define LED_BLINK_CXN_STABLE_MULT			64
#define LED_BLINK_CXN_STABLE_COUNT			2
#define LED_BLINK_ACCESS_MULT				12
#define AUTOCONNECT_TIMER_INTERVAL_MS       1000

#define STABLE_TIMEOUT_MS					3000
#define IDLE_RESET_TIMEOUT_MS				120000

#define AUTO_PUBLISH_TIMEOUT_MS				300000

#define ACTUATE_LEFT_TIMEOUT_MS				450
#define ACTUATE_RIGHT_TIMEOUT_MS			350

#define UUID_STRLEN							36														/**< Length of UUID (not including \0 ) */

#define SCHED_MAX_EVENT_DATA_SIZE           sizeof(worker_pub_param_t)                              /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    128                                                     /**< Maximum number of events in the scheduler queue. */

#define DEAD_BEEF                           0xDEADBEEF                                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define EUI_48_ADDR_SIZE					6

#define APP_ENABLE_LOGS                     1                                                       /**< Enable logs in the application. */

#if (APP_ENABLE_LOGS == 1)

#define APPL_LOG  NRF_LOG_INFO
#define APPL_DUMP NRF_LOG_RAW_HEXDUMP_INFO
#define APPL_ADDR IPV6_ADDRESS_LOG

#else // APP_ENABLE_LOGS

#define APPL_LOG(...)
#define APPL_DUMP(...)
#define APPL_ADDR(...)

#endif // APP_ENABLE_LOGS

#define APP_MQTT_BROKER_PORT                1883                                                    /**< Port number of MQTT Broker being used. */
#define APP_MQTT_BROKER_PORT_SECURE         8883
#define APP_MQTT_SUBSCRIPTION_PKT_ID        10                                                      /**< Unique identification of subscription, can be any unsigned 16 bit integer value. */

#define DEVICE_TYPE_ID 						79
#define DEVICE_ID_SIZE_BIN					7

APP_PWM_INSTANCE(PWM1, 1);

static const char state_locked_str[] = "3";
static const char state_unlocked_str[] = "0";
static const char state_locking_str[] = "1";
static const char state_unlocking_str[] = "2";

static const wchar_t                        state_topic[] = L"lock/state";
static const wchar_t                        state_request_topic[] = L"lock/getstate";
static const wchar_t						m_topic_prefix[] = L"/i/#";
static const wchar_t						m_pub_prefix[] = L"/o/";
static char username_utf8_buf[(EUI_48_ADDR_SIZE*8) + (sizeof(DEVICE_TYPE_ID)*8) + 1];
static char topic_utf8_buf[sizeof(m_topic_prefix) + sizeof(state_topic) + (EUI_48_ADDR_SIZE*8) + (sizeof(DEVICE_TYPE_ID)*8) + 1];
static char client_id_utf8_buf[(EUI_48_ADDR_SIZE*8) + (sizeof(DEVICE_TYPE_ID)*8) + 1];
static char pub_utf8_buf[((EUI_48_ADDR_SIZE*8) + (sizeof(DEVICE_TYPE_ID)*8)) + sizeof(m_pub_prefix) + sizeof(state_topic) + 1  + (UUID_STRLEN * 4) + 1];

static const uint32_t						device_id_strlen = (DEVICE_ID_SIZE_BIN * 2) + ((sizeof(m_topic_prefix)-4) / 4) - 1;

APP_TIMER_DEF(m_iot_timer_tick_src_id);                                                             /**< System Timer used to service CoAP and LWIP periodically. */

APP_TIMER_DEF(m_pwm_timer_src_id);																	/**< Timer used to provide timeout for PWM actions. */

eui64_t                                     eui64_local_iid;                                        /**< Local EUI64 value that is used as the IID for*/
eui48_t                   					ipv6_medium_eui48;
static ipv6_medium_instance_t               m_ipv6_medium;
static mqtt_client_t                        m_sub_mqtt_client;                                      /**< MQTT Client instance reference provided by the MQTT module. */
static display_state_t						m_prev_display_state = LEDS_INACTIVE;
static display_state_t                      m_display_state = LEDS_INACTIVE;                        /**< Board LED display state. */
static app_ipv6_state_t                     m_ipv6_state = APP_IPV6_IF_DOWN;

static lock_state_t                         m_lock_state;
static lock_direction_t						m_lock_direction;

static lock_state_t							m_pending_lock_state;

static uint16_t                             m_message_counter = 1;

static uint32_t 							idle_time = 0;
static uint32_t 							idle_start_time = 0;
static uint32_t								stable_time = 0;
static uint32_t								stable_start_time = 0;
static uint32_t								led_cxn_blink_count = 0;
static uint16_t								led_cxn_stable_blink_count = 0;
static bool									led_cxn_stable = false;
static bool									led_cxn_stable_blinkon = false;
static int8_t								led_access_blink_count = 0;
eui48_t 									ipv6_medium_eui48;

static ipv6_addr_t ipv6_broker_addr =
        { .u8 = { 0xfe, 0x80, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00, } };

static bool									led_dbg_on = false;
//#define FAR_SECURE_ENABLED 0
#if (FAR_SECURE_ENABLED == 1)

static const char tls_cert[] = "-----BEGIN CERTIFICATE-----\n"
                               "MIIDaTCCAVGgAwIBAgIBBDANBgkqhkiG9w0BAQsFADAfMR0wGwYDVQQDDBRTcG9v\n"
                               "aGFwcHNHYXRld2F5TVFDQTAeFw0xODA4MDMwNjE2MDZaFw0xOTA4MDMwNjE2MDZa\n"
                               "MCQxETAPBgNVBAMMCG5yZjUyODMyMQ8wDQYDVQQKDAZjbGllbnQwgZ8wDQYJKoZI\n"
                               "hvcNAQEBBQADgY0AMIGJAoGBAPqHLMmMZEuhdemX/uA9iN9DnHsbl5Wg6XgEXhWw\n"
                               "Oe7TwyJPa+9y+ClMnvBqHm6u+mo5ZIzBaf0MVlQG7x6jy1eHWTzie0+vhFCoBAP+\n"
                               "aOhfvztpBx7ffBaYesnsI5RpS1tJcLHHHWTWu4I1oIKLCGA4VjFAc690e/bS8Kcd\n"
                               "Uqb9AgMBAAGjLzAtMAkGA1UdEwQCMAAwCwYDVR0PBAQDAgWgMBMGA1UdJQQMMAoG\n"
                               "CCsGAQUFBwMCMA0GCSqGSIb3DQEBCwUAA4ICAQAyjoYPFqyLNbZbRHB3KUBJbWv9\n"
                               "E16d9X95ZsfobyCrPhn6fbUbIe6rCPdJhehYtx5YG7oWcx6UN+B9JNd4UhIT1j87\n"
                               "70sxH5EgOVx+ZbygO97zfK5S9S8QpiokvuUTCq2DWXjonFAN/9W8yjxpeRkj6dtE\n"
                               "zA6zwdNEbpmefRCL5u8SQS7uzc0LiIkHyeP0OkaTacHTGy2VWNISJeHQk3L+aPcA\n"
                               "DPMDGqS2EllTuo4RAsUDYaV6hljvWkiwjPl/8CEyz5QmIw/SpZLPo/4wv9V9aNF3\n"
                               "kFmpWLCWpZLPoJLKOaEOj45ALsrQcKwHZdPUF7NxHUVG7wkJD8b6nC5Dxa3Oz/gC\n"
                               "nllkdq90Vmeidl5KNel/TVAsyu2/RwEAFEaoo6XbgwClh0h62bMZHgQjbnWyl1LR\n"
                               "uhN0+6BnjlS5IXhVhDtjU0H/JHqksQ6BbvGbht3P+Jw0SyHvRpySloREXhcFf+p1\n"
                               "Il2NIZf1tlOKwohF4V5oAqZjQDPvswSeJMQW74dpbNMKXckgIoW6hsXtzcDvKWUt\n"
                               "vNkTGgVC+ZpxZ1wSAmXBxfTJN7pHSQXayLnTmofy7GsyEjjWWZQ1yzfz6EVeAQ+0\n"
                               "36Px1WbyYA1WMl7+XcsKBvPOu3yRX4+nh3TuowigFw5k1cMw1re3zM7zzvnyJK9P\n"
                               "cIoi7PNMxtSLMrjrNQ==\n"
                               "-----END CERTIFICATE-----\n";

//pkcs7 format
static const uint8_t tls_key[] = "-----BEGIN RSA PRIVATE KEY-----\n"
                                 "MIICXgIBAAKBgQD6hyzJjGRLoXXpl/7gPYjfQ5x7G5eVoOl4BF4VsDnu08MiT2vv\n"
                                 "cvgpTJ7wah5urvpqOWSMwWn9DFZUBu8eo8tXh1k84ntPr4RQqAQD/mjoX787aQce\n"
                                 "33wWmHrJ7COUaUtbSXCxxx1k1ruCNaCCiwhgOFYxQHOvdHv20vCnHVKm/QIDAQAB\n"
                                 "AoGBAKpCUYrL8aGAonzVQm8tAqcQ8PitYNBcoi5hcXt133lSyyu3JBiAXuzaWK30\n"
                                 "wKfiHOcyCAasr5Y82zKR39JH6LlEVbug/Sye+6nr3Ria/2vy9AGXUvJJwx4gaI5L\n"
                                 "CQabr00ENuXZdFFtS5Vd+6B0tNEtpGBJSssE1363CKK8S2zBAkEA/o6lizIXPZPA\n"
                                 "sJCJe2MnPSndeeGLjh+1wZmvAVvgOlhQvAre7O998XWlj+1gy685lXs45SaFc+8s\n"
                                 "ynsikGwcMQJBAPvyrp2Db6hCn7Trp/EQO6EjXkV5p/WxbEY//bcMXmfCyN6dP8DW\n"
                                 "H2X+gtFdl0mXkofa2Y/eveliLAkGrF8RII0CQCqdAJNzuDZDtL8aAxnXuGrhxkSw\n"
                                 "ACezoT6elpBYrAm4XRONklIBqYixVBzq9QhD9hTTAuxBbZfB0zK2OEwK/RECQQD1\n"
                                 "xxxIVDvMbJHXsDu4khlZbGM+axtKNrZlIW+j7dD6b638XHIg78DZgpqjGyXGiLJN\n"
                                 "DVVHYAbrcPV8KKJmaxLRAkEA2ZVNTwHL6o5l/TyaP4mlPzKaWl0jRjInNQqTIC3/\n"
                                 "wkDQxScd9OYX/btFwpwHEjyNvxK/mupKsTBT3XQrYfbu6g==\n"
                                 "-----END RSA PRIVATE KEY-----\n";


static const uint8_t ca_cert[] = "-----BEGIN CERTIFICATE-----\n"
                                 "MIIE3jCCAsagAwIBAgIJAId57WmG+7boMA0GCSqGSIb3DQEBCwUAMB8xHTAbBgNV\n"
                                 "BAMMFFNwb29oYXBwc0dhdGV3YXlNUUNBMB4XDTE4MDYxNjIyNDE0NFoXDTM4MDYx\n"
                                 "MTIyNDE0NFowHzEdMBsGA1UEAwwUU3Bvb2hhcHBzR2F0ZXdheU1RQ0EwggIiMA0G\n"
                                 "CSqGSIb3DQEBAQUAA4ICDwAwggIKAoICAQCohIvxu85TvGzd93hXbZwVx8LHrCTR\n"
                                 "HjkKL80HLLqu0yABTfCYGOiXkk4TKyHeWjcTXd7RkFnTPDe9GMOQX/8rm3duSy8S\n"
                                 "XQEWEJWYOU2u72hkbyrl0aeyhMkjGfbVONmQcqq5ELFQdG3TcIALK25fvnli4zMa\n"
                                 "SeuIk1RmpxVIcGnKFSok+n5nj+VSJi7UPBjfz5JNYAhIUOGsyfQ9Klk9DV0aXigw\n"
                                 "dMG5VCYjLMdrfBsCHdk/IyQ16SsfytDmEW1zTOotZxp6IxaecV9ltgXthDctKJIC\n"
                                 "TYQ+Ox1owAKvVzpqX73B35fV8w5TOSQA8c2q/HhTZWcVS2fAs4HQ3raQEZLmktio\n"
                                 "OAoIbVBdKwXne39fj/aAHYLvrT8FDzICTBfChXxfPBD/Jjp5m0jhpWWQGxKIDg4z\n"
                                 "7qombMgzxbw0HI+wE77hzxIzYjpptLMoU85drBoiOgXuhIzyxHeyzbQ4ptzLfqT2\n"
                                 "pHg31LqNTihfpngtD10oCA64vMGeTnGfIudZgkaiBsmeo4s0tQlvb76++fvzrdPN\n"
                                 "3VVrsybkDhzb5eW4ut4lylh2LME9BMDDhGjyx/aQYVNFA7jO0JZ82gITiCFViZnV\n"
                                 "/Nnfp/kEWEgvZCQ8RN9A+SULj+nzfABjaN+OjiJdrQUSSQfdQgBYOcDduSp3GTIR\n"
                                 "Teer8Def3LMlOwIDAQABox0wGzAMBgNVHRMEBTADAQH/MAsGA1UdDwQEAwIBBjAN\n"
                                 "BgkqhkiG9w0BAQsFAAOCAgEANzp9Ocnlyw09e/sHFDIm3s35pGi1BC3J9sM49/+8\n"
                                 "cvyli9FbhUfBfNqBzx+J/SK2lYw7qxXfi/xHm2Un9/u5r6hSKHuYgd0KPYGklXzF\n"
                                 "HqigsisFClIB2CQwuXPzHiN4l+ttcf9xt8CWAdi1aSMqeHFxBHI/ooaEwxFA02dw\n"
                                 "gUp6wh37oymAw3MLnbt7vdtGyFdDTt6WZrL5Hv1lX7PWqp/L9D4xCJ1eoZ1+PO4/\n"
                                 "HPcOitkUYz85CDq8x9VLhxFWH7rBY770uts57OMjG6ec78lFDnFOztX+nUXH1WKr\n"
                                 "/2++T9Pe31H6iHcxl1v5FjfETza63J7rL9y/LEiYdGU5UnRjZ4Z5wmq2otJF3aDP\n"
                                 "u2PJuIHtCOn8YKOUtUC5nvboPL3O9hd0MeWAz5rTYiCkbwSss30yFuPN5/xa+fDp\n"
                                 "OgKcwPUaQwuQsTpjzT4JKAQwM89p0aCjXkhotSGdp/Zbpm/TWClH6gmJStriAh4P\n"
                                 "WespSnF18aDjwB6bh5AsnbEfLGNJPmCFoGnyKRL87NxwPqVvz/VNMybGtGRESqAw\n"
                                 "5r8nQYsGfcHm5jU8aw3IDyjOTtM2euqOKETsw8ul9bGTQDDrskyspE8e3R8+TvCD\n"
                                 "vP/Cp1dr8iVQHaehcv80ESJdfxIFXnWImAUauzB5h4Um0IDTJzQJ2cLBziN0QlPG\n"
                                 "QGE=\n"
                                 "-----END CERTIFICATE-----\n";

static nrf_tls_certificate_t mq_certificate;

static nrf_tls_key_settings_t m_tls_keys;

#endif

static mqtt_worker_t m_subscriber = {
        .p_client = &m_sub_mqtt_client,
        .state = APP_MQTT_STATE_IDLE,
        .p_utf8_name = client_id_utf8_buf,
        .utf8_name_len = DEVICE_ID_SIZE_BIN * 2,
        .subscriber = 1
};

static mqtt_username_t m_user;
static mqtt_password_t m_pass;

static iot_interface_t * p_iot_interface;

void app_mqtt_evt_handler(mqtt_client_t * const p_client, const mqtt_evt_t * p_evt);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    // Configure application LED pins.
    LEDS_CONFIGURE(ALL_APP_LED);

    // Turn off all LED on initialization.
    LEDS_OFF(ALL_APP_LED);
}


/**@brief Timer callback used for controlling board LEDs to represent application state.
 *
 * @param[in]   wall_clock_value   The value of the wall clock that triggered the callback.
 */
static void blink_timeout_handler(iot_timer_time_in_ms_t wall_clock_value)
{
    UNUSED_PARAMETER(wall_clock_value);

    if (m_display_state != m_prev_display_state)
    {
        led_cxn_blink_count = 0;
    }

    m_prev_display_state = m_display_state;

    if (m_display_state == LEDS_CONNECTABLE_MODE || m_display_state == LEDS_IPV6_IF_UP || m_display_state == LEDS_SUBSCRIBED_TO_TOPIC)
    {
        led_cxn_blink_count++;
    }
    else
    {
        led_cxn_blink_count = 0;
    }

    if (led_dbg_on > 0)
    {
        LEDS_ON(LED_DBG);
    }
    else
    {
        LEDS_OFF(LED_DBG);
    }

    switch (m_display_state)
    {
        case LEDS_INACTIVE:
        {
            LEDS_OFF(ALL_APP_LED);
            break;
        }
        case LEDS_CONNECTABLE_MODE:
        {
            if (led_cxn_blink_count >= LED_BLINK_CXN_MULT1)
            {
                LEDS_INVERT(LED_CXN);
                led_cxn_blink_count = 0;
            }
            break;
        }
        case LEDS_IPV6_IF_DOWN:
        {
            LEDS_OFF(ALL_APP_LED);
            break;
        }
        case LEDS_IPV6_IF_UP:
        {
            if (led_cxn_blink_count >= LED_BLINK_CXN_MULT2)
            {
                LEDS_INVERT(LED_CXN);
                led_cxn_blink_count = 0;
            }
            break;
        }
        case LEDS_CONNECTED_TO_BROKER:
        {
            LEDS_INVERT(LED_CXN);
            break;
        }
        case LEDS_SUBSCRIBED_TO_TOPIC:
        {
            if (led_cxn_stable > 0)
            {
                if (led_cxn_blink_count >= LED_BLINK_CXN_STABLE_MULT)
                {
                    if (led_cxn_stable_blinkon)
                    {
                        LEDS_OFF(LED_CXN);
                        led_cxn_stable_blinkon = false;
                        if (led_cxn_stable_blink_count == LED_BLINK_CXN_STABLE_COUNT)
                        {
                            led_cxn_blink_count = 0;
                            led_cxn_stable_blink_count = 0;
                        }
                        else
                        {
                            led_cxn_blink_count--;
                        }
                    }
                    else
                    {
                        LEDS_ON(LED_CXN);
                        led_cxn_stable_blinkon = true;
                        led_cxn_stable_blink_count++;
                        led_cxn_blink_count--;
                    }
                }
            }
            else
            {
                LEDS_ON(LED_CXN);
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if (led_access_blink_count != 0)
    {
        if (led_access_blink_count > 0)
        {
            LEDS_ON(LED_ACCESS_GRANT);
            LEDS_OFF(LED_ACCESS_REJECT);
            led_access_blink_count--;
        }
        else
        {
            LEDS_OFF(LED_ACCESS_GRANT);
            LEDS_ON(LED_ACCESS_REJECT);
            led_access_blink_count++;
        }
    }
    else
    {
        LEDS_OFF(LED_ACCESS_GRANT);
        LEDS_OFF(LED_ACCESS_REJECT);
    }
}

static void bin_to_hex_str(char * to, const uint8_t * from, const uint32_t size) {
    uint8_t j = 0;
    for (uint8_t i = 0; i < size; i++) {
        to[j++] = hexdigits[from[i] >> 4];
        to[j++] = hexdigits[from[i] & 0xf];
    }
    to[j] = '\0';
}

static void connect_to_broker(void * p_event_data, uint16_t event_size) {
    worker_con_param_t con_param = *((worker_con_param_t *)p_event_data);
    if (con_param.p_worker->state == APP_MQTT_STATE_IDLE)
    {
        mqtt_client_t * client = con_param.p_worker->p_client;
        mqtt_client_init(client);

        uint8_t deviceIdBin[DEVICE_ID_SIZE_BIN];

        deviceIdBin[0] = DEVICE_TYPE_ID;

        for (uint8_t i = 0; i < sizeof(eui48_t); i++) {
            deviceIdBin[DEVICE_ID_SIZE_BIN-1-i] = ipv6_medium_eui48.identifier[i];
        }

        char usernameStr[(sizeof(deviceIdBin)*2)+1];

        bin_to_hex_str(usernameStr, deviceIdBin, sizeof(deviceIdBin));

        wchar_t username_wcs[sizeof(usernameStr)];

        mbstowcs(username_wcs, usernameStr, sizeof(usernameStr));

        u8_toutf8(username_utf8_buf, sizeof(username_utf8_buf), username_wcs, -1);

        m_user.p_utf_str = (uint8_t *)username_utf8_buf;
        m_user.utf_strlen = wcslen(username_wcs);

        m_pass.p_utf_str = (uint8_t *)username_utf8_buf;
        m_pass.utf_strlen = wcslen(username_wcs);

        memset(ipv6_broker_addr.u8 + 2, 0, 14);

        if (p_iot_interface) {

            memcpy(ipv6_broker_addr.u8 + EUI_64_ADDR_SIZE, p_iot_interface->peer_addr.identifier, EUI_64_ADDR_SIZE);

#if (BLE_6LOWPAN_LEGACY_MODE == 1)
            ipv6_broker_addr.u8[EUI_64_ADDR_SIZE] |= 0x02;
#endif

            memcpy(client->broker_addr.u8, ipv6_broker_addr.u8, IPV6_ADDR_SIZE);

            client->protocol_version = MQTT_VERSION_3_1_1;

#if defined(FAR_SECURE_ENABLED)

            mq_certificate.p_certificate = &tls_cert[0];
            mq_certificate.certificate_len = strlen(tls_cert) + 1;
            mq_certificate.p_private_key = &tls_key[0];
            mq_certificate.private_key_len = strlen(tls_key) + 1;

            m_tls_keys.p_own_certificate = &mq_certificate;
            m_tls_keys.p_ca_cert_pem = &ca_cert[0];
            m_tls_keys.ca_cert_pem_len = strlen(ca_cert) + 1;
            m_tls_keys.p_psk = NULL;
            m_tls_keys.p_raw_key = NULL;

            client->broker_port = APP_MQTT_BROKER_PORT_SECURE;
            client->transport_type = MQTT_TRANSPORT_SECURE;
            client->p_security_settings = &m_tls_keys;
#else
            client->broker_port = APP_MQTT_BROKER_PORT;
            client->transport_type = MQTT_TRANSPORT_NON_SECURE;
            client->p_security_settings = NULL;
#endif
            client->evt_cb = app_mqtt_evt_handler;
            client->client_id.p_utf_str = con_param.p_worker->p_utf8_name;
            client->client_id.utf_strlen = con_param.p_worker->utf8_name_len;
            client->p_user_name = &m_user;
            client->p_password = &m_pass;
            client->clean_session = con_param.clean_session;

            uint32_t err_code = mqtt_connect(client);

            if (err_code == NRF_SUCCESS) {
                APPL_LOG("[APPL]: CONNECTING - %d", err_code);

                con_param.p_worker->state = APP_MQTT_STATE_CONNECTING;
            } else {
                APPL_LOG("[APPL]: ERROR CONNECTING - %d", err_code);
            }
        } else {

            APPL_LOG("[APPL]: ERROR CONNECTING - no interface");

        }
    }
}

static void subscribe_to_topic(void * p_event_data, uint16_t event_size) {
    worker_sub_param_t sub_param = *((worker_sub_param_t *)p_event_data);
    if (sub_param.p_worker->state == APP_MQTT_STATE_CONNECTED)
    {
        uint8_t deviceIdBin[DEVICE_ID_SIZE_BIN];

        deviceIdBin[0] = DEVICE_TYPE_ID;

        for (uint8_t i = 0; i < sizeof(eui48_t); i++) {
            deviceIdBin[DEVICE_ID_SIZE_BIN-1-i] = ipv6_medium_eui48.identifier[i];
        }

        char topicStr[(sizeof(deviceIdBin)*2)+1];

        bin_to_hex_str(topicStr, deviceIdBin, sizeof(deviceIdBin));

        wchar_t topic_wcs[sizeof(topicStr) + wcslen(m_topic_prefix)];

        mbstowcs(topic_wcs, topicStr, sizeof(topicStr));

        wcscat(topic_wcs, m_topic_prefix);

        u8_toutf8(topic_utf8_buf, sizeof(topic_utf8_buf), topic_wcs, -1);

        mqtt_utf8_t topic_utf8 = { .p_utf_str = (uint8_t *)topic_utf8_buf, .utf_strlen = wcslen(topic_wcs) };

        mqtt_topic_t topic =
                {
                        .topic = topic_utf8,
                        .qos = MQTT_QoS_1_ATLEAST_ONCE
                };

        const mqtt_subscription_list_t subscription_list =
                {
                        .p_list     = &topic,
                        .list_count = 1,
                        .message_id = APP_MQTT_SUBSCRIPTION_PKT_ID
                };

        uint32_t err_code = mqtt_subscribe(sub_param.p_worker->p_client, &subscription_list);
        if (err_code == NRF_SUCCESS) {
            APPL_LOG("[APPL]: SUBSCRIBING");
            sub_param.p_worker->state = APP_MQTT_STATE_SUBSCRIBING;
        } else {
            APPL_LOG("[APPL]: ERROR SUBSCRIBING - %d", err_code);
        }
    }
}

static void bin_to_uuid_str(char * to, const uint8_t * from, const uint32_t size) {
    uint8_t j = 0;
    for (uint8_t i = 0; i < size; i++) {
        to[j++] = hexdigits[from[i] >> 4];
        to[j++] = hexdigits[from[i] & 0xf];
        if (i == 3 || i == 5 || i == 7 || i == 9) {
            to[j++] = '-';
        }
    }
    to[j] = '\0';
}

static uint8_t * get_lock_state_str()
{
    switch (m_lock_state)
    {
        case LOCK_STATE_UNLOCKED:
        {
            return state_unlocked_str;
            break;
        }
        case LOCK_STATE_UNLOCKING:
        {
            return state_unlocking_str;
            break;
        }
        case LOCK_STATE_LOCKING:
        {
            return state_locking_str;
            break;
        }
    }

    return state_locked_str;
}

static uint8_t is_empty_uuid(wchar_t * p_uuid) {
    wchar_t * c = p_uuid;
    while (*c) {
        if (*c != L'0' && *c != L'-') {
            return 0;
        }
        c++;
    }
    return 1;
}

static void publish_state(void * p_event_data, uint16_t event_size)
{
    worker_pub_param_t pub_param = *((worker_pub_param_t *)p_event_data);

    APPL_LOG("[APPL] publish_state %d", pub_param.p_worker->state)

    if (pub_param.p_worker->state == APP_MQTT_STATE_CONNECTED
        || pub_param.p_worker->state == APP_MQTT_STATE_SUBSCRIBED) {

        uint8_t * msg = get_lock_state_str();

        mqtt_publish_param_t param;

        char topic[UUID_STRLEN + 1];

        bin_to_uuid_str(topic, pub_param.topic_uuid, 16);

        wchar_t uuidStr[UUID_STRLEN + 1];

        mbstowcs(uuidStr, topic, UUID_STRLEN + 1);

        uint8_t deviceIdBin[DEVICE_ID_SIZE_BIN];

        deviceIdBin[0] = DEVICE_TYPE_ID;

        for (uint32_t i = 0; i < sizeof(eui48_t); i++) {
            deviceIdBin[DEVICE_ID_SIZE_BIN-1-i] = ipv6_medium_eui48.identifier[i];
        }

        char topicStr[(sizeof(deviceIdBin)*2)+1];

        bin_to_hex_str(topicStr, deviceIdBin, sizeof(deviceIdBin));

        wchar_t topic_wcs[sizeof(topicStr)];

        mbstowcs(topic_wcs, topicStr, sizeof(topicStr));

        int resp_topic_len = wcslen(topic_wcs) + wcslen(m_pub_prefix) + wcslen(state_topic);

        if (is_empty_uuid(uuidStr) == 0) {
            resp_topic_len += (wcslen(uuidStr) + 1);
        }

        wchar_t response_topic[resp_topic_len + 1]; // +1 for the \0

        wcscpy(response_topic, topic_wcs);
        wcscat(response_topic, m_pub_prefix);
        wcscat(response_topic, state_topic);

        if (is_empty_uuid(uuidStr) == 0) {
            wcscat(response_topic, L"/");
            wcscat(response_topic, uuidStr);
        }

        u8_toutf8(pub_utf8_buf, sizeof(pub_utf8_buf), response_topic, -1);

        param.message.topic.qos              = MQTT_QoS_1_ATLEAST_ONCE;
        param.message.topic.topic.p_utf_str  = (uint8_t *)pub_utf8_buf;
        param.message.topic.topic.utf_strlen = wcslen(response_topic);
        param.message.payload.p_bin_str      = msg,
                param.message.payload.bin_strlen     = strlen(msg);
        param.message_id                     = m_message_counter;
        param.dup_flag                       = 0;
        param.retain_flag                    = 0;

        uint32_t err_code = mqtt_publish(pub_param.p_worker->p_client, &param);

        if (err_code == NRF_SUCCESS) {
            APPL_LOG("successful publish");
            m_message_counter += 1;
        } else {
            APPL_LOG("unsuccessful publish err_code = %d", err_code);
            app_sched_event_put(&pub_param, sizeof(worker_pub_param_t), publish_state);
        }
    }
}

static void acknowledge_message(void * p_event_data, uint16_t event_size) {
    worker_ack_param_t ack_param = *((worker_ack_param_t *)p_event_data);

    uint32_t err_code = mqtt_publish_ack(ack_param.p_worker->p_client, (mqtt_puback_param_t * )&ack_param.message_id);
    if (err_code != NRF_SUCCESS) {
        app_sched_event_put(&ack_param, sizeof(worker_ack_param_t), acknowledge_message);
    }
}

//static void disconnect_from_broker(void * p_event_data, uint16_t event_size) {
//	worker_discon_param_t discon_param = *((worker_discon_param_t *)p_event_data);
//    if ((discon_param.p_worker->state == APP_MQTT_STATE_CONNECTED) ||
//        (discon_param.p_worker->state == APP_MQTT_STATE_SUBSCRIBED))
//    {
//        UNUSED_VARIABLE(mqtt_disconnect(discon_param.p_worker->p_client));
//    }
//}

static void queue_state_publish() {

    APPL_LOG("[APPL] queue_state_publish()")

    worker_pub_param_t pub_param;

    pub_param.p_worker = &m_subscriber;

    memset(pub_param.topic_uuid, 0, 16);

    app_sched_event_put(&pub_param, sizeof(worker_pub_param_t), publish_state);
}

static void autoconnect_handler(mqtt_worker_t * worker, char * label_str) {
    if (m_ipv6_state == APP_IPV6_IF_UP) {
        switch(worker->state) {
            case APP_MQTT_STATE_IDLE:
            {
                APPL_LOG ("[APPL]: autoconnect_timeout_handler [%s] MQTT STATE IDLE\r\n", label_str);
                worker_con_param_t con_param = {
                        .p_worker = worker,
                        .clean_session = 1
                };
                app_sched_event_put(&con_param, sizeof(worker_con_param_t), connect_to_broker);
                break;
            }
            case APP_MQTT_STATE_CONNECTED:
            {
                APPL_LOG ("[APPL]: autoconnect_timeout_handler [%s] MQTT STATE CONNECTED\r\n", label_str);
                if (worker->subscriber > 0) {
                    worker_sub_param_t sub_param = {
                            .p_worker = worker
                    };
                    app_sched_event_put(&sub_param, sizeof(worker_sub_param_t), subscribe_to_topic);
                }
                break;
            }
        }
    }
}

static void set_display_state(void) {
    if (m_ipv6_state == APP_IPV6_IF_UP) {
        switch(m_subscriber.state) {
            case APP_MQTT_STATE_IDLE:
            case APP_MQTT_STATE_CONNECTING:
            {
                m_display_state = LEDS_IPV6_IF_UP;
                break;
            }
            case APP_MQTT_STATE_CONNECTED:
            case APP_MQTT_STATE_SUBSCRIBING:
            {
                m_display_state = LEDS_CONNECTED_TO_BROKER;
                break;
            }
            case APP_MQTT_STATE_SUBSCRIBED:
            {
                m_display_state = LEDS_SUBSCRIBED_TO_TOPIC;
                break;
            }
            default:
            {
                m_display_state = LEDS_IPV6_IF_UP;
                break;
            }
        }
    }
}

static void autoconnect_timeout_handler(iot_timer_time_in_ms_t wall_clock_value)
{
    if (m_subscriber.state != APP_MQTT_STATE_SUBSCRIBED)
    {
        stable_time = 0;
        stable_start_time = 0;
        if (idle_start_time == 0) idle_start_time = wall_clock_value;
        idle_time = wall_clock_value - idle_start_time;
        led_cxn_stable = false;
    } else {
        idle_time = 0;
        idle_start_time = 0;
        if (stable_start_time == 0) {
            stable_start_time = wall_clock_value;
            queue_state_publish();
        }
        stable_time = wall_clock_value - stable_start_time;
    }

    if (idle_time <= IDLE_RESET_TIMEOUT_MS)
    {
        autoconnect_handler(&m_subscriber, "SUB");

        if (stable_time >= STABLE_TIMEOUT_MS)
        {
            if (led_cxn_stable <= 0)
            {
                led_cxn_stable_blinkon = false;
                LEDS_OFF(LED_CXN);
                led_cxn_blink_count = LED_BLINK_CXN_STABLE_MULT;
            }
            led_cxn_stable = true;

            if (stable_time >= AUTO_PUBLISH_TIMEOUT_MS) {
                stable_start_time = wall_clock_value - STABLE_TIMEOUT_MS;
                queue_state_publish();
            }
        }
        else
        {
            led_cxn_stable = false;
        }
    }
    else
    {
        APPL_LOG("Idle for too long, issuing system reset.");
        NVIC_SystemReset();
    }

    set_display_state();
}

static void actuate_motor(uint16_t speed, lock_direction_t direction)
{
    nrf_gpio_pin_set(MOTOR_STBY);

    if (direction == LOCK_DIRECTION_LEFT)
    {
        nrf_gpio_pin_clear(MOTOR_IN1);
        nrf_gpio_pin_set(MOTOR_IN2);
    }
    else
    {
        nrf_gpio_pin_set(MOTOR_IN1);
        nrf_gpio_pin_clear(MOTOR_IN2);
    }

    app_pwm_channel_duty_set(&PWM1, 0, speed);

    if (speed == 0)
    {
        nrf_gpio_pin_clear(MOTOR_STBY);
    }
}

static void lock()
{
    APPL_LOG("lock operation issued");
    if (m_lock_state == LOCK_STATE_UNLOCKED)
    {
        APPL_LOG("commencing lock operation");
        led_access_blink_count = 5;
        m_lock_state = LOCK_STATE_LOCKING;
        m_pending_lock_state = LOCK_STATE_LOCKED;
        actuate_motor(100, m_lock_direction);
        app_timer_start(m_pwm_timer_src_id,
                        APP_TIMER_TICKS(m_lock_direction == LOCK_DIRECTION_LEFT ? ACTUATE_LEFT_TIMEOUT_MS : ACTUATE_RIGHT_TIMEOUT_MS),
                        &m_pending_lock_state);
    }
    else
    {
        APPL_LOG("aborting lock operation");
    }
}

static void unlock()
{
    APPL_LOG("unlock operation issued");
    if (m_lock_state == LOCK_STATE_LOCKED)
    {
        APPL_LOG("commencing unlock operation");
        led_access_blink_count = 5;
        m_lock_state = LOCK_STATE_UNLOCKING;
        m_pending_lock_state = LOCK_STATE_UNLOCKED;
        actuate_motor(100, m_lock_direction == LOCK_DIRECTION_LEFT ? LOCK_DIRECTION_RIGHT : LOCK_DIRECTION_LEFT);
        app_timer_start(m_pwm_timer_src_id,
                        APP_TIMER_TICKS(m_lock_direction == LOCK_DIRECTION_LEFT ? ACTUATE_RIGHT_TIMEOUT_MS : ACTUATE_LEFT_TIMEOUT_MS),
                        &m_pending_lock_state);
    }
    else
    {
        APPL_LOG("aborting unlock operation");
    }
}

static void set_lock_state_from_pos_switch(void)
{
    if (app_button_is_pushed(2) > 0)
    {
        if (m_lock_direction == LOCK_DIRECTION_RIGHT)
        {
            APPL_LOG("set lock state = locked");
            m_lock_state = LOCK_STATE_LOCKED;
        }
        else
        {
            APPL_LOG("set lock state = unlocked");
            m_lock_state = LOCK_STATE_UNLOCKED;
        }
    }
    else
    {
        if (m_lock_direction == LOCK_DIRECTION_RIGHT)
        {
            APPL_LOG("set lock state = unlocked");
            m_lock_state = LOCK_STATE_UNLOCKED;
        }
        else
        {
            APPL_LOG("set lock state = locked");
            m_lock_state = LOCK_STATE_LOCKED;
        }
    }
}

static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    lock_state_t old_state = m_lock_state;
    if (button_action == APP_BUTTON_PUSH)
    {
        switch (pin_no)
        {
            case BTN_DBG:
            {
                APPL_LOG("dbg button pushed");
                led_dbg_on = true;
                break;
            }
            case BTN_PRG:
            {
                APPL_LOG("prg button pushed");
                led_access_blink_count = 5;
                break;
            }
            case SW_LOCK_DIR:
            {
                APPL_LOG("lock direction switch right");
                m_lock_direction = LOCK_DIRECTION_RIGHT;
                set_lock_state_from_pos_switch();
                break;
            }
            case SW_LOCK_POS:
            {
                APPL_LOG("lock position switch on");
                if (m_lock_direction == LOCK_DIRECTION_LEFT)
                {
                    APPL_LOG("set state to unlocked");
                    m_lock_state = LOCK_STATE_UNLOCKED;
                }
                else
                {
                    APPL_LOG("set state to locked");
                    m_lock_state = LOCK_STATE_LOCKED;
                }
                break;
            }
            default:
                break;
        }
    }
    else if (button_action == APP_BUTTON_RELEASE)
    {
        switch (pin_no)
        {
            case BTN_DBG:
            {
                APPL_LOG("dbg button released");
                led_dbg_on = false;
                queue_state_publish();
                break;
            }
            case BTN_PRG:
            {
                APPL_LOG("prg button released");
                break;
            }
            case SW_LOCK_DIR:
            {
                APPL_LOG("lock direction switch left");
                m_lock_direction = LOCK_DIRECTION_LEFT;
                set_lock_state_from_pos_switch();
                break;
            }
            case SW_LOCK_POS:
            {
                APPL_LOG("lock position switch off");
                if (m_lock_direction == LOCK_DIRECTION_LEFT)
                {
                    APPL_LOG("set state to locked");
                    m_lock_state = LOCK_STATE_LOCKED;
                }
                else
                {
                    APPL_LOG("set state to unlocked");
                    m_lock_state = LOCK_STATE_UNLOCKED;
                }
                break;
            }
            default:
                break;
        }
    }

    if (m_lock_state != old_state) {
        queue_state_publish();
    }
}

static void button_init(void)
{
    uint32_t err_code;
    static app_button_cfg_t buttons[] =
            {
                    {BTN_DBG,         false, BUTTON_PULL, button_event_handler},
                    {BTN_PRG,         false, BUTTON_PULL, button_event_handler},
                    {SW_LOCK_POS,     false, BUTTON_PULL, button_event_handler},
                    {SW_LOCK_DIR,     false, BUTTON_PULL, button_event_handler},
            };

    #define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50)

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);

    if (app_button_is_pushed(3) > 0)
    {
        APPL_LOG("set initial lock direction = right");
        m_lock_direction = LOCK_DIRECTION_RIGHT;
    }
    else
    {
        APPL_LOG("set initial lock direction = left");
        m_lock_direction = LOCK_DIRECTION_LEFT;
    }

    set_lock_state_from_pos_switch();
}

/**@brief Function for initializing IP stack.
 *
 * @details Initialize the IP Stack and its driver.
 */
static void ip_stack_init(void)
{
    uint32_t err_code;

    err_code = ipv6_medium_eui64_get(m_ipv6_medium.ipv6_medium_instance_id,
                                     &eui64_local_iid);
    APP_ERROR_CHECK(err_code);

    err_code = ipv6_medium_eui48_get(m_ipv6_medium.ipv6_medium_instance_id,
                                     &ipv6_medium_eui48);

    uint8_t deviceIdBin[DEVICE_ID_SIZE_BIN];

    deviceIdBin[0] = DEVICE_TYPE_ID;

    for (uint8_t i = 0; i < sizeof(eui48_t); i++) {
        deviceIdBin[DEVICE_ID_SIZE_BIN-1-i] = ipv6_medium_eui48.identifier[i];
    }

    char clientIdStr[(sizeof(deviceIdBin)*2)+1];

    bin_to_hex_str(clientIdStr, deviceIdBin, sizeof(deviceIdBin));

    wchar_t clientId_wcs[sizeof(clientIdStr)];

    mbstowcs(clientId_wcs, clientIdStr, sizeof(clientIdStr));

    u8_toutf8(client_id_utf8_buf, sizeof(client_id_utf8_buf), clientId_wcs, -1);

    APP_ERROR_CHECK(err_code);

    err_code = nrf_mem_init();
    APP_ERROR_CHECK(err_code);

    // Initialize LwIP stack.
    lwip_init();

    // Initialize LwIP stack driver.
    err_code = nrf_driver_init();
    APP_ERROR_CHECK(err_code);

    err_code = mqtt_init();
    APP_ERROR_CHECK(err_code);

}


/**@brief Timer callback used for periodic servicing of LwIP protocol timers.
 *        This trigger is also used in the example to trigger sending TCP Connection.
 *
 * @details Timer callback used for periodic servicing of LwIP protocol timers.
 *
 * @param[in]   wall_clock_value   The value of the wall clock that triggered the callback.
 */
static void system_timer_callback(iot_timer_time_in_ms_t wall_clock_value)
{
    UNUSED_VARIABLE(wall_clock_value);

    sys_check_timeouts();
    UNUSED_VARIABLE(mqtt_live());
}


/**@brief Function for updating the wall clock of the IoT Timer module.
 *
 * @param[in]   p_context   Pointer used for passing context. No context used in this application.
 */
static void iot_timer_tick_callback(void * p_context)
{
    UNUSED_VARIABLE(p_context);

    uint32_t err_code = iot_timer_update();

    APP_ERROR_CHECK(err_code);
}

static void pwm_timer_callback(void * p_context)
{
    UNUSED_VARIABLE(p_context);

    APPL_LOG("stopping motor operation");

    actuate_motor(0,0);

    set_lock_state_from_pos_switch();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_ERROR_CHECK(app_timer_init());

    // Create a sys timer.
    err_code = app_timer_create(&m_iot_timer_tick_src_id,
                                APP_TIMER_MODE_REPEATED,
                                iot_timer_tick_callback);

    APP_ERROR_CHECK(err_code);

    //Create PWM timer
    err_code = app_timer_create(&m_pwm_timer_src_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                pwm_timer_callback);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the IoT Timer. */
static void iot_timer_init(void)
{
    uint32_t err_code;

    static const iot_timer_client_t list_of_clients[] =
            {
                    {system_timer_callback,      	LWIP_SYS_TICK_MS},
                    {blink_timeout_handler,      	LED_BLINK_INTERVAL_MS},
                    {autoconnect_timeout_handler,  	AUTOCONNECT_TIMER_INTERVAL_MS}
            };

    // The list of IoT Timer clients is declared as a constant.
    static const iot_timer_clients_list_t iot_timer_clients =
            {
                    (sizeof(list_of_clients) / sizeof(iot_timer_client_t)),
                    &(list_of_clients[0]),
            };

    // Passing the list of clients to the IoT Timer module.
    err_code = iot_timer_client_list_set(&iot_timer_clients);
    APP_ERROR_CHECK(err_code);

    // Starting the app timer instance that is the tick source for the IoT Timer.
    err_code = app_timer_start(m_iot_timer_tick_src_id,
                               APP_TIMER_TICKS(IOT_TIMER_RESOLUTION_IN_MS),
                               NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void motor_init(void)
{
    nrf_gpio_cfg_output(MOTOR_IN1);
    nrf_gpio_pin_clear(MOTOR_IN1);
    nrf_gpio_cfg_output(MOTOR_IN2);
    nrf_gpio_pin_clear(MOTOR_IN2);
    nrf_gpio_cfg_output(MOTOR_STBY);
    nrf_gpio_pin_clear(MOTOR_STBY);
}

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    UNUSED_PARAMETER(pwm_id);
}

static void pwm_init(void)
{
    ret_code_t err_code;

    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(20L, MOTOR_PWM);

    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

    err_code = app_pwm_init(&PWM1, &pwm1_cfg, pwm_ready_callback);

    APP_ERROR_CHECK(err_code);

    app_pwm_enable(&PWM1);
}

/**@brief Function to handle interface up event. */
void nrf_driver_interface_up(iot_interface_t const * p_interface)
{
    UNUSED_PARAMETER(p_interface);

    APPL_LOG ("IPv6 Interface Up.");

    sys_check_timeouts();

    m_display_state = LEDS_IPV6_IF_UP;

    p_iot_interface = p_interface;
}


/**@brief Function to handle interface down event. */
void nrf_driver_interface_down(iot_interface_t const * p_interface)
{
    UNUSED_PARAMETER(p_interface);

    APPL_LOG ("IPv6 Interface Down.");

    p_iot_interface = NULL;

    m_display_state = LEDS_IPV6_IF_DOWN;
    m_subscriber.state = APP_MQTT_STATE_IDLE;
}

static void uuid_str_to_bin(uint8_t * to, const char * from) {
    uint8_t j = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (from[j] == '-') j++;
        int h = (int)(strchr(hexdigits, from[j]) - hexdigits);
        int l = (int)(strchr(hexdigits, from[j+1]) - hexdigits);
        if (h > 15) h -= 6;
        if (l > 15) l -= 6;
        *(to+i) = (hexbytes[h] << 4) | hexbytes[l];
        j+=2;
    }
}

static void log_mqtt_connack_result(const uint32_t result, char * label) {
    switch (result) {
        case MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
        {
            APPL_LOG ("[APPL]: >> [%s] MQTT_UNACCEPTABLE_PROTOCOL_VERSION\r\n", label);
            break;
        }
        case MQTT_IDENTIFIER_REJECTED:
        {
            APPL_LOG ("[APPL]: >> [%s] MQTT_IDENTIFIER_REJECTED\r\n", label);
            break;
        }
        case MQTT_SERVER_UNAVAILABLE:
        {
            APPL_LOG ("[APPL]: >> [%s] MQTT_SERVER_UNAVAILABLE\r\n", label);
            break;
        }
        case MQTT_BAD_USER_NAME_OR_PASSWORD:
        {
            APPL_LOG ("[APPL]: >> [%s]MQTT_BAD_USER_NAME_OR_PASSWORD\r\n", label);
            break;
        }
        case MQTT_NOT_AUTHORIZED:
        {
            APPL_LOG ("[APPL]: >> [%s] MQTT_NOT_AUTHORIZED\r\n", label);
            break;
        }
        default:
        {
            APPL_LOG ("[APPL]: >> [%s] UNKNOWN [%d]\r\n", label, result);
            break;
        }
    }
}

void app_mqtt_evt_handler(mqtt_client_t * const p_client, const mqtt_evt_t * p_evt) {
    switch(p_evt->id)
    {
        case MQTT_EVT_CONNACK:
        {
            APPL_LOG ("[APPL]: >> [SUB] MQTT_EVT_CONNACK\r\n");
            if(p_evt->result == MQTT_CONNECTION_ACCEPTED)
            {
                APPL_LOG ("[APPL]: >> [SUB] MQTT_CONNECTION_ACCEPTED\r\n");
                m_subscriber.state = APP_MQTT_STATE_CONNECTED;
            } else {
                m_subscriber.state = APP_MQTT_STATE_IDLE;
                log_mqtt_connack_result(p_evt->result, "SUB");
            }
            break;
        }
        case MQTT_EVT_PUBLISH:
        {
            APPL_LOG ("[APPL]: >> [SUB] MQTT_EVT_PUBLISH\r\n");
            if (p_evt->param.publish.message.topic.qos == MQTT_QoS_1_ATLEAST_ONCE)
            {
                worker_ack_param_t ack_param = {
                        .p_worker = &m_subscriber,
                        .message_id = p_evt->param.publish.message_id
                };
                app_sched_event_put(&ack_param, sizeof(worker_ack_param_t), acknowledge_message);
            }

            int topicSz = p_evt->param.publish.message.topic.topic.utf_strlen;

            wchar_t topicStr[topicSz+1];

            UNUSED_VARIABLE(u8_toucs(topicStr, topicSz+1, p_evt->param.publish.message.topic.topic.p_utf_str, -1));

            if (wcscmp(&topicStr[device_id_strlen], state_topic) == 0)
            {
                if (p_evt->param.publish.message.payload.bin_strlen == 1)
                {
                    // Accept binary or ASCII 0 and 1.
                    if((p_evt->param.publish.message.payload.p_bin_str[0] == 0) ||
                       (p_evt->param.publish.message.payload.p_bin_str[0] == 0x30))
                    {
                        unlock();
                    }
                    else if((p_evt->param.publish.message.payload.p_bin_str[0] == 1) ||
                            (p_evt->param.publish.message.payload.p_bin_str[0] == 0x31))
                    {
                        lock();
                    }
                }
            }
            else if (wcscmp(&topicStr[device_id_strlen], state_request_topic) == 0)
            {
                worker_pub_param_t pub_param;

                pub_param.p_worker = &m_subscriber;

                uuid_str_to_bin(pub_param.topic_uuid, p_evt->param.publish.message.payload.p_bin_str);

                app_sched_event_put(&pub_param, sizeof(worker_pub_param_t), publish_state);
            }
            break;
        }
        case MQTT_EVT_DISCONNECT:
        {
            APPL_LOG ("[APPL]: >> [SUB] MQTT_EVT_DISCONNECT\r\n");
            m_subscriber.state = APP_MQTT_STATE_IDLE;
            break;
        }
        case MQTT_EVT_PUBACK:
        {
            APPL_LOG ("[APPL]: >> [SUB] MQTT_EVT_PUBACK\r\n");
            break;
        }
        case MQTT_EVT_SUBACK:
        {
            APPL_LOG ("[APPL]: >> [SUB] MQTT_EVT_SUBACK\r\n");
            if (p_evt->result == NRF_SUCCESS) {
                m_subscriber.state = APP_MQTT_STATE_SUBSCRIBED;
            }
            break;
        }
        case MQTT_EVT_UNSUBACK:
        {
            APPL_LOG ("[APPL]: >> [SUB] MQTT_EVT_UNSUBACK\r\n");
            m_subscriber.state = APP_MQTT_STATE_CONNECTED;
            break;
        }
        default:
        {
            APPL_LOG ("[APPL]: >> [SUB] MQTT_EVT_SOME OTHER EVENT!? %d\r\n", p_evt->id);
            break;
        }
    }
}

/**@brief Function for starting connectable mode.
 */
static void connectable_mode_enter(void)
{
    uint32_t err_code = ipv6_medium_connectable_mode_enter(m_ipv6_medium.ipv6_medium_instance_id);
    APP_ERROR_CHECK(err_code);

    APPL_LOG("Physical layer in connectable mode.");
    m_display_state = LEDS_CONNECTABLE_MODE;
    m_subscriber.state = APP_MQTT_STATE_IDLE;
}


static void on_ipv6_medium_evt(ipv6_medium_evt_t * p_ipv6_medium_evt)
{
    switch (p_ipv6_medium_evt->ipv6_medium_evt_id)
    {
        case IPV6_MEDIUM_EVT_CONN_UP:
        {
            APPL_LOG("Physical layer: connected.");
            m_ipv6_state = APP_IPV6_IF_UP;
            m_display_state = LEDS_IPV6_IF_UP;
            break;
        }
        case IPV6_MEDIUM_EVT_CONN_DOWN:
        {
            APPL_LOG("Physical layer: disconnected.");
            m_ipv6_state = APP_IPV6_IF_DOWN;
            connectable_mode_enter();
            break;
        }
        default:
        {
            break;
        }
    }
}


static void on_ipv6_medium_error(ipv6_medium_error_t * p_ipv6_medium_error)
{
    APPL_LOG("ipv6 medium error.");
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    pwm_init();
    motor_init();
    scheduler_init();
    log_init();
    leds_init();
    timers_init();
    iot_timer_init();
    button_init();

    static ipv6_medium_init_params_t ipv6_medium_init_params;
    memset(&ipv6_medium_init_params, 0x00, sizeof(ipv6_medium_init_params));
    ipv6_medium_init_params.ipv6_medium_evt_handler    = on_ipv6_medium_evt;
    ipv6_medium_init_params.ipv6_medium_error_handler  = on_ipv6_medium_error;

    err_code = ipv6_medium_init(&ipv6_medium_init_params,
                                IPV6_MEDIUM_ID_BLE,
                                &m_ipv6_medium);
    APP_ERROR_CHECK(err_code);

    err_code = ipv6_medium_eui48_get(m_ipv6_medium.ipv6_medium_instance_id,
                                     &ipv6_medium_eui48);

    APP_ERROR_CHECK(err_code);

    ipv6_medium_eui48.identifier[EUI_48_SIZE - 1] = 0x00;

    err_code = ipv6_medium_eui48_set(m_ipv6_medium.ipv6_medium_instance_id,
                                     &ipv6_medium_eui48);

    APP_ERROR_CHECK(err_code);

    ip_stack_init();

    // Start execution.
    connectable_mode_enter();

    // Enter main loop.
    for (;;)
    {
        app_sched_execute();

        if (NRF_LOG_PROCESS() == false)
        {
            // Sleep waiting for an application event.
            err_code = sd_app_evt_wait();
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**
 * @}
 */
