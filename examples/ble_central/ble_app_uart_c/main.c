/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "ble_advdata.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "b_tp.h"
#include "tool_cmd.h"
#include "tc_ble.h"

#include "b_tp_port.h"

#define APP_BLE_CONN_CFG_TAG    1                                       /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< Application's BLE observer priority. You shoulnd't need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define SCAN_INTERVAL           0x0050                                  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                                  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0x0000                                  /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)         /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)         /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                                       /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Determines supervision time-out in units of 10 millisecond. */

#define UUID16_SIZE             2                                       /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                                       /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                                      /**< Size of 128 bit UUID */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service back to the sender. */

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< DB discovery module instance. */
BLE_APPS_DEF(m_tc_ble);

static uint8_t connect_flag = 0;
uint16_t connect_handle = 0;
/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
    (uint16_t)SLAVE_LATENCY,            // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
};

/** @brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist = 0,
    #endif
};


void uart_send_buff(uint8_t *pbuf, uint16_t len);
void uart_send_string(uint8_t *);
void tc_get_conn_status(void);
/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}

static void scan_stop(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_stop();
    APP_ERROR_CHECK(ret);

}



/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */

static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_tc_c_on_db_disc_evt(&m_tc_ble, p_evt);
}



/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:

            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}



/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    data_t                adv_data;
    data_t                dev_name;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_gap_evt_adv_report_t const * p_adv_report = &p_gap_evt->params.adv_report;
            pro_scan_response_t scan_resp;
            memset(&scan_resp, 0, sizeof(scan_resp));
            // For readibility.
            ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;

            // Initialize advertisement report for parsing
            adv_data.p_data   = (uint8_t *)p_gap_evt->params.adv_report.data;
            adv_data.data_len = p_gap_evt->params.adv_report.dlen;

            // Search for advertising names.
            err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);     
            
            if(err_code == NRF_ERROR_NOT_FOUND)
            {
                break;
            }
            scan_resp.rssi = p_adv_report->rssi;
            
//            uart_send_buff((uint8_t *)&(p_adv_report->peer_addr), sizeof(p_adv_report->peer_addr));
            memcpy(scan_resp.addr, p_adv_report->peer_addr.addr, sizeof(scan_resp.addr));
            memcpy(scan_resp.name, dev_name.p_data, (dev_name.data_len > DEVICE_NAME_MAX_LEN) ? DEVICE_NAME_MAX_LEN : dev_name.data_len);
            tc_send(CMD_TOOL_SCAN, 0, (uint8_t *)&scan_resp, sizeof(pro_scan_response_t));
            
        }break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected to target");
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
        
            connect_flag = 0x1;
            tc_get_conn_status();
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            connect_flag = 0x0;
            tc_get_conn_status();
            break;
        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                connect_flag = 0x0;
                tc_get_conn_status();
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
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


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{

}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the Power manager. */
static void power_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the Database Discovery Module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief uart send/rec functions
 */

void uart_send_string(uint8_t *pstr)
{
    if(pstr != NULL)
    {
        while(*pstr != '\0')
        {
            app_uart_put(*pstr);
            pstr++;
        }
    }
}

void uart_send_buff(uint8_t *pbuf, uint16_t len)
{
    uint16_t i = 0;
    if(pbuf == NULL)
    {
        return;
    }
    for(i = 0;i < len;i++)
    {
        app_uart_put(pbuf[i]);
    }
}

uint16_t uart_rec_buff(uint8_t *pbuf, uint16_t len)
{
    uint16_t i = 0;
    if(pbuf == NULL)
    {
        return 0;
    }
    for(i = 0;i < len;i++)
    {
        if(NRF_ERROR_NOT_FOUND == app_uart_get(pbuf))
        {
            break;
        }
        pbuf++;
    }
    return i;
}



/**
 * @brief loop timer
 */
APP_TIMER_DEF(utimer_20ms_id);
#define UART_RX_LEN         256
static uint8_t urx_table[UART_RX_LEN];
#define UTIMER_20MS_INTERVAL    APP_TIMER_TICKS(20)
static uint32_t connect_timeout = 0;
static uint8_t timeout_start = 0;
static void _timer_20ms_handler(void *parg)
{
    uint16_t len = 0;
    len = uart_rec_buff(urx_table, UART_RX_LEN);
    if(len > 0)
    {
        b_tp_receive_data(urx_table, len);
    }
    if(timeout_start == 0x1)
    {
        connect_timeout++;
        if(connect_timeout > 500)
        {
            sd_ble_gap_connect_cancel();
            timeout_start = 0;
        }
    }
    else 
    {
        connect_timeout = 0;
    }
    uint32_t err_code = app_timer_start(utimer_20ms_id, UTIMER_20MS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);    
}

static void user_timer_init()
{
	uint32_t err_code;
    err_code = app_timer_create(&utimer_20ms_id, APP_TIMER_MODE_SINGLE_SHOT, _timer_20ms_handler);
    APP_ERROR_CHECK(err_code);
}


static void user_timer_start()
{
	uint32_t err_code;
    err_code = app_timer_start(utimer_20ms_id, UTIMER_20MS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/** timer end */

/**
 * @proto
 */

uint8_t tmp_table[256];
uint8_t tmp_flag = 0;
uint8_t tmp_buf_len = 0;
tcmd_pstruct_t rec_result;


void b_tp_callback(b_TPU8 *pbuf, b_TPU32 len)
{
    tcmd_struct_t *ptmp = (tcmd_struct_t *)pbuf;
    uint8_t off = STRUCT_OFF(tcmd_struct_t, buf);
    if(len < off)
    {
        return;
    }
    rec_result.cmd = ptmp->cmd;
    rec_result.status = ptmp->status;
    memcpy(tmp_table, ptmp->buf, len - off);
    tmp_buf_len = len - off;
    if(len == off)
    {
        rec_result.pbuf = NULL;
    }
    else
    {
        rec_result.pbuf = tmp_table;
    }
    tmp_flag = 1;
}

/*************************************************************************************/

void tc_scan(uint8_t *pbuf)
{
    static uint8_t scan_flag = 0;
    pro_scan_require_t *ptmp = (pro_scan_require_t *)pbuf;
    if(pbuf == NULL)
    {
        return;
    }
    if(ptmp->type == 0)
    {
        if(scan_flag == 1)
        {
            scan_stop();
            scan_flag = 0;
        }
    }
    else
    {
        if(scan_flag == 0)
        {
            scan_start();
            scan_flag = 1;
        }
    }
}

void tc_connect(uint8_t *pbuf)
{
    uint32_t err_code;
    ble_gap_addr_t peer_addr;
    pro_connect_info_t *ptmp = (pro_connect_info_t *)pbuf;
    if(pbuf == NULL || connect_flag != 0x0)
    {
        if(connect_flag)
        {
            tc_send(CMD_TOOL_CONNECT, 0, NULL, 0);
        }
        return;
    }
    memcpy(peer_addr.addr, ptmp->addr, 6);
    peer_addr.addr_id_peer = 0;
    peer_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    err_code = sd_ble_gap_connect(&peer_addr,
                                  &m_scan_params,
                                  &m_connection_param,
                                  APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
    tc_send(CMD_TOOL_CONNECT, 0, NULL, 0);
}


void ble_send_port(uint8_t *pbuf, uint32_t len)
{
    ble_tc_c_send_buf(&m_tc_ble, pbuf, len);
}


void tc_bridge(tcmd_pstruct_t result)
{
    reg_port_callback(ble_send_port);
    tc_send(result.cmd, result.status, result.pbuf, tmp_buf_len);
    reg_port_cb_reset();
}

void tc_get_conn_status()
{
    pro_conn_sta_t tmp;
    tmp.status = connect_flag;
    tc_send(CMD_TOOL_CONN_STA, 0, (uint8_t *)&tmp, sizeof(pro_conn_sta_t));
}

void tc_parse(tcmd_pstruct_t result)
{
    switch(result.cmd)
    {
        case CMD_TOOL_SCAN:
            tc_scan(result.pbuf); 
            break;
        case CMD_TOOL_CONNECT:
            tc_connect(result.pbuf);
            timeout_start = 0x1;
            break;
        case CMD_TOOL_CONN_STA:
            tc_get_conn_status();
            break;
        default:
            tc_bridge(result);
            break;
    }
}

void tc_could_read_write()
{
    connect_flag = 0x2;
    tc_get_conn_status();
}

/*************************************************************************************/
int main(void)
{
    log_init();
    timer_init();
    power_init();
    uart_init();
    user_timer_init();
    b_tp_reg_callback(b_tp_callback);
    buttons_leds_init();
    db_discovery_init();
    ble_stack_init();
    gatt_init();
    tc_ble_init(&m_tc_ble);
    
    // Start scanning for peripherals and initiate connection
    // with devices that advertise NUS UUID.
    //    printf("BLE UART central example started.\r\n");
    //    NRF_LOG_INFO("BLE UART central example started.");
    //    scan_start();
    user_timer_start();
    //uart_send_string((uint8_t *)"hello world");
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
        if(tmp_flag != 0)
        {
            tc_parse(rec_result);
            tmp_flag = 0;
        }
    }
}
