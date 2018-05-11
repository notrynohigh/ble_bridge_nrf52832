#include "tc_ble.h"
#include "ble_types.h"

static ble_uuid_t const m_tc_uuid =
{
    .uuid = BLE_APPS_SERVICE_UUID,
    .type = BLE_UUID_TYPE_BLE
};

extern void uart_send_buff(uint8_t *pbuf, uint16_t len);
extern void uart_send_string(uint8_t *);


uint32_t tc_ble_init(ble_apps_t * p_ble_tc_c)
{
    p_ble_tc_c->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_ble_tc_c->app_write_handle = BLE_CONN_HANDLE_INVALID;
    p_ble_tc_c->app_notify_ccc_handle = BLE_CONN_HANDLE_INVALID;
    return ble_db_discovery_evt_register(&m_tc_uuid);
}


static void on_hvx(ble_apps_t * p_ble_tc_c, ble_evt_t const * p_ble_evt)
{
    // HVX can only occur from client sending.
    if (   (p_ble_tc_c->app_notify_handle != BLE_GATT_HANDLE_INVALID)
        && (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_tc_c->app_notify_handle))
    {
        uart_send_buff((uint8_t *)p_ble_evt->evt.gattc_evt.params.hvx.data, p_ble_evt->evt.gattc_evt.params.hvx.len);
    }
}



void ble_apps_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
   ble_apps_t * p_ble_tc_c = (ble_apps_t *)p_context;

    if ((p_ble_tc_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    if ( (p_ble_tc_c->conn_handle != BLE_CONN_HANDLE_INVALID)
       &&(p_ble_tc_c->conn_handle != p_ble_evt->evt.gap_evt.conn_handle)
       )
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_tc_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            
            break;
        case BLE_GAP_EVT_CONNECTED:
            p_ble_tc_c->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for creating a message for writing to the CCCD. */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t cccd_handle, bool enable)
{
    uint8_t buf[BLE_CCCD_VALUE_LEN];

    buf[0] = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    buf[1] = 0;

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = cccd_handle,
        .offset   = 0,
        .len      = sizeof(buf),
        .p_value  = buf
    };

    return sd_ble_gattc_write(conn_handle, &write_params);
}


uint32_t ble_tc_c_tx_notif_enable(ble_apps_t * p_ble_tc_c)
{
    uint32_t err_code;
    if(p_ble_tc_c == NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if ( (p_ble_tc_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_tc_c->app_notify_ccc_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if( NRF_SUCCESS != (err_code = cccd_configure(p_ble_tc_c->conn_handle, p_ble_tc_c->app_notify_ccc_handle, true)))
    {
        uart_send_buff((uint8_t *)&err_code, sizeof(err_code));
    }
    return err_code;
}



void ble_tc_c_on_db_disc_evt(ble_apps_t * p_ble_tc_c, ble_db_discovery_evt_t * p_evt)
{
    ble_apps_t tc_c_evt;
    memset(&tc_c_evt,0,sizeof(tc_c_evt));

    ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

    // Check if the NUS was discovered.
    if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
        &&  (p_evt->params.discovered_db.srv_uuid.uuid == BLE_APPS_SERVICE_UUID)
        &&  (p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE))
    {
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            switch (p_chars[i].characteristic.uuid.uuid)
            {
                case BLE_APPS_WRITE_CHAR_UUID:
                    tc_c_evt.app_write_handle = p_chars[i].characteristic.handle_value;
                    break;

                case BLE_APPS_NOTIFY_CHAR_UUID:
                    tc_c_evt.app_notify_handle = p_chars[i].characteristic.handle_value;
                    tc_c_evt.app_notify_ccc_handle = p_chars[i].cccd_handle;
                    break;
                default:
                    break;
            }
        }
        
        p_ble_tc_c->app_write_handle = tc_c_evt.app_write_handle;
        p_ble_tc_c->app_notify_handle = tc_c_evt.app_notify_handle;
        p_ble_tc_c->app_notify_ccc_handle = tc_c_evt.app_notify_ccc_handle;

        ble_tc_c_tx_notif_enable(p_ble_tc_c);
        uint8_t tip_table[3] = {0, 1, 2};
        uart_send_buff(tip_table, 3);
    }
}


uint32_t ble_tc_c_send_buf(ble_apps_t * p_ble_tc_c, uint8_t * pbuf, uint16_t length)
{
    if(p_ble_tc_c == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (length > BLE_APPS_SERVICE_MTU)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_ble_tc_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = p_ble_tc_c->app_write_handle,
        .offset   = 0,
        .len      = length,
        .p_value  = pbuf
    };

    return sd_ble_gattc_write(p_ble_tc_c->conn_handle, &write_params);
}

