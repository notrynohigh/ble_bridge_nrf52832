/**
 ************************************************************
 * @file  app_service.h
 * @brief app service interface
 ************************************************************
 */
#ifndef __APP_SERVICE_H__
#define __APP_SERVICE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "ble_db_discovery.h"
/**
 * @addtogroup USER_DRIVER
 * @{
 */ 

/**
 * @addtogroup APP_SERVICE
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup APPS_CONFIG 
 * @{
 */	
#define     BLE_APPS_BLE_OBSERVER_PRIO         2	

#define     BLE_APPS_BASE_UUID                 {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */	

#define     BLE_APPS_SERVICE_UUID               0xAA00
#define     BLE_APPS_WRITE_CHAR_UUID     	    0xAA01
#define     BLE_APPS_NOTIFY_CHAR_UUID     	    0xAA02
#define     BLE_APPS_INDICATE_CHAR_UUID     	0xAA03

#define     BLE_APPS_SERVICE_MTU                20
/**
 * @}
 */
	
/**@brief   Macro for defining a ble_apps instance.
 *
 * @param   _name   Name of the instance.
 */
#define BLE_APPS_DEF(_name)                                                                          \
static ble_apps_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_APPS_BLE_OBSERVER_PRIO,                                                     \
                     ble_apps_on_ble_evt, &_name)



/**@brief Forward declaration of the ble_apps_t type. */
typedef struct ble_apps_s ble_apps_t;


/**@brief Service structure. This contains various status information for the service. */
struct ble_apps_s
{
	  uint8_t  uuid_type;
	  uint16_t app_service_handle;
      uint16_t conn_handle;
	  uint16_t app_notify_handle;
	  uint16_t app_write_handle;
	  uint16_t app_notify_ccc_handle;
};

/**
 * @addtogroup PUBLIC_FUNC
 * @{
 */
uint32_t tc_ble_init(ble_apps_t * p_ble_tc_c);

void ble_apps_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
void ble_tc_c_on_db_disc_evt(ble_apps_t * p_ble_nus_c, ble_db_discovery_evt_t * p_evt);
uint32_t ble_tc_c_send_buf(ble_apps_t * p_ble_tc_c, uint8_t * pbuf, uint16_t length);
/**
 * @}
 */


#ifdef __cplusplus
}
#endif 
 

/**
 * @}
 */
 
/**
 * @}
 */

#endif



