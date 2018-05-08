#ifndef __TOOL_CMD_H__
#define __TOOL_CMD_H__


#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STRUCT_OFF(type, n)     ((uint32_t)(&(((type *)0)->n)))    
    
/**
 * @defgroup PROTOCOL_CMD_LIST 
 * @{
 */

#define CMD_HEART               0X00
#define CMD_VERSION             0X01
#define CMD_BATTERY             0X02
#define CMD_SET_USER_ID         0X03
#define CMD_SET_TIME            0X04
#define CMD_GET_TIME            0X05
#define CMD_GET_TOTAL_STEP      0X06
#define CMD_SYN_WALK_DATA       0X07
#define CMD_SYN_RUN_DATA        0X08
#define CMD_RT_RUN_START        0X09
#define CMD_RT_RUN_STOP         0X0A
#define CMD_CHIP_ADJUST         0X0B
#define CMD_DRAW_WAVE_START     0X0C
#define CMD_DRAW_WAVE_END       0X0D

#define CMD_LED_SHOW            0X0E
#define CMD_SET_LED_ONOFF       0X0F
#define CMD_GET_LED_ONOFF       0X10
#define CMD_SET_LED_COLOR       0X11
#define CMD_GET_LED_COLOR       0X12

#define CMD_NOTIFY_MESSAGE      0X13


#define CMD_REBOOT              0X50
#define CMD_ERASE_CHIP          0X51
#define CMD_GET_ERR_INFO        0X52
#define CMD_ENABLE_SAMPLE       0X53
#define CMD_UPLOAD_SAMPLE       0X54

#define CMD_TOOL_SCAN           0XA0
#define CMD_TOOL_CONNECT        0XA1
/**
 * @}
 */

/**
 * @defgroup CMD_STATUS_LIST
 * @{
 */
#define CMD_STATUS_SUCCESS          0X0
#define CMD_STATUS_UNKNOWN          0X1
#define CMD_STATUS_ACK              0X2
#define CMD_STATUS_REPEAT           0X3
#define CMD_STATUS_LAST_ONE         0X4
#define CMD_STATUS_NO_DATA          0X5
#define CMD_STATUS_HW_ERROR         0X6
#define CMD_STATUS_PARAM_INVALID    0X7

/**
 * @}
 */

/**
 * @defgroup SYSTEM_ERROR_CODE
 * @{
 */
#define MEMS_NO_INTERRUP_CODE   200
#define USER_ID_CHANGED_CODE    201
#define SYSTEM_REBOOT_CODE      255
/**
 * @}
 */

/**
 * @defgroup MESSAGE_TYPE
 * @{
 */
#define MSG_STOMP     0X1
/**
 * @}
 */





#pragma pack(1)
    

typedef struct
{
    uint8_t cmd;
    uint8_t status;
    uint8_t buf[1];
}tcmd_struct_t;

typedef struct
{
    uint8_t cmd;
    uint8_t status;
    uint8_t *pbuf;
}tcmd_pstruct_t;

/** CMD_VERSION */
typedef struct
{
    uint16_t internal_code;
    uint32_t hw_version;
    uint32_t fw_version;
    uint16_t algo_version;
    uint16_t protocol_version;
}pro_version_t;

/** CMD_BATTERY */
typedef struct
{
    uint16_t voltage_mv; 
}pro_battery_t;


/** CMD_SET_USER_ID */
typedef struct
{
    uint32_t user_id; 
}pro_user_id_t;


/** CMD_SET/GET_TIME */
typedef struct
{
    uint8_t  year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
}pro_time_t;


/** CMD_GET_TOTAL_STEP */
typedef struct
{
    uint8_t  month;
    uint8_t  day;
}pro_total_step_require_t;


typedef struct
{
    uint8_t  month;
    uint8_t  day;
    uint32_t total_step;
    uint32_t walk;
    uint32_t race;
    uint32_t run;
}pro_total_step_response_t;


typedef struct
{
    uint8_t  month;
    uint8_t  day;
    uint8_t  s_hour;
    uint8_t  s_minute;
    uint8_t  e_hour;
    uint8_t  e_minute;
}pro_syn_require_t;

/** CMD_SYN_WALK_DATA */
typedef struct
{
    uint8_t  hour;
    uint8_t  minute;
    uint16_t total_step;
    uint16_t slow_walk_step;
    uint16_t fast_walk_step;
    uint16_t run_step;
    uint16_t inside_step;
    uint16_t outside_setp;
    uint16_t normal_step;
}pro_walk_info_t;


#define PROTO_SYN_WALK_PER_NUMBER     6
typedef struct
{
    uint8_t  month;
    uint8_t  day;
    pro_walk_info_t walk_info[PROTO_SYN_WALK_PER_NUMBER];
}pro_syn_walk_response_t;



/** CMD_SYN_RUN_DATA */
typedef struct
{
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  run_step;
    uint8_t  forefoot_setp;
    uint8_t  midfoot_setp;
    uint8_t  backfoot_setp;
    uint8_t  inside_setp;
    uint8_t  normal_setp;
    uint8_t  outside_setp;
    uint16_t height_cm;
    uint16_t force_g;
    uint16_t land_time_ms;
    uint16_t float_time_ms;
}pro_run_info_t;

#define PROTO_SYN_RUN_PER_NUMBER     6
typedef struct
{
    uint8_t  month;
    uint8_t  day;
    pro_run_info_t run_info[PROTO_SYN_RUN_PER_NUMBER];
}pro_syn_run_response_t;


/** CMD_RT_RUN_START */
typedef struct
{
    uint8_t param_reset;
}pro_rt_start_require_t;


typedef struct
{
    uint32_t total_step;
    uint32_t slow_walk_step;
    uint32_t fast_walk_step;
    uint32_t run_step;
    uint32_t walk_inside_step;
    uint32_t walk_normal_step;
    uint32_t walk_outside_step;
    uint32_t run_forefoot_step;
    uint32_t run_midfoot_step;
    uint32_t run_backfoot_step;
    uint32_t run_inside_step;
    uint32_t run_normal_step;
    uint32_t run_outside_step;
    uint16_t height_cm;
    uint16_t force_g;
    uint16_t land_time_ms;
    uint16_t float_time_ms;
}pro_rt_info_t;

typedef struct
{
    uint8_t flag;
    pro_rt_info_t rt_info;
}pro_rt_detail_response_t;

typedef struct
{
    uint8_t  flag;
    uint32_t total_step;
}pro_rt_simple_response_t;


/** CMD_CHIP_ADJUST */
typedef struct
{
    uint8_t status;
}pro_adjust_t;

/** CMD_DRAW_WAVE_START */
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
}pro_xyz_info_t;

typedef struct
{
    pro_xyz_info_t xyz_info[2];
}pro_xyz_t;


/** CMD_SET_RUN_LED_ONOFF */
typedef struct
{
    uint8_t onoff_swing;
    uint8_t onoff_race;
    uint8_t onoff_run;
    uint8_t onoff_force;  
}pro_led_onoff_t;


/** CMD_SET_LED_COLOR */
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
}pro_rgb_color_t;

typedef struct
{
    pro_rgb_color_t color_swing;
    pro_rgb_color_t color_race;
    pro_rgb_color_t color_run;
    pro_rgb_color_t color_force;
}pro_led_color_t;


/** CMD_GET_ERR_CODE */

typedef struct
{
    uint32_t utc;
}pro_req_err_code_t;

typedef struct
{
    uint32_t utc;
    uint8_t  err_code;
    uint32_t reserved;
}pro_err_code_t;

#define PROTO_ERR_CODE_PER_NUMBER     6
typedef struct
{
    pro_err_code_t err_code[PROTO_ERR_CODE_PER_NUMBER];    
}pro_upload_err_code_t;

#define PROTO_SAMPLE_DATA_PER_NUMBER  12
typedef struct
{
    uint8_t sample_data[PROTO_SAMPLE_DATA_PER_NUMBER];
}pro_upload_sample_data_t;

#define PROTO_MSG_PARAM_MAX_LEN       11
typedef struct
{
    uint8_t msg;
    uint8_t param[PROTO_MSG_PARAM_MAX_LEN];
}pro_notify_message_t;



/** CMD_TOOL_SCAN */

#define DEVICE_NAME_MAX_LEN           16

typedef struct
{
    uint8_t addr[6];
    int8_t  rssi;
    uint8_t name[DEVICE_NAME_MAX_LEN];
}pro_scan_response_t;

typedef struct
{
    uint8_t type;           //  0: stop  1: start
}pro_scan_require_t;

/** CMD_TOOL_CONNECT */

typedef struct
{
    uint8_t addr[6];
}pro_connect_info_t;


#pragma pack()


void tc_send(uint8_t cmd, uint8_t status, uint8_t *pbuf, uint32_t len);

void tc_parse(tcmd_pstruct_t result);

#ifdef __cplusplus
}
#endif



#endif

