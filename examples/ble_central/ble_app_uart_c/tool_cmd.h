#ifndef __TOOL_CMD_H__
#define __TOOL_CMD_H__


#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif


#define TCMD_SCAN              0XA0
#define TCMD_UNKNOW            0XAF




#pragma pack(1)
    

typedef struct
{
    uint8_t cmd;
    uint8_t status;
    uint8_t buf[1];
}tcmd_struct_t;




#pragma pack()


void tc_send(uint8_t cmd, uint8_t status, uint8_t *pbuf, uint32_t len);

void tc_parse(uint8_t *pbuf, uint32_t len);

#ifdef __cplusplus
}
#endif



#endif

