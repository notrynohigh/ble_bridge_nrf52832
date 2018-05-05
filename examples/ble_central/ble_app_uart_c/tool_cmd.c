#include "tool_cmd.h"
#include "b_tp.h"
#include "string.h"






void tc_send(uint8_t cmd, uint8_t status, uint8_t *pbuf, uint32_t len)
{
    uint8_t tmp_buf[256];
    tcmd_struct_t *pstruct = (tcmd_struct_t *)tmp_buf;
    if(pbuf == NULL && len > 0)
    {
        return;
    }
    pstruct->cmd = cmd;
    pstruct->status = status;
    memcpy(pstruct->buf, pbuf, len);
    b_tp_send_data(tmp_buf, len + 2);
}


void tc_parse(uint8_t *pbuf, uint32_t len)
{
    tcmd_struct_t *pstruct = (tcmd_struct_t *)pbuf;
    switch(pstruct->cmd)
    {
        case TCMD_SCAN:
        tc_send(TCMD_SCAN, 0, NULL, 0);    
        break;
        default:
        tc_send(TCMD_UNKNOW, 0, NULL, 0);
        break;
    }
}





