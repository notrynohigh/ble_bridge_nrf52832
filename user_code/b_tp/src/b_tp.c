/**
 ****************************************************************************
 * MIT License
 * @file b_tp.c  
 * @version v2.0.1
 * Copyright (c) [2018-2019] [Bean  email: notrynohigh@outlook.com]
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************
 */
#include "b_tp.h"
#include "b_tp_port.h"
#include "stdlib.h"
#include "string.h"

#if (B_TP_CHECK_SELECT == B_TP_SUM)
#include "sum_8bit.h"
#elif (B_TP_CHECK_SELECT == B_TP_CRC16)
#include "crc16.h"
#else
#include "crc32.h"
#endif

#if 0
#define WEAK_FUNC    __weak
#else
#define WEAK_FUNC    __attribute__((weak))
#endif


static b_TPU8 sg_b_tp_buf[B_TP_STATIC_BUF_LEN];


typedef enum
{
    B_TP_UNLOCK,
    B_TP_LOCK
}b_tp_lock_t;

static b_tp_lock_t sg_lock_flag = B_TP_UNLOCK;

/**
 * @addtogroup B_TP
 * @{
 */

/**
 * @defgroup B_TP_CODE main code
 * @{
 */

static pb_tp_callback_t gps_rec_success_cb = b_TP_NULL;


static b_tp_rec_info_t  gs_tp_rec_info;

/**
 * @defgroup B_TP_PRIVATE_FUNC private functions
 * @{
 */


WEAK_FUNC b_tp_err_code_t _b_tp_rec_check_head(b_tp_head_t *phead)
{
    return B_TP_SUCCESS;
}

WEAK_FUNC void _b_tp_send_set_head(b_tp_head_t *phead)
{
    ;
}

static void _b_tp_send_lock()
{
    sg_lock_flag = B_TP_LOCK;
}

static void _b_tp_send_unlock()
{
    sg_lock_flag = B_TP_UNLOCK;
}

static b_tp_lock_t _b_tp_send_get_lock()
{
    return sg_lock_flag;
}
 
static b_tp_err_code_t _b_tp_check_data(b_tp_pack_info_t *pb_tp_pack_info)
{
    B_TP_CHECK_TYPE check_tmp; 
    B_TP_CHECK_TYPE check_calculate = 0;
    b_TPU8 *ptmp = (b_TPU8 *)pb_tp_pack_info;
    if(pb_tp_pack_info == b_TP_NULL)
    {
        return B_TP_MEM_ERR;
    }
#if B_TP_DEBUG_NO_CHECK		
    return B_TP_SUCCESS;
#endif		
    check_tmp = ((B_TP_CHECK_TYPE *)(&(pb_tp_pack_info->buf[pb_tp_pack_info->head.total_len])))[0];

#if (B_TP_CHECK_SELECT == B_TP_SUM)
    check_calculate = sum_8bit(ptmp, pb_tp_pack_info->head.total_len + B_TP_PACKET_HEAD_LEN);
#elif (B_TP_CHECK_SELECT == B_TP_CRC16)
    check_calculate = crc16(ptmp, pb_tp_pack_info->head.total_len + B_TP_PACKET_HEAD_LEN);
#else
    check_calculate = crc32(ptmp, pb_tp_pack_info->head.total_len + B_TP_PACKET_HEAD_LEN);
#endif
    if(check_tmp != check_calculate)
    {
      return B_TP_CHECK_ERR;
    }
    return B_TP_SUCCESS;
}


static b_tp_err_code_t _b_tp_create_check_code(b_tp_pack_info_t *pb_tp_pack_info)
{
    b_TPU8 *ptmp = (b_TPU8 *)pb_tp_pack_info;
    B_TP_CHECK_TYPE check_calculate = 0;
    if(pb_tp_pack_info == b_TP_NULL)
    {
        return B_TP_MEM_ERR;
    }   
#if (B_TP_CHECK_SELECT == B_TP_SUM)
    check_calculate = sum_8bit(ptmp, pb_tp_pack_info->head.total_len + B_TP_PACKET_HEAD_LEN);
#elif (B_TP_CHECK_SELECT == B_TP_CRC16)
    check_calculate = crc16(ptmp, pb_tp_pack_info->head.total_len + B_TP_PACKET_HEAD_LEN);
#else
    check_calculate = crc32(ptmp, pb_tp_pack_info->head.total_len + B_TP_PACKET_HEAD_LEN);
#endif	
    ((B_TP_CHECK_TYPE *)(&(pb_tp_pack_info->buf[pb_tp_pack_info->head.total_len])))[0] = check_calculate;
    return B_TP_SUCCESS;
}

static b_tp_err_code_t _b_tp_unpack_send(b_tp_pack_info_t *pb_tp_pack_info)
{
    b_tp_err_code_t err_code = B_TP_SUCCESS;
    b_TPU32 len, send_len = 0, len_tmp = 0;
    b_TPU8  *ptmp = (b_TPU8 *)pb_tp_pack_info;
    b_TPU8  frame_table[B_TP_MTU];
    B_TP_FRAME_NUMBER_TYPE frames = 0, i = 0;
    if(pb_tp_pack_info == b_TP_NULL)
    {
        return B_TP_PARAM_ERR;
    }
    len = pb_tp_pack_info->head.total_len + B_TP_PACKET_HEAD_LEN + B_TP_CHECK_LEN;
    if(len <= B_TP_MTU)
    {
    	  frames = 1;
    }
    else
    {
        frames = 1 + (len - B_TP_MTU) / (B_TP_MTU - sizeof(B_TP_FRAME_NUMBER_TYPE));
    }
    for(i = 0;i < frames; )
    {
        if(i == 0)
        {
            len_tmp = (len <= B_TP_MTU) ? len : B_TP_MTU;
            memcpy(frame_table, ptmp + send_len, len_tmp);  
        }
        else
        {
            len_tmp = B_TP_MTU;
            ((B_TP_FRAME_NUMBER_TYPE *)frame_table)[0] = i + 1;
            memcpy(&(frame_table[sizeof(B_TP_FRAME_NUMBER_TYPE)]), ptmp + send_len, B_TP_MTU - sizeof(B_TP_FRAME_NUMBER_TYPE));      
        }
        if(B_TP_LOCK == _b_tp_send_get_lock())
        {
            return B_TP_BUSY;
        }
        err_code = b_tp_port_send(frame_table, len_tmp);
        if(err_code == B_TP_SUCCESS)
        {
            if(i == 0)
            {
                send_len += len_tmp;
            }
            else
            {
                send_len += B_TP_MTU - sizeof(B_TP_FRAME_NUMBER_TYPE);
            }
            i++;
        }
        else if(err_code != B_TP_BUSY)
        {
            return err_code;
        }
    }
    if(send_len < len)
    {
        do
        {
            ((B_TP_FRAME_NUMBER_TYPE *)frame_table)[0] = i + 1;
            memcpy(&(frame_table[sizeof(B_TP_FRAME_NUMBER_TYPE)]), ptmp + send_len, len - send_len);
            if(B_TP_LOCK == _b_tp_send_get_lock())
            {
                return B_TP_BUSY;
            }        
            err_code = b_tp_port_send(frame_table, len - send_len + sizeof(B_TP_FRAME_NUMBER_TYPE));
        }while(err_code == B_TP_BUSY);
    }
    return err_code;
}


static void _b_tp_reset_rec_info()
{
    gs_tp_rec_info.data_index = -1;
    gs_tp_rec_info.expect_fnum = 0;
    gs_tp_rec_info.expect_number = 0;
    gs_tp_rec_info.frame_number = 0;
    gs_tp_rec_info.phead = b_TP_NULL;
    gs_tp_rec_info.remain_len = 0;
    gs_tp_rec_info.status = STA_WAIT_HEAD;
    _b_tp_send_unlock();
}

/**
 * @}
 */





/**
 * @defgroup B_TP_PUBLIC_FUNC public functions
 * @{
 */
b_tp_err_code_t b_tp_receive_data(b_TPU8 *pbuf, b_TPU32 len)
{
    b_TPU32 i = 0, off = 0;
    
    b_tp_pack_info_t *pb_tp_pack_info = NULL;
    b_tp_err_code_t err_code = B_TP_SUCCESS;
    
    if(pbuf == b_TP_NULL || len == 0 || b_TP_NULL == gps_rec_success_cb)
    {
        return B_TP_PARAM_ERR;
    }	

    for(i = 0;i < len;i++)
    {
        if(gs_tp_rec_info.data_index == -1)
        {
            if(((i - off) % (sizeof(B_TP_HEAD_TYPE))) == 0)
            {
                if(((B_TP_HEAD_TYPE *)pbuf)[i / sizeof(B_TP_HEAD_TYPE)] == B_TP_HEAD)
                {
                    gs_tp_rec_info.data_index = 0;
                    gs_tp_rec_info.expect_number = B_TP_PACKET_HEAD_LEN;
                    gs_tp_rec_info.status = STA_WAIT_HEAD;
                    _b_tp_send_lock();
                }
            } 
        }
        
        if(gs_tp_rec_info.data_index != -1)
        {
            if(gs_tp_rec_info.expect_number > 0)
            {
                sg_b_tp_buf[gs_tp_rec_info.data_index++] = pbuf[i];
                gs_tp_rec_info.expect_number--;
            }
            
            if(gs_tp_rec_info.expect_number == 0)
            {
                switch(gs_tp_rec_info.status)
                {
                    case STA_WAIT_HEAD: 
                        gs_tp_rec_info.phead = (b_tp_head_t *)sg_b_tp_buf;
                        if(B_TP_SUCCESS != _b_tp_rec_check_head(gs_tp_rec_info.phead))
                        {
                            _b_tp_reset_rec_info();
                            off = i + 1;
                        }
                        else
                        {
                            if(gs_tp_rec_info.phead->f_num == 0x0)
                            {
                                gs_tp_rec_info.expect_number = gs_tp_rec_info.phead->total_len + B_TP_CHECK_LEN;
                                gs_tp_rec_info.status = STA_WAIT_SINGLE_PACK;
                            }
                            else
                            {
                                gs_tp_rec_info.frame_number = (gs_tp_rec_info.phead->total_len + B_TP_CHECK_LEN - (B_TP_MTU - B_TP_PACKET_HEAD_LEN) + (B_TP_MTU - sizeof(B_TP_FRAME_NUMBER_TYPE) - 1)) / (B_TP_MTU - sizeof(B_TP_FRAME_NUMBER_TYPE));
                                gs_tp_rec_info.frame_number += 1;
                                gs_tp_rec_info.expect_fnum = 0x2;
                                gs_tp_rec_info.remain_len = gs_tp_rec_info.phead->total_len + B_TP_CHECK_LEN;
                                gs_tp_rec_info.expect_number = B_TP_MTU - B_TP_PACKET_HEAD_LEN;
                                gs_tp_rec_info.status = STA_WAIT_MULTI_PACK;
                                gs_tp_rec_info.remain_len -= gs_tp_rec_info.expect_number;
                            }
                        }
                        break;
                    case STA_WAIT_SINGLE_PACK:
                        pb_tp_pack_info = (b_tp_pack_info_t *)sg_b_tp_buf;
                        if(B_TP_SUCCESS == _b_tp_check_data(pb_tp_pack_info))
                        {
                            gps_rec_success_cb(pb_tp_pack_info->buf, pb_tp_pack_info->head.total_len);
                        }
                        _b_tp_reset_rec_info();
                        off = i + 1;
                        break;
                    case STA_WAIT_MULTI_PACK:
                        if(gs_tp_rec_info.remain_len == 0 )
                        {
                            if(gs_tp_rec_info.frame_number == (gs_tp_rec_info.expect_fnum - 1))
                            {
                                pb_tp_pack_info = (b_tp_pack_info_t *)sg_b_tp_buf;
                                if(B_TP_SUCCESS == _b_tp_check_data(pb_tp_pack_info))
                                {
                                    gps_rec_success_cb(pb_tp_pack_info->buf, pb_tp_pack_info->head.total_len);
                                } 
                            }
                            _b_tp_reset_rec_info();
                            off = i + 1;
                        }
                        else
                        {
                            gs_tp_rec_info.status = STA_CHECK_FNUM;
                            gs_tp_rec_info.expect_number = sizeof(B_TP_FRAME_NUMBER_TYPE);
                        }
                        break;
                    case STA_CHECK_FNUM:
                        gs_tp_rec_info.data_index = gs_tp_rec_info.data_index - sizeof(B_TP_FRAME_NUMBER_TYPE);
                        if(((B_TP_FRAME_NUMBER_TYPE *)(&(sg_b_tp_buf[gs_tp_rec_info.data_index])))[0] == gs_tp_rec_info.expect_fnum)
                        {
                            gs_tp_rec_info.expect_fnum++;
                            if(gs_tp_rec_info.remain_len >= (B_TP_MTU - sizeof(B_TP_FRAME_NUMBER_TYPE)))
                            {
                                gs_tp_rec_info.expect_number = B_TP_MTU - sizeof(B_TP_FRAME_NUMBER_TYPE);
                            }
                            else
                            {
                                gs_tp_rec_info.expect_number = gs_tp_rec_info.remain_len;
                            }
                            gs_tp_rec_info.remain_len -= gs_tp_rec_info.expect_number;
                            gs_tp_rec_info.status = STA_WAIT_MULTI_PACK;
                        }
                        else
                        {
                            _b_tp_reset_rec_info();
                            off = i + 1;
                        }
                        break;
                        
                }
            }
        }
    }
    
    return err_code;
}


b_tp_err_code_t b_tp_send_data(b_TPU8 *pbuf, b_TPU32 len)
{
    b_tp_err_code_t err_code = B_TP_SUCCESS;
    b_tp_pack_info_t *pb_tp_pack_info = b_TP_NULL;
    if(pbuf == b_TP_NULL || len == 0 || B_TP_LOCK == _b_tp_send_get_lock())
    {
        return B_TP_PARAM_ERR;
    }

    pb_tp_pack_info = (b_tp_pack_info_t *)sg_b_tp_buf;
    
    _b_tp_send_set_head(&(pb_tp_pack_info->head));
    pb_tp_pack_info->head.head = B_TP_HEAD;
    pb_tp_pack_info->head.total_len = len;	
    if((len + B_TP_CHECK_LEN + B_TP_PACKET_HEAD_LEN) > B_TP_MTU)
    {
        pb_tp_pack_info->head.f_num = 0X1;
    }
    else
    {
        pb_tp_pack_info->head.f_num = 0X0;
    }	
    memcpy(pb_tp_pack_info->buf, pbuf, len);
    if(B_TP_SUCCESS != _b_tp_create_check_code(pb_tp_pack_info))
    {
        return B_TP_CHECK_ERR;        
    }

    err_code = _b_tp_unpack_send(pb_tp_pack_info);

    return err_code;
}



void b_tp_reg_callback(pb_tp_callback_t pfunc)
{
    if(pfunc == b_TP_NULL)
    {
        return;    
    }    
    gps_rec_success_cb = pfunc;
    _b_tp_reset_rec_info();
}


/**
 * @}
 */


/**
 * @}
 */

/**
 * @}
 */

