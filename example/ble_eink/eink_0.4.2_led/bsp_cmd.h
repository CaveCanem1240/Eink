//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2020, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : bsp_cmd.h
//  Date :     2019-07-17 9:51
//  Version :  V0001
//  History :  ��ʼ�����汾
//  ������
//******************************************************************************

#ifndef _BSP_CMD_H__
#define _BSP_CMD_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif
  
//******************************************************************************
//define CMD Item struct
typedef void (*cmd_handler)(size_t argc, char **argv);

typedef struct
{
  const char*  cmd;       //cmd
  cmd_handler  handler;   //handler
}cmd_item_t;

//******************************************************************************
// ����������ָ��
//
typedef enum
{
  CMD_SUCCESS,
  CMD_NOT_FIND,
  CMD_BAD_BUF
}CMD_CODE_T;

//******************************************************************************
// fn : CLI_Process
//
// brief : ���������ݣ�Ѱ������
//
// param : buf -> ����ָ��
//
// return : cmd_code_t
CMD_CODE_T CLI_Process(uint8_t* buf);

#ifdef __cplusplus
}
#endif

#endif  //_BSP_CMD_H__

/**
 * @}
 */
