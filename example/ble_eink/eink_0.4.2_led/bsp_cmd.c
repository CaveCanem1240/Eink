//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2020, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : bsp_cmd.c
//  Date :     2019-07-17 9:51
//  Version :  V0001
//  History :  ��ʼ�����汾
//  ������
//******************************************************************************
#include <string.h>
#include "bsp_cmd.h"
#include "nrf_nvmc.h"
#include "bsp_flash.h"

//����������Ŀ
const cmd_item_t cmd_item[] = 
{
  {STRINGIFY(read),Flash_read_cmd},
  {STRINGIFY(write),Flash_write_cmd},
  {STRINGIFY(erase),Flash_erase_cmd},
  {NULL,NULL}
};

/* ִ������*/
static CMD_CODE_T  CMD_ExcuteHandle(size_t size , char**argv);


//******************************************************************************
// fn : CLI_Process
//
// brief : ���������ݣ�Ѱ������
//
// param : buf -> ����ָ��
//
// return : none
CMD_CODE_T CLI_Process(uint8_t* buf)
{
  char *pTmp = NULL;
  char* argv[10];
  size_t count = 0;
  
  if(buf == NULL)
  {
    //��������Ϊ�գ�ֱ�ӷ���
    return CMD_BAD_BUF;
  }
  
  pTmp = (char*)strchr((char const*)buf,'\r');
  if(pTmp)
  {
    *pTmp = '\0';
  }
  pTmp = (char*)buf;
  
  //��ȡָ��صĲ�������ʱbuf�Ѿ��⵽�ƻ�
  while((argv[count] = strtok(pTmp," ")) != NULL)
  {
    count++;
    pTmp = NULL;
    if(count >=10)
    {
      break;
    }
  }
  return CMD_ExcuteHandle(count,argv);
  
}
//******************************************************************************
// fn : CMD_ExcuteHandle
//
// brief : ִ������
//
// param : none
//
// return : cmd_code_t
static CMD_CODE_T  CMD_ExcuteHandle(size_t size , char**argv)
{
  uint8_t i = 0;
  if(size == 0)
  {
    return CMD_NOT_FIND;
  }
  
  while((cmd_item + i)->cmd)
  {
    if(!strcmp((cmd_item + i)->cmd,argv[0]))
    {
      (cmd_item + i)->handler(size,argv);
      return CMD_SUCCESS;
    }
    i++;
  }
  return CMD_NOT_FIND;
}

/**
 * @}
 */
