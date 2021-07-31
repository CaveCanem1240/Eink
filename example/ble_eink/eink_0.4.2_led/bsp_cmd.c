//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2020, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : bsp_cmd.c
//  Date :     2019-07-17 9:51
//  Version :  V0001
//  History :  初始创建版本
//  描述：
//******************************************************************************
#include <string.h>
#include "bsp_cmd.h"
#include "nrf_nvmc.h"
#include "bsp_flash.h"

//增加命令条目
const cmd_item_t cmd_item[] = 
{
  {STRINGIFY(read),Flash_read_cmd},
  {STRINGIFY(write),Flash_write_cmd},
  {STRINGIFY(erase),Flash_erase_cmd},
  {NULL,NULL}
};

/* 执行命令*/
static CMD_CODE_T  CMD_ExcuteHandle(size_t size , char**argv);


//******************************************************************************
// fn : CLI_Process
//
// brief : 处理串口数据，寻找命令
//
// param : buf -> 数据指针
//
// return : none
CMD_CODE_T CLI_Process(uint8_t* buf)
{
  char *pTmp = NULL;
  char* argv[10];
  size_t count = 0;
  
  if(buf == NULL)
  {
    //数据内容为空，直接返回
    return CMD_BAD_BUF;
  }
  
  pTmp = (char*)strchr((char const*)buf,'\r');
  if(pTmp)
  {
    *pTmp = '\0';
  }
  pTmp = (char*)buf;
  
  //提取指令返回的参数，此时buf已经遭到破坏
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
// brief : 执行命令
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
