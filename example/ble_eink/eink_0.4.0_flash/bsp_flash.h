//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2020, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : bsp_flash.h
//  Date :     2019-07-17 9:51
//  Version :  V0001
//  History :  初始创建版本
//  描述：
//******************************************************************************
#ifndef _BSP_FLASH_H__
#define _BSP_FLASH_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif
  
//操作flash的page信息
typedef struct
{
  uint32_t page_num;
  uint32_t page_size;
  uint32_t opt_addr;
}flash_opt_t;

extern flash_opt_t  flash_info;

/* Flash page init*/
void Flash_info_init(void);

/* Flash page erase*/
void Flash_erase_cmd(size_t argc, char **argv);

/*Flash page write*/
void Flash_write_cmd(size_t argc, char **argv);

/*Flash page read*/
void Flash_read_cmd(size_t argc, char **argv);

#ifdef __cplusplus
}
#endif



#endif   //_BSP_FLASH_H__

/**
 * @}
 */
