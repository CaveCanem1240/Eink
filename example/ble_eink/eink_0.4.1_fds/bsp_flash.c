//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2020, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : bsp_flash.c
//  Date :     2019-07-17 9:51
//  Version :  V0001
//  History :  ��ʼ�����汾
//  ������
//******************************************************************************
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bsp_flash.h"
#include "nrf_nvmc.h"

#define  FLASH_MAX_STRING_LEN                   (64u)

typedef struct
{
   uint32_t buffer[FLASH_MAX_STRING_LEN]; 
}flash_data_t;

//******************************************************************************
//Declare area
//
/* chip infomation */
flash_opt_t   flash_info;

//******************************************************************************
// fn : Flash_info_init
//
// brife : Flash page init
//
// param : none
//
// return : none
void Flash_info_init(void)
{
  flash_info.page_num = NRF_FICR->CODESIZE - 1;  //�������һҳflash
  flash_info.page_size = NRF_FICR->CODEPAGESIZE;
  flash_info.opt_addr = flash_info.page_num * flash_info.page_size;  
}
//******************************************************************************
// fn : Flash_erase_cmd
//
// brife : Flash page erase
//
// param : none
//
// return : none
void Flash_erase_cmd(size_t argc, char **argv)
{
  //����ָ��ҳ
  if(flash_info.page_num == 0)
  {
    Flash_info_init();
  }
  nrf_nvmc_page_erase(flash_info.opt_addr);
}
//******************************************************************************
// fn : Flash_write_cmd
//
// brife : Flash page write
//
// param : argc -> ��������
//         argv -> ����ָ��
//
// return : none
// format = [write] [offset] [data]
void Flash_write_cmd(size_t argc, char **argv)
{
  uint32_t offset = 0;
  uint32_t len = 0;
  uint32_t address = 0;
  char*    pBuf = NULL;
  
  if (argc != 3)
  {
    printf("%s: bad parameter count\r\n", argv[0]);
    return;
  }
  
  //��ȡ���ݳ��ȣ���ƫ����
  len = strlen(argv[2]);
  offset = strtoul(argv[1],NULL,0);
  
  if (len > FLASH_MAX_STRING_LEN)
  {
      len = FLASH_MAX_STRING_LEN;
      return;
  }
  //���ַ���д��ָ����ַFlash�С�
  address = flash_info.opt_addr + offset;
  pBuf = argv[2];
  
  nrf_nvmc_write_bytes(address,(const uint8_t*)pBuf,len);
}
//******************************************************************************
// fn : Flash page read
//
// brife : Flash page write
//
// param : argc -> ��������
//         argv -> ����ָ��
//
// return : none
// format = [read] [offset] [len]
void Flash_read_cmd(size_t argc, char **argv)
{
  uint32_t offset = 0;
  uint32_t len    = 0;
  uint8_t* pAddr = NULL;
  
  if(argc != 3)
  {
    printf("%s: bad parameter count\r\n", argv[0]);
    return;
  }
  
  offset =  strtoul(argv[1],NULL,0);
  len = strtoul(argv[2],NULL,0);
  
  if(len == 0)
  {
    printf("%s: len = 0\r\n", argv[0]);
    return ;
  }
  //���ȼ��
  if(len > FLASH_MAX_STRING_LEN)
  {
    len = FLASH_MAX_STRING_LEN;
  }
  
  //��ȡָ���ַ����
  pAddr = (uint8_t*)(flash_info.opt_addr + offset);
  for (uint8_t i = 0 ; i < len; i++)
  {
    printf("0x%02X\t",*( pAddr + i ));
    if(!((i+1)%8))
    {
      printf("\r\n");
    }
  }

}
