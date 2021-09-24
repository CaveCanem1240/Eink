
#include "nrf_gpio.h"



#define   FRAM_CS       30
//#define   FRAM_DC       23
//#define   FRAM_RESET    22
//#define   FRAM_BUSY     27

#define   FRAM_MOSI     2
#define   FRAM_MISO     28
#define   FRAM_CLK      31




#define FRAM_MOSI_0	nrf_gpio_pin_clear(FRAM_MOSI)
#define FRAM_MOSI_1	nrf_gpio_pin_set(FRAM_MOSI)


#define FRAM_CLK_0	nrf_gpio_pin_clear(FRAM_CLK)
#define FRAM_CLK_1	nrf_gpio_pin_set(FRAM_CLK)

#define FRAM_CS_0	nrf_gpio_pin_clear(FRAM_CS)
#define FRAM_CS_1	nrf_gpio_pin_set(FRAM_CS)

//#define FRAM_DC_0	nrf_gpio_pin_clear(FRAM_DC)
//#define FRAM_DC_1	nrf_gpio_pin_set(FRAM_DC)

//#define FRAM_RST_0	nrf_gpio_pin_clear(FRAM_RESET)
//#define FRAM_RST_1	nrf_gpio_pin_set(FRAM_RESET)

void FRAM_SPI2_Init(void);
void FRAM_SPI2_unInit(void);


void SPI2_Write_enable(void);
void SPI2_Write_disable(void);

void SPI2_Read_reg(uint8_t * rx_data);
void SPI2_Write_reg(uint8_t reg);

void SPI2_Read_bytes(uint16_t Addr, uint8_t * rx_data, uint32_t length);
void SPI2_Write_bytes(uint16_t Addr, const unsigned char * datas, uint32_t length);

uint8_t Find_fram_game_num(void);
void Update_fram_game_num(uint8_t game_num);
uint8_t Find_fram_chapter_num(void);
void Update_fram_chapter_num(uint8_t chapter_num);

void Find_Display_IMG(uint8_t * datas);
void Update_Display_IMG(uint8_t * datas);
void load_menu_data_from_fram(void);




