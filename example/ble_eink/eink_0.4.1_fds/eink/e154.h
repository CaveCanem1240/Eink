
//#include "nrf_delay.h"
#include "nrf_gpio.h"



#define MONOMSB_MODE 1
#define MONOLSB_MODE 2 
#define RED_MODE     3

#define MAX_LINE_BYTES 25// =200/8
#define MAX_COLUMN_BYTES  200

#define ALLSCREEN_GRAGHBYTES	5000


/*****************************************************************************
**引脚定义
*****************************************************************************/
#define   EINK_CS       24  //LCD_TFT数据/命令切换
#define   EINK_DC       23  //LCD_TFT数据
#define   EINK_RESET    22  //LCD_TFT时钟
#define   EINK_BUSY     27  //LCD_TFT时钟

#define   EINK_MOSI     26  //LCD_TFT片选
#define   EINK_CLK      25  //LCD_TFT复位



#define EPD_W21_MOSI_0	nrf_gpio_pin_clear(EINK_MOSI)
#define EPD_W21_MOSI_1	nrf_gpio_pin_set(EINK_MOSI)


#define EPD_W21_CLK_0	nrf_gpio_pin_clear(EINK_CLK)
#define EPD_W21_CLK_1	nrf_gpio_pin_set(EINK_CLK)

#define EPD_W21_CS_0	nrf_gpio_pin_clear(EINK_CS)
#define EPD_W21_CS_1	nrf_gpio_pin_set(EINK_CS)

#define EPD_W21_DC_0	nrf_gpio_pin_clear(EINK_DC)
#define EPD_W21_DC_1	nrf_gpio_pin_set(EINK_DC)

#define EPD_W21_RST_0	nrf_gpio_pin_clear(EINK_RESET)
#define EPD_W21_RST_1	nrf_gpio_pin_set(EINK_RESET)

//#define EPD_W21_BS_0	nrf_gpio_pin_clear(EINK_BUSY)

#define EPD_W21_BUSY_LEVEL 0
#define isEPD_W21_BUSY nrf_gpio_pin_read(EINK_BUSY) 



void SPI_EP_Init(void);
void SPI_EP_unInit(void);
void SPI_GPIO_Init(void);			
void Epaper_READBUSY(void);

void Epaper_Spi_WriteByte(unsigned char TxData);
unsigned char ReadBusy(void);

void Epaper_Write_Command(unsigned char command);
void Epaper_Write_Data(unsigned char data);

void EPD_HW_Init(void); //Electronic paper initialization
void EPD_Update(void);

void EPD_Part_Init(void);//Local refresh initialization
void EPD_Part_Update(void); 

void EPD_WhiteScreen_White(void);
void EPD_DeepSleep(void);
//Display 
void EPD_WhiteScreen_ALL(const unsigned char *datas);
void EPD_SetRAMValue_BaseMap(const unsigned char * datas);
void EPD_Dis_Part(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_COLUMN,unsigned int PART_LINE);
void EPD_Dis_Part_half1(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_COLUMN,unsigned int PART_LINE);
void EPD_Dis_Part_half2(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_COLUMN,unsigned int PART_LINE);
void EPD_Dis_Part_myself(unsigned int x_startA,unsigned int y_startA,const unsigned char * datasA,
	                       unsigned int x_startB,unsigned int y_startB,const unsigned char * datasB,
												 unsigned int x_startC,unsigned int y_startC,const unsigned char * datasC,
												 unsigned int x_startD,unsigned int y_startD,const unsigned char * datasD,
											   unsigned int x_startE,unsigned int y_startE,const unsigned char * datasE,
												 unsigned int PART_COLUMN,unsigned int PART_LINE
	                      );
