
//#include "nrf_delay.h"
#include "nrf_gpio.h"

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

#define MONOMSB_MODE 1
#define MONOLSB_MODE 2 
#define RED_MODE     3


#define ALLSCREEN_GRAGHBYTES	5000

#define EPD_1IN02_WIDTH   80
#define EPD_1IN02_HEIGHT  128
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

extern bool eink_update;

void SPI_EP_Init(void);
void SPI_EP_unInit(void);
void SPI_GPIO_Init(void);			

unsigned char ReadBusy(void);

void Epaper_Write_Command(unsigned char command);
void Epaper_Write_Data(unsigned char data);

void EPD_HW_Init(void); //Electronic paper initialization
void EPD_HW_Init1(void);
void EPD_HW_Init2(void);
void EPD_HW_Init3(void);
void EPD_Update(void);

void EPD_Part_Update(void); 

//Display 
void EPD_SetRAMValue(const unsigned char * datas);
void EPD_SetRAMValue_BaseMap( const unsigned char * datas);
void EPD_Dis_Part(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_LINE,unsigned int PART_COLUMN);


void EPD_HW_1in02_Init(void);
void EPD_HW_1in02_Part_Init(void);
void EPD_1IN02_Display(const unsigned char *Image);
void EPD_1IN02_DisplayPartial(const unsigned char *old_Image, const unsigned char *Image);
void EPD_SetRAMValue_1in02(const unsigned char *old_Image);
void EPD_1IN02_Partial_new_Image(const unsigned char *Image);
void EPD_Dis_Part_1in02(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_LINE,unsigned int PART_COLUMN);

