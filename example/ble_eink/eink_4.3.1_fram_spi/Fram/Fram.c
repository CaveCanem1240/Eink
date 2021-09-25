
#include "Fram.h"
#include "nrf_delay.h"
#include "sdk_config.h"
#include "nrfx_spim.h"
#include "nrf_pwr_mgmt.h"



#define GAME_NUM_ADDR 0x0000

#define CHAPTER_NUM_ADDR 0x0001

#define DISPLAY_IMG_ADDR 0x0002

#define MENU_DATA_ADDR 0x138A

#define SOKUBAN_DATA_ADDR 0x138E


//SPI驱动程序实例ID,ID和外设编号对应，0:SPI0  1:SPI1 2:SPI2
#define SPI_INSTANCE2  2
//定义SPI驱动呢程序实例
static const nrfx_spim_t spi2 = NRFX_SPIM_INSTANCE(SPI_INSTANCE2);
static volatile bool spi2_xfer_done;  //SPI数据传输完成标志

static uint8_t    spi2_tx_buf[5003];  //SPI发送缓存数组
static uint8_t    spi2_rx_buf[5003];  //SPI接收缓存数组

static nrfx_spim_xfer_desc_t spim2_xfer;
//SPI事件处理函数
void spi2_event_handler(nrfx_spim_evt_t const * p_event,
                       void *                    p_context)
{ 
	spi2_xfer_done = true;
}

void FRAM_SPI2_Init(void)
{
		
	  //使用默认配置参数初始化SPI配置结构体
	  nrfx_spim_config_t spi2_config = NRFX_SPIM_DEFAULT_CONFIG;
	  //重写SPI信号连接的引脚配置
    spi2_config.ss_pin   = FRAM_CS;
    spi2_config.miso_pin = FRAM_MISO;
    spi2_config.mosi_pin = FRAM_MOSI;
    spi2_config.sck_pin  = FRAM_CLK;
		spi2_config.frequency  = NRF_SPIM_FREQ_8M;//NRF_SPIM_FREQ_125K;
	  //初始化SPI
    APP_ERROR_CHECK(nrfx_spim_init(&spi2, &spi2_config, spi2_event_handler, NULL));
}

void FRAM_SPI2_unInit(void)
{
		nrfx_spim_uninit(&spi2);
		nrf_gpio_cfg_default(FRAM_CLK);
}

void SPI2_Write_enable(void)
{                                                           
		uint8_t op_code = 0x06;
		spi2_tx_buf[0] = op_code;
		
    //spi_tx_buf = datas;
	  spi2_xfer_done = false;
		
		
	  spim2_xfer.p_tx_buffer = spi2_tx_buf;
    spim2_xfer.tx_length = 1;
    spim2_xfer.p_rx_buffer = spi2_rx_buf;
    spim2_xfer.rx_length = 0;
	
		
	  APP_ERROR_CHECK(nrfx_spim_xfer(&spi2, &spim2_xfer, 0));
    while(!spi2_xfer_done){
			__WFE();
			__SEV();
			__WFI();
		}
}
void SPI2_Write_disable(void)
{                                                           
		uint8_t op_code = 0x04;
		spi2_tx_buf[0] = op_code;
		
    //spi_tx_buf = datas;
	  spi2_xfer_done = false;
		
		
	  spim2_xfer.p_tx_buffer = spi2_tx_buf;
    spim2_xfer.tx_length = 1;
    spim2_xfer.p_rx_buffer = spi2_rx_buf;
    spim2_xfer.rx_length = 0;
	
		
	  APP_ERROR_CHECK(nrfx_spim_xfer(&spi2, &spim2_xfer, 0));
    while(!spi2_xfer_done){
			__WFE();
			__SEV();
			__WFI();
		}
}

void SPI2_Read_reg(uint8_t * rx_data)
{                                                           
		uint8_t op_code = 0x05;
		spi2_tx_buf[0] = op_code;
		
		
		uint8_t temp_rx[2];
	
    //spi_tx_buf = datas;
	  spi2_xfer_done = false;
		
	  spim2_xfer.p_tx_buffer = spi2_tx_buf;
    spim2_xfer.tx_length = 1;
    spim2_xfer.p_rx_buffer = temp_rx;
    spim2_xfer.rx_length = 2;
	
		
	  APP_ERROR_CHECK(nrfx_spim_xfer(&spi2, &spim2_xfer, 0));
    while(!spi2_xfer_done){
			__WFE();
			__SEV();
			__WFI();
		}
		*rx_data = temp_rx[1];
}
void SPI2_Write_reg(uint8_t reg)
{                                                           
		uint8_t op_code = 0x01;
		spi2_tx_buf[0] = op_code;
		spi2_tx_buf[1] = reg;
		
    //spi_tx_buf = datas;
	  spi2_xfer_done = false;
		
	  spim2_xfer.p_tx_buffer = spi2_tx_buf;
    spim2_xfer.tx_length = 2;
    spim2_xfer.p_rx_buffer = spi2_rx_buf;
    spim2_xfer.rx_length = 0;
		
		
	  APP_ERROR_CHECK(nrfx_spim_xfer(&spi2, &spim2_xfer, 0));
    while(!spi2_xfer_done){
			__WFE();
			__SEV();
			__WFI();
		}
}


void SPI2_Read_bytes(uint16_t Addr, uint8_t * rx_data, uint32_t length)
{                                                           
	  uint32_t len = length + 3;
		
		uint8_t op_code = 0x03;
		spi2_tx_buf[0] = op_code;
		spi2_tx_buf[1] = (uint8_t) (Addr >> 8);
		spi2_tx_buf[2] = (uint8_t) (Addr & 0x00FF);
		
		
		uint8_t temp_rx[len];
    //spi_tx_buf = datas;
	  spi2_xfer_done = false;
		
		
	  spim2_xfer.p_tx_buffer = spi2_tx_buf;
    spim2_xfer.tx_length = 3;
    spim2_xfer.p_rx_buffer = temp_rx;
    spim2_xfer.rx_length = len;
		

	  APP_ERROR_CHECK(nrfx_spim_xfer(&spi2, &spim2_xfer, 0));
    while(!spi2_xfer_done){
			__WFE();
			__SEV();
			__WFI();
		}
		for(uint32_t i=3;i<len;i++){
				*rx_data = temp_rx[i];
				rx_data++;
		}
}


void SPI2_Write_bytes(uint16_t Addr, const unsigned char * datas, uint32_t length)
{                                                           
	  uint32_t len = length + 3;
		
		uint8_t op_code = 0x02;
		spi2_tx_buf[0] = op_code;
		spi2_tx_buf[1] = (uint8_t) (Addr >> 8);
		spi2_tx_buf[2] = (uint8_t) (Addr & 0x00FF);
		
		for(uint16_t i=3; i<len; i++){
				spi2_tx_buf[i] = *datas;
				datas++;
		}
    //spi_tx_buf = datas;
	  spi2_xfer_done = false;
		
	  spim2_xfer.p_tx_buffer = spi2_tx_buf;
    spim2_xfer.tx_length = len;
    spim2_xfer.p_rx_buffer = spi2_rx_buf;
    spim2_xfer.rx_length = 0;
		
		
	  APP_ERROR_CHECK(nrfx_spim_xfer(&spi2, &spim2_xfer, 0));
    while(!spi2_xfer_done){
			__WFE();
			__SEV();
			__WFI();
		}
}

/**********************************************************************************************************/



uint8_t Find_fram_game_num(void)
{
		uint8_t game_num;
		SPI2_Read_bytes(GAME_NUM_ADDR, &game_num, 1);
		return game_num;
}

void Update_fram_game_num(uint8_t game_num)
{
		SPI2_Write_enable();
		SPI2_Write_bytes(GAME_NUM_ADDR, &game_num, 1);
}

uint8_t Find_fram_chapter_num(void)
{
		uint8_t chapter_num;
		SPI2_Read_bytes(CHAPTER_NUM_ADDR, &chapter_num, 1);
		return chapter_num;
}

void Update_fram_chapter_num(uint8_t chapter_num)
{
		SPI2_Write_enable();
		SPI2_Write_bytes(CHAPTER_NUM_ADDR, &chapter_num, 1);
}




void Find_Display_IMG(uint8_t * datas)
{
		uint8_t * p_PIC;
		p_PIC = datas;
		uint16_t PIC_Addr = DISPLAY_IMG_ADDR;
		for(uint8_t i=0;i<20;i++){
				SPI2_Read_bytes(PIC_Addr, p_PIC, 250);
				PIC_Addr += 250;
				p_PIC += 250;
		}
}

void Update_Display_IMG(uint8_t * datas)
{
		uint8_t * p_PIC;
		p_PIC = datas;
		uint16_t PIC_Addr = DISPLAY_IMG_ADDR;
		for(uint8_t i=0;i<20;i++){
				SPI2_Write_enable();
				SPI2_Write_bytes(PIC_Addr, p_PIC, 250);
				PIC_Addr += 250;
				p_PIC += 250;
		}
}



void load_menu_data_from_fram(uint8_t * menu_data)
{
		uint8_t * p_PIC;
		p_PIC = menu_data;
		uint16_t PIC_Addr = MENU_DATA_ADDR;
		SPI2_Read_bytes(PIC_Addr, p_PIC, 4);
}

void store_menu_data_to_fram(uint8_t * menu_data)
{
		uint8_t * p_PIC;
		p_PIC = menu_data;
		uint16_t PIC_Addr = MENU_DATA_ADDR;
		SPI2_Write_enable();
		SPI2_Write_bytes(PIC_Addr, p_PIC, 4);
}

void load_sokuban_data_from_fram(uint8_t * sokuban_data)
{
		uint8_t * p_PIC;
		p_PIC = sokuban_data;
		uint16_t PIC_Addr = SOKUBAN_DATA_ADDR;
		SPI2_Read_bytes(PIC_Addr, p_PIC, 81);
}

void store_sokuban_data_to_fram(uint8_t * sokuban_data)
{
		uint8_t * p_PIC;
		p_PIC = sokuban_data;
		uint16_t PIC_Addr = SOKUBAN_DATA_ADDR;
		SPI2_Write_enable();
		SPI2_Write_bytes(PIC_Addr, p_PIC, 81);
}




/*
				SPI2_EP_Init();
				UWORD Imagesize = screen_size;
				if((Map_Image = (UBYTE *)malloc(Imagesize)) == NULL) {
						
				}
				Paint_NewImage(Map_Image, screen_size_x, screen_size_y, 270, WHITE);
				Paint_SelectImage(Map_Image);
				Paint_Clear(WHITE);
				
				uint8_t * p_PIC;
				p_PIC = Map_Image;
				uint16_t PIC_Addr = 0x0000;
				for(uint8_t i=0;i<20;i++){
						SPI2_Write_enable();
						SPI2_Write_bytes(PIC_Addr, p_PIC, 250);
						PIC_Addr += 250;
						p_PIC += 250;
				}
				unsigned char temp_pic[5000];
				p_PIC = temp_pic;
				PIC_Addr = 0x0000;
				for(uint8_t i=0;i<20;i++){
						SPI2_Read_bytes(PIC_Addr, p_PIC, 250);
						PIC_Addr += 250;
						p_PIC += 250;
				}
				SPI2_EP_unInit();
				
*/



