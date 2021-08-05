#include "nrf_delay.h"
#include "sdk_config.h"
#include "nrfx_spim.h"
#include "eink.h"

/**
 * full screen update LUT
**/
const unsigned char lut_w1[] =
{
0x60	,0x5A	,0x5A	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
 	
};	
const unsigned char lut_b1[] =
{
0x90	,0x5A	,0x5A	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,


};
/**
 * partial screen update LUT
**/
const unsigned char lut_w[] =
{
0x60	,0x01	,0x01	,0x00	,0x00	,0x01	,
0x80	,0x0f	,0x00	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,

};	
const unsigned char lut_b[] =
{
0x90	,0x01	,0x01	,0x00	,0x00	,0x01	,
0x40	,0x0f	,0x00	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,

};

bool eink_update;

//SPI驱动程序实例ID,ID和外设编号对应，0:SPI0  1:SPI1 2:SPI2
#define SPI_INSTANCE  0 
//定义SPI驱动呢程序实例
static const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);
static volatile bool spi_xfer_done;  //SPI数据传输完成标志

static uint8_t    spi_tx_buf[256];  //SPI发送缓存数组
static uint8_t    spi_rx_buf[256];  //SPI接收缓存数组

static nrfx_spim_xfer_desc_t spim0_xfer;
//SPI事件处理函数
void spi_event_handler(nrfx_spim_evt_t const * p_event,
                       void *                    p_context)
{
  //设置SPI传输完成  
	spi_xfer_done = true;
}

void SPI_EP_Init(void)
{
    //配置用于SPI片选的引脚为输出
	  nrf_gpio_cfg_output(EINK_CS);
	  nrf_gpio_cfg_output(EINK_DC);
	  nrf_gpio_cfg_output(EINK_RESET);
	  nrf_gpio_cfg_input(EINK_BUSY,NRF_GPIO_PIN_NOPULL);
    EPD_W21_CS_1;//拉高CS
	  EPD_W21_RST_1;
    
		
	  //使用默认配置参数初始化SPI配置结构体
	  nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
	  //重写SPI信号连接的引脚配置
    spi_config.ss_pin   = NRFX_SPIM_PIN_NOT_USED;
    spi_config.miso_pin = NRFX_SPIM_PIN_NOT_USED;
    spi_config.mosi_pin = EINK_MOSI;
    spi_config.sck_pin  = EINK_CLK;
	  //初始化SPI
    APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, spi_event_handler, NULL));
}

void SPI_EP_unInit(void)
{
		SPI_EP_Init();
}

void SPI_Write(unsigned char value)                                    
{                                                           
	  uint8_t len = 1;

    spi_tx_buf[0] = value;
	  spi_xfer_done = false;
	
	  spim0_xfer.p_tx_buffer = spi_tx_buf;
    spim0_xfer.tx_length =len;
    spim0_xfer.p_rx_buffer = spi_rx_buf;
    spim0_xfer.rx_length = 0;
	
	
	  APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &spim0_xfer, 0));
    while(!spi_xfer_done);
		//SPI_EP_Init();
		/**/
}
static unsigned char ReadBusy(void)
{
  unsigned long i=0;

  for(i=0;i<400;i++){
	    if(isEPD_W21_BUSY==EPD_W21_BUSY_LEVEL) {
      return 1;
    }
	  nrf_delay_ms(10);
  }
  return 0;
}

void Epaper_Write_Command(unsigned char command)
{                
	EPD_W21_DC_0;		// command write
	EPD_W21_CS_0;
	SPI_Write(command);
	EPD_W21_CS_1;
/**/
}

void Epaper_Write_Data(unsigned char data)
{
	EPD_W21_CS_1;
	EPD_W21_CS_0;
	EPD_W21_DC_1;  // D/C#   0:command  1:data

	SPI_Write(data);
	EPD_W21_CS_1;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//1in54
void EPD_HW_Init1(void)
{
	EPD_W21_RST_0;  // Module reset   
	nrf_delay_ms(10);//At least 10ms delay 
	EPD_W21_RST_1;
	//nrf_delay_ms(10); //At least 10ms delay 
}
void EPD_HW_Init2(void)
{ 
	Epaper_Write_Command(0x12);  //SWRESET
}
void EPD_HW_Init3(void)
{
	Epaper_Write_Command(0x01); //Driver output control      
	Epaper_Write_Data(0xC7);
	Epaper_Write_Data(0x00);
	Epaper_Write_Data(0x00);

	Epaper_Write_Command(0x11); //data entry mode       
	Epaper_Write_Data(0x01);
 
	Epaper_Write_Command(0x44); //set Ram-X address start/end position   
	Epaper_Write_Data(0x00);
	Epaper_Write_Data(0x18);    //0x0C-->(18+1)*8=200

	Epaper_Write_Command(0x45); //set Ram-Y address start/end position          
	Epaper_Write_Data(0xC7);    //0xC7-->(199+1)=200
	Epaper_Write_Data(0x00);
	Epaper_Write_Data(0x00);
	Epaper_Write_Data(0x00); 

	Epaper_Write_Command(0x3C); //BorderWavefrom
	Epaper_Write_Data(0x05);	
	  	
  Epaper_Write_Command(0x18); //Read built-in temperature sensor
	Epaper_Write_Data(0x80);	

	Epaper_Write_Command(0x4E);   // set RAM x address count to 0;
	Epaper_Write_Data(0x00);
	Epaper_Write_Command(0x4F);   // set RAM y address count to 0X199;    
	Epaper_Write_Data(0xC7);
	Epaper_Write_Data(0x00);
}
void EPD_HW_Init(void)
{
	EPD_W21_RST_0;  // Module reset   
	nrf_delay_ms(10);//At least 10ms delay 
	EPD_W21_RST_1;
	nrf_delay_ms(10); //At least 10ms delay 
	
	ReadBusy();  
	Epaper_Write_Command(0x12);  //SWRESET
	ReadBusy();   
		
	Epaper_Write_Command(0x01); //Driver output control      
	Epaper_Write_Data(0xC7);
	Epaper_Write_Data(0x00);
	Epaper_Write_Data(0x00);

	Epaper_Write_Command(0x11); //data entry mode       
	Epaper_Write_Data(0x01);
 
	Epaper_Write_Command(0x44); //set Ram-X address start/end position   
	Epaper_Write_Data(0x00);
	Epaper_Write_Data(0x18);    //0x0C-->(18+1)*8=200

	Epaper_Write_Command(0x45); //set Ram-Y address start/end position          
	Epaper_Write_Data(0xC7);    //0xC7-->(199+1)=200
	Epaper_Write_Data(0x00);
	Epaper_Write_Data(0x00);
	Epaper_Write_Data(0x00); 

	Epaper_Write_Command(0x3C); //BorderWavefrom
	Epaper_Write_Data(0x05);	
	  	
  Epaper_Write_Command(0x18); //Read built-in temperature sensor
	Epaper_Write_Data(0x80);	

	Epaper_Write_Command(0x4E);   // set RAM x address count to 0;
	Epaper_Write_Data(0x00);
	Epaper_Write_Command(0x4F);   // set RAM y address count to 0X199;    
	Epaper_Write_Data(0xC7);
	Epaper_Write_Data(0x00);
  /*ReadBusy();
	*/
}


void EPD_Update(void)
{   
  Epaper_Write_Command(0x22); //Display Update Control
  Epaper_Write_Data(0xF7);   
  Epaper_Write_Command(0x20); //Activate Display Update Sequence
  //ReadBusy();   

}
/*When the electronic paper screen is updated, do not unplug the electronic paper to avoid damage to the screen*/
void EPD_Part_Update(void)
{
	Epaper_Write_Command(0x22); //Display Update Control
	Epaper_Write_Data(0xFF);   
	Epaper_Write_Command(0x20); //Activate Display Update Sequence
	//ReadBusy(); 			
}


void EPD_SetRAMValue( const unsigned char * datas)
{
	unsigned int i;   
	const unsigned char  *datas_flag;   
	datas_flag=datas;

	
	Epaper_Write_Command(0x24);   //Write Black and White image to RAM
	for(i=0;i<ALLSCREEN_GRAGHBYTES;i++)
	{               
		Epaper_Write_Data(*datas);
		datas++;
	}
	datas=datas_flag;
	Epaper_Write_Command(0x26);   //Write Black and White image to RAM
	for(i=0;i<ALLSCREEN_GRAGHBYTES;i++)
	{               
		Epaper_Write_Data(*datas);
		datas++;
	}
	//EPD_Update();		 
	 
}


void EPD_Dis_Part(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_LINE,unsigned int PART_COLUMN)
{
	unsigned int i;  
	unsigned int x_end,y_start1,y_start2,y_end1,y_end2;
	x_start=x_start/8;
	x_end=x_start+PART_LINE/8-1; 
	
	y_start1=0;
	y_start2=y_start;
	if(y_start>=256)
	{
		y_start1=y_start2/256;
		y_start2=y_start2%256;
	}
	y_end1=0;
	y_end2=y_start+PART_COLUMN-1;
	if(y_end2>=256)
	{
		y_end1=y_end2/256;
		y_end2=y_end2%256;		
	}		
	
//	// Add hardware reset to prevent background color change
//	EPD_W21_RST_0;  // Module reset   
//	nrf_delay_ms(10);//At least 10ms delay 
//	EPD_W21_RST_1;
//	nrf_delay_ms(10); //At least 10ms delay 	
//	  //Lock the border to prevent flashing
//	Epaper_Write_Command(0x3C); //BorderWavefrom,
//	Epaper_Write_Data(0x80);
	
	Epaper_Write_Command(0x44);       // set RAM x address start/end, in page 35
	Epaper_Write_Data(x_start);    // RAM x address start at 00h;
	Epaper_Write_Data(x_end);    // RAM x address end at 0fh(15+1)*8->128 
	Epaper_Write_Command(0x45);       // set RAM y address start/end, in page 35
	Epaper_Write_Data(y_start2);    // RAM y address start at 0127h;
	Epaper_Write_Data(y_start1);    // RAM y address start at 0127h;
	Epaper_Write_Data(y_end2);    // RAM y address end at 00h;
	Epaper_Write_Data(y_end1);    // ????=0	


	Epaper_Write_Command(0x4E);   // set RAM x address count to 0;
	Epaper_Write_Data(x_start); 
	Epaper_Write_Command(0x4F);   // set RAM y address count to 0X127;    
	Epaper_Write_Data(y_start2);
	Epaper_Write_Data(y_start1);
	
	
	 Epaper_Write_Command(0x24);   //Write Black and White image to RAM
   for(i=0;i<PART_COLUMN*PART_LINE/8;i++)
   {                         
     Epaper_Write_Data(*datas);
			datas++;
   } 
	 EPD_Part_Update();

}

//////////////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////////////////////////
//1in02
unsigned char lcd_chkstatus(void)
{
	unsigned long i=0;

  for(i=0;i<400;i++){
			Epaper_Write_Command(0x71);
	    if(isEPD_W21_BUSY==1) {
      return 1;
    }
	  nrf_delay_ms(50);
  }
  return 0;	
}
void EPD_HW_1in02_Init(void)
{
	EPD_W21_RST_0;  // Module reset   
	nrf_delay_ms(10);//At least 10ms delay 
	EPD_W21_RST_1;
	
	Epaper_Write_Command(0xD2);			
	Epaper_Write_Data(0x3F);
						 
	Epaper_Write_Command(0x00);  			
	Epaper_Write_Data (0x6F);  //from outside

	Epaper_Write_Command(0x01);  //power setting
	Epaper_Write_Data (0x03);	    
	Epaper_Write_Data (0x00);
	Epaper_Write_Data (0x2b);		
	Epaper_Write_Data (0x2b); 
	
	Epaper_Write_Command(0x06);  //Configuring the charge pump
	Epaper_Write_Data(0x3f);
	
	Epaper_Write_Command(0x2A);  //Setting XON and the options of LUT
	Epaper_Write_Data(0x00); 
	Epaper_Write_Data(0x00); 
	
	Epaper_Write_Command(0x30);  //Set the clock frequency
	Epaper_Write_Data(0x13); //50Hz

	Epaper_Write_Command(0x50);  //Set VCOM and data output interval
	Epaper_Write_Data(0x57);			

	Epaper_Write_Command(0x60);  //Set The non-overlapping period of Gate and Source.
	Epaper_Write_Data(0x22);

	Epaper_Write_Command(0x61);  //resolution setting
	Epaper_Write_Data (0x50);    //source 128 	 
	Epaper_Write_Data (0x80);       

	Epaper_Write_Command(0x82);  //sets VCOM_DC value
	Epaper_Write_Data(0x12);  //-1v

	Epaper_Write_Command(0xe3);//Set POWER SAVING
	Epaper_Write_Data(0x33);	
	
	unsigned int count;
	Epaper_Write_Command(0x23);
	for(count=0;count<42;count++)	     
	{Epaper_Write_Data(lut_w1[count]);}    
	
	Epaper_Write_Command(0x24);
	for(count=0;count<42;count++)	     
	{Epaper_Write_Data(lut_b1[count]);} 
 /**/
}

void EPD_HW_1in02_Part_Init(void)
{
	EPD_W21_RST_0;  // Module reset   
	nrf_delay_ms(10);//At least 10ms delay 
	EPD_W21_RST_1;
	nrf_delay_ms(10);//At least 10ms delay 
	
	//EPD_1IN02_Reset(); 
	
	Epaper_Write_Command(0xD2);			
	Epaper_Write_Data(0x3F);
						 
	Epaper_Write_Command(0x00);  			
	Epaper_Write_Data (0x6F);  //from outside

	Epaper_Write_Command(0x01);  //power setting
	Epaper_Write_Data (0x03);	    
	Epaper_Write_Data (0x00);
	Epaper_Write_Data (0x2b);		
	Epaper_Write_Data (0x2b); 
	
	Epaper_Write_Command(0x06);  //Configuring the charge pump
	Epaper_Write_Data(0x3f);
	
	Epaper_Write_Command(0x2A);  //Setting XON and the options of LUT
	Epaper_Write_Data(0x00); 
	Epaper_Write_Data(0x00); 
	
	Epaper_Write_Command(0x30);  //Set the clock frequency
	Epaper_Write_Data(0x05); //50Hz

	Epaper_Write_Command(0x50);  //Set VCOM and data output interval
	Epaper_Write_Data(0xF2);			

	Epaper_Write_Command(0x60);  //Set The non-overlapping period of Gate and Source.
	Epaper_Write_Data(0x22);

	Epaper_Write_Command(0x61);  //resolution setting
	Epaper_Write_Data (0x50);    //source 128 	 
	Epaper_Write_Data (0x80);       

	Epaper_Write_Command(0x82);  //sets VCOM_DC value
	Epaper_Write_Data(0x00);  //-1v

	Epaper_Write_Command(0xe3);//Set POWER SAVING
	Epaper_Write_Data(0x33);	
	//

	unsigned int count;
	Epaper_Write_Command(0x23);
	for(count=0;count<42;count++)	     
	{Epaper_Write_Data(lut_w[count]);}    
	
	Epaper_Write_Command(0x24);
	for(count=0;count<42;count++)	     
	{Epaper_Write_Data(lut_b[count]);} 
	
}

void EPD_1in02_Update(void)
{    
		Epaper_Write_Command(0x04);
		//nrf_delay_ms(1);
		lcd_chkstatus();
		Epaper_Write_Command(0x12);
		nrf_delay_ms(10);
		lcd_chkstatus();
		//eink_chkstatus();
		Epaper_Write_Command(0x02);
		//nrf_delay_ms(10);
		lcd_chkstatus();
		//eink_chkstatus();
}

void EPD_1IN02_Clear(void)
{ 
	unsigned int i;
	Epaper_Write_Command(0x10);
	for(i=0;i<1280;i++){
		Epaper_Write_Data(0X00);
	}
	Epaper_Write_Command(0x13);	       //Transfer new data
	for(i=0;i<1280;i++){
		Epaper_Write_Data(0xff);
	}
	EPD_1in02_Update();
}

void EPD_1IN02_Display(const unsigned char *Image)
{ 
	UWORD Width;
	Width = (EPD_1IN02_WIDTH % 8 == 0)? (EPD_1IN02_WIDTH / 8 ): (EPD_1IN02_WIDTH / 8 + 1);
	//EPD_1IN02_Init();
	Epaper_Write_Command(0x10);
	for (UWORD j = 0; j < EPD_1IN02_HEIGHT; j++) {
        for (UWORD i = 0; i < Width; i++) {
            Epaper_Write_Data(0xff);
        }
	}

	Epaper_Write_Command(0x13);
	for (UWORD j = 0; j < EPD_1IN02_HEIGHT; j++) {
        for (UWORD i = 0; i < Width; i++) {
            Epaper_Write_Data(Image[i + j * Width]);
        }
	}
	//EPD_1in02_Update();
	eink_update = 1;
	//EPD_1in02_Update_main();
}
void EPD_1IN02_DisplayPartial(const unsigned char *old_Image, const unsigned char *Image)
{
    /* Set partial Windows */
    Epaper_Write_Command(0x91);		//This command makes the display enter partial mode
    Epaper_Write_Command(0x90);		//resolution setting
    Epaper_Write_Data(0);           //x-start
    Epaper_Write_Data(79);       //x-end

    Epaper_Write_Data(0);
    Epaper_Write_Data(127);  //y-end
    Epaper_Write_Data(0x00);

    UWORD Width;
    Width = (EPD_1IN02_WIDTH % 8 == 0)? (EPD_1IN02_WIDTH / 8 ): (EPD_1IN02_WIDTH / 8 + 1);

    /* send data */
    Epaper_Write_Command(0x10);
    for (UWORD j = 0; j < EPD_1IN02_HEIGHT; j++) {
        for (UWORD i = 0; i < Width; i++) {
            Epaper_Write_Data(old_Image[i + j * Width]);
        }
    }

    Epaper_Write_Command(0x13);
    for (UWORD j = 0; j < EPD_1IN02_HEIGHT; j++) {
        for (UWORD i = 0; i < Width; i++) {
            Epaper_Write_Data(Image[i + j * Width]);
        }
    }

    /* Set partial refresh */
    //EPD_1in02_Update();
		eink_update = 1;
}
void EPD_SetRAMValue_1in02(const unsigned char *old_Image)
{
    /* Set partial Windows */
    Epaper_Write_Command(0x91);		//This command makes the display enter partial mode
    Epaper_Write_Command(0x90);		//resolution setting
    Epaper_Write_Data(0);           //x-start
    Epaper_Write_Data(79);       //x-end

    Epaper_Write_Data(0);
    Epaper_Write_Data(127);  //y-end
    Epaper_Write_Data(0x00);

    UWORD Width;
    Width = (EPD_1IN02_WIDTH % 8 == 0)? (EPD_1IN02_WIDTH / 8 ): (EPD_1IN02_WIDTH / 8 + 1);

    /* send data */
    Epaper_Write_Command(0x10);
    for (UWORD j = 0; j < EPD_1IN02_HEIGHT; j++) {
        for (UWORD i = 0; i < Width; i++) {
            Epaper_Write_Data(old_Image[i + j * Width]);
        }
    }
}

void EPD_1IN02_Partial_new_Image(const unsigned char *Image)
{
		UWORD Width;
		Width = (EPD_1IN02_WIDTH % 8 == 0)? (EPD_1IN02_WIDTH / 8 ): (EPD_1IN02_WIDTH / 8 + 1);
    Epaper_Write_Command(0x13);
    for (UWORD j = 0; j < EPD_1IN02_HEIGHT; j++) {
        for (UWORD i = 0; i < Width; i++) {
            Epaper_Write_Data(Image[i + j * Width]);
        }
    }

    /* Set partial refresh */
    //EPD_1in02_Update();
		eink_update = 1;
}

void EPD_Dis_Part_1in02(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_LINE,unsigned int PART_COLUMN)
{

		unsigned int x_end,y_end;

		x_end=x_start+PART_LINE-1; 

		y_end=y_start+PART_COLUMN-1;

			/* Set partial Windows */
		Epaper_Write_Command(0x91);		//This command makes the display enter partial mode
		Epaper_Write_Command(0x90);		//resolution setting
		Epaper_Write_Data(x_start);           //x-start
		Epaper_Write_Data(x_end);       //x-end

		Epaper_Write_Data(y_start);
		Epaper_Write_Data(y_end);  //y-end
		Epaper_Write_Data(0x00);
		
		
		UWORD Width;
		Width = (PART_LINE % 8 == 0)? (PART_LINE / 8 ): (PART_LINE / 8 + 1);
	
    Epaper_Write_Command(0x13);
    for (UWORD j = 0; j < PART_COLUMN; j++) {
        for (UWORD i = 0; i < Width; i++) {
            Epaper_Write_Data(datas[i + j * Width]);
        }
    }

    /* Set partial refresh */
    //EPD_1in02_Update();
		eink_update = 1;
}

/////////////////////////////////////////////////////////////////////////////////////////



/*********************************END FILE*************************************/
