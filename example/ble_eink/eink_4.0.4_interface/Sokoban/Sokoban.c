#include <stdint.h>
#include <stdlib.h>
#include <string.h> //memset()
#include <stdbool.h>
#include <math.h>
#include "GUI_Paint.h"
#include "Sokoban.h"

const unsigned char gImage_wall[60] = { /* 0X00,0X01,0X14,0X00,0X14,0X00, */
0X00,0X00,0X00,0X7F,0X7F,0X60,0X7F,0X7F,0X60,0X7F,0X7F,0X60,0X00,0X00,0X00,0X77,
0XF7,0XE0,0X77,0XF7,0XE0,0X77,0XF7,0XE0,0X00,0X00,0X00,0X7F,0X7F,0X60,0X7F,0X7F,
0X60,0X7F,0X7F,0X60,0X00,0X00,0X00,0X77,0XF7,0XE0,0X77,0XF7,0XE0,0X77,0XF7,0XE0,
0X00,0X00,0X00,0X7F,0X7F,0X60,0X7F,0X7F,0X60,0X00,0X00,0X00,};

const unsigned char gImage_player[60] = { /* 0X00,0X01,0X14,0X00,0X14,0X00, */
0XFF,0XFF,0XF0,0XFE,0X07,0XF0,0XFE,0XC7,0XF0,0XFD,0XF3,0XF0,0XFC,0X63,0XF0,0XFD,
0XFB,0XF0,0XFC,0XF3,0XF0,0XFE,0X07,0XF0,0XF9,0XF1,0XF0,0XF3,0XFE,0XF0,0XF6,0XF6,
0XF0,0XEE,0XFB,0X70,0XEC,0X0B,0X70,0XEA,0XF4,0X70,0XE5,0X9A,0XF0,0XFD,0X9B,0XF0,
0XFD,0X9B,0XF0,0XFD,0X9B,0XF0,0XFE,0X63,0XF0,0XFF,0XFF,0XF0,};

const unsigned char gImage_box[60] = { /* 0X00,0X01,0X14,0X00,0X14,0X00, */
0XFF,0XFF,0XF0,0X80,0X00,0X10,0XBF,0XFF,0XD0,0XBF,0XFF,0XD0,0XB0,0X00,0XD0,0XB3,
0XFC,0XD0,0XB5,0XFA,0XD0,0XB6,0XF6,0XD0,0XB7,0X6E,0XD0,0XB7,0X9E,0XD0,0XB7,0X9E,
0XD0,0XB7,0X6E,0XD0,0XB6,0XF6,0XD0,0XB5,0XFA,0XD0,0XB3,0XFC,0XD0,0XB0,0X00,0XD0,
0XBF,0XFF,0XD0,0XBF,0XFF,0XD0,0X80,0X00,0X10,0XFF,0XFF,0XF0,};

const unsigned char gImage_box_over_point[60] = { /* 0X00,0X01,0X14,0X00,0X14,0X00, */
0X00,0X00,0X00,0X7F,0XFF,0XE0,0X40,0X00,0X20,0X40,0X00,0X20,0X4F,0XFF,0X20,0X4C,
0X03,0X20,0X4A,0X05,0X20,0X49,0X09,0X20,0X48,0X91,0X20,0X48,0X61,0X20,0X48,0X61,
0X20,0X48,0X91,0X20,0X49,0X09,0X20,0X4A,0X05,0X20,0X4C,0X03,0X20,0X4F,0XFF,0X20,
0X40,0X00,0X20,0X40,0X00,0X20,0X7F,0XFF,0XE0,0X00,0X00,0X00,};

const unsigned char gImage_point[60] = { /* 0X00,0X01,0X14,0X00,0X14,0X00, */
0XFF,0XFF,0XF0,0XFF,0XFF,0XF0,0XFF,0XFF,0XF0,0XEF,0XFF,0X70,0XF7,0XFE,0XF0,0XFB,
0XFD,0XF0,0XFD,0XFB,0XF0,0XFE,0XF7,0XF0,0XFF,0X6F,0XF0,0XFF,0X9F,0XF0,0XFF,0X9F,
0XF0,0XFF,0X6F,0XF0,0XFE,0XF7,0XF0,0XFD,0XFB,0XF0,0XFB,0XFD,0XF0,0XF7,0XFE,0XF0,
0XEF,0XFF,0X70,0XFF,0XFF,0XF0,0XFF,0XFF,0XF0,0XFF,0XFF,0XF0,};

const unsigned char gImage_player_over_point[60] = { /* 0X00,0X01,0X14,0X00,0X14,0X00, */
0XFF,0XFF,0XF0,0XFE,0X07,0XF0,0XFE,0XC7,0XF0,0XED,0XF3,0X70,0XF4,0X62,0XF0,0XF9,
0XF9,0XF0,0XFC,0XF3,0XF0,0XFE,0X07,0XF0,0XF9,0XF1,0XF0,0XF3,0XFE,0XF0,0XF6,0XF6,
0XF0,0XEE,0XFB,0X70,0XEC,0X03,0X70,0XE8,0XF0,0X70,0XE1,0X98,0XF0,0XF5,0X9A,0XF0,
0XED,0X9B,0X70,0XFD,0X9B,0XF0,0XFE,0X63,0XF0,0XFF,0XFF,0XF0,};


const unsigned char gImage_none[60] = { /* 0X00,0X01,0X14,0X00,0X14,0X00, */
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,};

MAP map;

uint8_t x_start = 15;
uint8_t y_start = 17;

uint8_t next_map[9][9];
uint32_t init_chapter = 0;

/*0:none, 1:wall, 2:box, 3:point, 4:player, 5:box-over-point, 6:player-over-point*/
extern uint8_t chapter_array[3][9][9] = {
			{		
				{0,0,1,1,1,1,1,0,0},
				{1,1,1,0,0,0,1,0,0},
				{1,3,4,2,0,0,1,0,0},
				{1,1,1,0,2,3,1,0,0},
				{1,3,1,1,2,0,1,0,0},
				{1,0,1,0,3,0,1,1,0},
				{1,2,0,5,2,2,3,1,0},
				{1,0,0,0,3,0,0,1,0},
				{1,1,1,1,1,1,1,1,0}
			},
			{		
				{0,0,1,1,1,1,1,0,0},
				{1,1,1,0,0,0,1,0,1},
				{1,3,0,4,2,0,1,0,2},
				{1,1,1,0,2,3,1,0,3},
				{1,3,1,1,2,0,1,0,4},
				{1,0,1,0,3,0,1,1,5},
				{1,2,0,2,2,2,3,1,6},
				{1,0,0,0,3,0,0,1,0},
				{1,1,1,1,1,1,1,1,0}
			},
			{		
				{0,0,1,1,1,1,1,0,0},
				{1,1,1,0,0,0,1,0,0},
				{1,3,6,5,3,0,1,0,0},
				{1,1,1,0,2,3,1,0,0},
				{1,3,1,1,2,0,1,0,0},
				{1,0,1,0,3,0,1,1,0},
				{1,2,0,5,2,2,3,1,0},
				{1,0,0,0,3,0,0,1,0},
				{1,1,1,1,1,1,1,1,0}
			}
		};

void MAP_INIT(uint32_t chapter){
		uint8_t *elements_array = chapter_array[chapter][0];
		map.ELEMENTS = elements_array;
		map.Width = sizeof(chapter_array[chapter][0])/sizeof(chapter_array[chapter][0][0]);
		map.Height = sizeof(chapter_array[chapter])/sizeof(chapter_array[chapter][0]);
		
}

void GEN_MAP_PIC(uint32_t chapter)
{
		MAP_INIT(chapter);
		uint8_t Height = 9;
		uint8_t Width = 9;
		uint8_t * p_element;
		p_element = map.ELEMENTS;
		uint8_t x_addr = 0;
		uint8_t y_addr = 0;;
	
		Paint_DrawString_EN(5, 0, "Sokoban chapter-", &Font16, WHITE, BLACK);
		Paint_DrawNum(181, 0, chapter, &Font16, WHITE, BLACK);
		
		for(uint8_t row = 0; row < Height; row++){
				for(uint8_t column = 0; column < Width; column++){
						x_addr = x_start + 20*column;
						y_addr = y_start + 20*row;
						if(*p_element==0){
							Paint_DrawPic(x_addr, y_addr, gImage_none, 20, 20, WHITE, BLACK);
						}
						if(*p_element==1){
							Paint_DrawPic(x_addr, y_addr, gImage_wall, 20, 20, WHITE, BLACK);
						}
						if(*p_element==2){
							Paint_DrawPic(x_addr, y_addr, gImage_box, 20, 20, WHITE, BLACK);
						}
						if(*p_element==3){
							Paint_DrawPic(x_addr, y_addr, gImage_point, 20, 20, WHITE, BLACK);
						}
						if(*p_element==4){
							Paint_DrawPic(x_addr, y_addr, gImage_player, 20, 20, WHITE, BLACK);
						}
						if(*p_element==5){
							Paint_DrawPic(x_addr, y_addr, gImage_box_over_point, 20, 20, WHITE, BLACK);
						}
						if(*p_element==6){
							Paint_DrawPic(x_addr, y_addr, gImage_player_over_point, 20, 20, WHITE, BLACK);
						}

						p_element++;
				}
		}
}

void UPDATE_MAP_PIC(uint32_t chapter)
{
		MAP_INIT(chapter);
		uint8_t Height = 9;
		uint8_t Width = 9;
		uint8_t x_addr = 0;
		uint8_t y_addr = 0;;
	
		Paint_DrawString_EN(5, 0, "Sokoban chapter-", &Font16, WHITE, BLACK);
		Paint_DrawNum(181, 0, chapter, &Font16, WHITE, BLACK);
	
		for(uint8_t row = 0; row < Height; row++){
				for(uint8_t column = 0; column < Width; column++){
						x_addr = x_start + 20*column;
						y_addr = y_start + 20*row;
						if(next_map[row][column]==0){
							Paint_DrawPic(x_addr, y_addr, gImage_none, 20, 20, WHITE, BLACK);
						}
						if(next_map[row][column]==1){
							Paint_DrawPic(x_addr, y_addr, gImage_wall, 20, 20, WHITE, BLACK);
						}
						if(next_map[row][column]==2){
							Paint_DrawPic(x_addr, y_addr, gImage_box, 20, 20, WHITE, BLACK);
						}
						if(next_map[row][column]==3){
							Paint_DrawPic(x_addr, y_addr, gImage_point, 20, 20, WHITE, BLACK);
						}
						if(next_map[row][column]==4){
							Paint_DrawPic(x_addr, y_addr, gImage_player, 20, 20, WHITE, BLACK);
						}
						if(next_map[row][column]==5){
							Paint_DrawPic(x_addr, y_addr, gImage_box_over_point, 20, 20, WHITE, BLACK);
						}
						if(next_map[row][column]==6){
							Paint_DrawPic(x_addr, y_addr, gImage_player_over_point, 20, 20, WHITE, BLACK);
						}
				}
		}
}
uint8_t move_up(void)
{
		uint8_t Height = 9;
		uint8_t Width = 9;
		uint8_t num_box = 0;
		uint8_t last_map[Height][Width];
		uint8_t player_x,player_y;
		for(uint8_t i=0;i<Height;i++){
				for(uint8_t j=0;j<Width;j++){
						last_map[i][j] = next_map[i][j];
						if(last_map[i][j]==4 || last_map[i][j]==6){
								player_y = i;
								player_x = j;
						}
						if(last_map[i][j] == 2){
								num_box++;
						}
				}
		}
		
		/*0:none, 1:wall, 2:box, 3:point, 4:player, 5:box-over-point, 6:player-over-point*/
		if(last_map[player_y-1][player_x] != 1){
				if(last_map[player_y-1][player_x] == 0){
						next_map[player_y-1][player_x] = 4;
						if(last_map[player_y][player_x] == 4){
								next_map[player_y][player_x] = 0;
						}
						
						if(last_map[player_y][player_x] == 6){
								next_map[player_y][player_x] = 3;
						}
						
				}
				
				if(last_map[player_y-1][player_x] == 3){
						next_map[player_y-1][player_x] = 6;
						if(last_map[player_y][player_x] == 4){
								next_map[player_y][player_x] = 0;
						}
						
						if(last_map[player_y][player_x] == 6){
								next_map[player_y][player_x] = 3;
						}
						
				}
						
				if(last_map[player_y-1][player_x] == 2){
						if(last_map[player_y-2][player_x] == 0){
								next_map[player_y-1][player_x] = 4;
								next_map[player_y-2][player_x] = 2;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
						if(last_map[player_y-2][player_x] == 3){
								next_map[player_y-1][player_x] = 4;
								next_map[player_y-2][player_x] = 5;
								num_box--;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
				
				}
								
				if(last_map[player_y-1][player_x] == 5){
						if(last_map[player_y-2][player_x] == 0){
								num_box++;
								next_map[player_y-1][player_x] = 6;
								next_map[player_y-2][player_x] = 2;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
						if(last_map[player_y-2][player_x] == 3){
								next_map[player_y-1][player_x] = 6;
								next_map[player_y-2][player_x] = 5;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
				}
		}
		return num_box;
}
uint8_t move_down(void)
{
		uint8_t Height = 9;
		uint8_t Width = 9;
		uint8_t num_box = 0;
		uint8_t last_map[Height][Width];
		uint8_t player_x,player_y;
		for(uint8_t i=0;i<Height;i++){
				for(uint8_t j=0;j<Width;j++){
						last_map[i][j] = next_map[i][j];
						if(last_map[i][j]==4 || last_map[i][j]==6){
								player_y = i;
								player_x = j;
						}
						if(last_map[i][j] == 2){
								num_box++;
						}
				}
		}
		
		/*0:none, 1:wall, 2:box, 3:point, 4:player, 5:box-over-point, 6:player-over-point*/
		if(last_map[player_y+1][player_x] != 1){
				if(last_map[player_y+1][player_x] == 0){
						next_map[player_y+1][player_x] = 4;
						if(last_map[player_y][player_x] == 4){
								next_map[player_y][player_x] = 0;
						}
						
						if(last_map[player_y][player_x] == 6){
								next_map[player_y][player_x] = 3;
						}
						
				}
				
				if(last_map[player_y+1][player_x] == 3){
						next_map[player_y+1][player_x] = 6;
						if(last_map[player_y][player_x] == 4){
								next_map[player_y][player_x] = 0;
						}
						
						if(last_map[player_y][player_x] == 6){
								next_map[player_y][player_x] = 3;
						}
						
				}
						
				if(last_map[player_y+1][player_x] == 2){
						if(last_map[player_y+2][player_x] == 0){
								next_map[player_y+1][player_x] = 4;
								next_map[player_y+2][player_x] = 2;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
						if(last_map[player_y+2][player_x] == 3){
								next_map[player_y+1][player_x] = 4;
								next_map[player_y+2][player_x] = 5;
								num_box--;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
				
				}
								
				if(last_map[player_y+1][player_x] == 5){
						if(last_map[player_y+2][player_x] == 0){
								num_box++;
								next_map[player_y+1][player_x] = 6;
								next_map[player_y+2][player_x] = 2;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
						if(last_map[player_y+2][player_x] == 3){
								next_map[player_y+1][player_x] = 6;
								next_map[player_y+2][player_x] = 5;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
				}
		}
		return num_box;
}

uint8_t move_left(void)
{
		uint8_t Height = 9;
		uint8_t Width = 9;
		uint8_t num_box = 0;
		uint8_t last_map[Height][Width];
		uint8_t player_x,player_y;
		for(uint8_t i=0;i<Height;i++){
				for(uint8_t j=0;j<Width;j++){
						last_map[i][j] = next_map[i][j];
						if(last_map[i][j]==4 || last_map[i][j]==6){
								player_y = i;
								player_x = j;
						}
						if(last_map[i][j] == 2){
								num_box++;
						}
				}
		}
		
		/*0:none, 1:wall, 2:box, 3:point, 4:player, 5:box-over-point, 6:player-over-point*/
		if(last_map[player_y][player_x-1] != 1){
				if(last_map[player_y][player_x-1] == 0){
						next_map[player_y][player_x-1] = 4;
						if(last_map[player_y][player_x] == 4){
								next_map[player_y][player_x] = 0;
						}
						
						if(last_map[player_y][player_x] == 6){
								next_map[player_y][player_x] = 3;
						}
						
				}
				
				if(last_map[player_y][player_x-1] == 3){
						next_map[player_y][player_x-1] = 6;
						if(last_map[player_y][player_x] == 4){
								next_map[player_y][player_x] = 0;
						}
						
						if(last_map[player_y][player_x] == 6){
								next_map[player_y][player_x] = 3;
						}
						
				}
						
				if(last_map[player_y][player_x-1] == 2){
						if(last_map[player_y][player_x-2] == 0){
								next_map[player_y][player_x-1] = 4;
								next_map[player_y][player_x-2] = 2;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
						if(last_map[player_y][player_x-2] == 3){
								next_map[player_y][player_x-1] = 4;
								next_map[player_y][player_x-2] = 5;
								num_box--;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
				
				}
								
				if(last_map[player_y][player_x-1] == 5){
						if(last_map[player_y][player_x-2] == 0){
								num_box++;
								next_map[player_y][player_x-1] = 6;
								next_map[player_y][player_x-2] = 2;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
						if(last_map[player_y][player_x-2] == 3){
								next_map[player_y][player_x-1] = 6;
								next_map[player_y][player_x-2] = 5;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
				}
		}
		return num_box;
}
uint8_t move_right(void)
{
		uint8_t Height = 9;
		uint8_t Width = 9;
		uint8_t num_box = 0;
		uint8_t last_map[Height][Width];
		uint8_t player_x,player_y;
		for(uint8_t i=0;i<Height;i++){
				for(uint8_t j=0;j<Width;j++){
						last_map[i][j] = next_map[i][j];
						if(last_map[i][j]==4 || last_map[i][j]==6){
								player_y = i;
								player_x = j;
						}
						if(last_map[i][j] == 2){
								num_box++;
						}
				}
		}
		
		/*0:none, 1:wall, 2:box, 3:point, 4:player, 5:box-over-point, 6:player-over-point*/
		if(last_map[player_y][player_x+1] != 1){
				if(last_map[player_y][player_x+1] == 0){
						next_map[player_y][player_x+1] = 4;
						if(last_map[player_y][player_x] == 4){
								next_map[player_y][player_x] = 0;
						}
						
						if(last_map[player_y][player_x] == 6){
								next_map[player_y][player_x] = 3;
						}
						
				}
				
				if(last_map[player_y][player_x+1] == 3){
						next_map[player_y][player_x+1] = 6;
						if(last_map[player_y][player_x] == 4){
								next_map[player_y][player_x] = 0;
						}
						
						if(last_map[player_y][player_x] == 6){
								next_map[player_y][player_x] = 3;
						}
						
				}
						
				if(last_map[player_y][player_x+1] == 2){
						if(last_map[player_y][player_x+2] == 0){
								next_map[player_y][player_x+1] = 4;
								next_map[player_y][player_x+2] = 2;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
						if(last_map[player_y][player_x+2] == 3){
								next_map[player_y][player_x+1] = 4;
								next_map[player_y][player_x+2] = 5;
								num_box--;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
				
				}
								
				if(last_map[player_y][player_x+1] == 5){
						if(last_map[player_y][player_x+2] == 0){
								num_box++;
								next_map[player_y][player_x+1] = 6;
								next_map[player_y][player_x+2] = 2;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
						if(last_map[player_y][player_x+2] == 3){
								next_map[player_y][player_x+1] = 6;
								next_map[player_y][player_x+2] = 5;
								if(last_map[player_y][player_x] == 4){
										next_map[player_y][player_x] = 0;
								}
								
								if(last_map[player_y][player_x] == 6){
										next_map[player_y][player_x] = 3;
								}
								
						}
						
				}
		}
		return num_box;
}

/*operation:  0-null 1-up 2-down 3-left 4-right 5-enter 6-cancel*/

uint8_t GEN_NEXT_PIC(uint8_t operation, bool init_next_chapter, uint32_t chapter)
{
		
		uint8_t Height = 9;
		uint8_t Width = 9;
		if(init_next_chapter){
				
				for(uint8_t i=0;i<Height;i++){
						for(uint8_t j=0;j<Width;j++){
								next_map[i][j] = chapter_array[chapter][i][j];
						}
				}
				GEN_MAP_PIC(chapter);
				return 1;
		}
		else{
				uint8_t num_box;
				if(operation==1){
						num_box = move_up();
				}
				if(operation==2){
						num_box = move_down();
				}
				if(operation==3){
						num_box = move_left();
				}
				if(operation==4){
						num_box = move_right();
				}
				UPDATE_MAP_PIC(chapter);
				return num_box;
		}
}




