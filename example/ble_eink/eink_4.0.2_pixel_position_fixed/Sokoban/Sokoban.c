#include <stdint.h>
#include <stdlib.h>
#include <string.h> //memset()
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


MAP map;
int x_start = 15;
int y_start = 17;

/*0:none, 1:wall, 2:box, 3:point, 4:player, 5:box-over-point*/
int chapter_array[3][9][8] = {
			{		
				{0,0,1,1,1,1,1,0},
				{1,1,1,0,0,0,1,0},
				{1,3,4,2,0,0,1,0},
				{1,1,1,0,2,3,1,0},
				{1,3,1,1,2,0,1,0},
				{1,0,1,0,3,0,1,1},
				{1,2,0,5,2,2,3,1},
				{1,0,0,0,3,0,0,1},
				{1,1,1,1,1,1,1,1}
			},
			{		
				{0,0,1,1,1,1,1,0},
				{1,1,1,0,0,0,1,0},
				{1,3,0,4,2,0,1,0},
				{1,1,1,0,2,3,1,0},
				{1,3,1,1,2,0,1,0},
				{1,0,1,0,3,0,1,1},
				{1,2,0,2,2,2,3,1},
				{1,0,0,0,3,0,0,1},
				{1,1,1,1,1,1,1,1}
			},
			{		
				{0,0,1,1,1,1,1,0},
				{1,1,1,0,0,0,1,0},
				{1,3,0,0,4,2,1,0},
				{1,1,1,0,2,3,1,0},
				{1,3,1,1,2,0,1,0},
				{1,0,1,0,3,0,1,1},
				{1,2,0,2,2,2,3,1},
				{1,0,0,0,3,0,0,1},
				{1,1,1,1,1,1,1,1}
			}
		};

void MAP_INIT(int32_t chapter){
		int *elements_array = chapter_array[chapter][0];
		map.ELEMENTS = elements_array;
		map.Width = sizeof(chapter_array[chapter][0])/sizeof(chapter_array[chapter][0][0]);
		map.Height = sizeof(chapter_array[chapter])/sizeof(chapter_array[chapter][0]);
		
}

void GEN_MAP_PIC(int32_t chapter)
{
		MAP_INIT(chapter);
		int Height = map.Height;
		int Width = map.Width;
		int * p_element;
		p_element = map.ELEMENTS;
		int x_addr = 0;
		int y_addr = 0;;
	
		Paint_DrawString_EN(5, 0, "Sokoban chapter-", &Font16, WHITE, BLACK);
		Paint_DrawNum(181, 0, chapter, &Font16, WHITE, BLACK);
		
		for(int row = 0; row < Height; row++){
				for(int column = 0; column < Width; column++){
						x_addr = x_start + 20*column;
						y_addr = y_start + 20*row;
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

						p_element++;
				}
		}
		
		//Paint_DrawPic(5, 145, gImage_wall, 20, 20, WHITE, BLACK);
		//Paint_DrawNum(5, 125, Width, &Font16, WHITE, BLACK);
		//Paint_DrawNum(5, 145, Height, &Font16, WHITE, BLACK);
}




