#include <stdint.h>
#include <stdlib.h>
#include <string.h> //memset()
#include <stdbool.h>
#include <math.h>
#include "GUI_Paint.h"
#include "Menu.h"

MENU menu;

uint8_t next_menu[4];
uint32_t init_page = 0;

/*0:none, 1:wall, 2:box, 3:point, 4:player, 5:box-over-point, 6:player-over-point*/
extern uint8_t menu_array[3][4] = {	
				{2,3,5,7},
				{0,0,0,0},
				{0,0,0,0}
		};

void MENU_INIT(uint32_t page){
		uint8_t *elements_array = menu_array[0];
		menu.ELEMENTS = elements_array;
		menu.Height = sizeof(menu_array)/sizeof(menu_array[page]);
}


void GEN_MENU_PIC(uint32_t page)
{
		MENU_INIT(page);
		uint8_t Height = 4;
		uint8_t * p_element;
		p_element = menu.ELEMENTS;
		uint8_t x_addr = 0;
		uint8_t y_addr = 0;
		uint8_t x_start = 15;
		uint8_t y_start = 80;

	
		
		Paint_DrawString_EN(5, 10, "Vipsn-Eink", &Font16, WHITE, BLACK);
		Paint_DrawString_EN(5, 30, "Battery-Free", &Font16, WHITE, BLACK);
		Paint_DrawString_EN(5, 50, "GameBoy", &Font16, WHITE, BLACK);
		//Paint_DrawNum(181, 0, chapter, &Font16, WHITE, BLACK);
		
		for(uint8_t row = 0; row < Height; row++){
				y_addr = y_start + 20*row;
				if(*p_element==0){
					Paint_DrawString_EN(50, y_addr, "Null", &Font16, WHITE, BLACK);
				}
				if(*p_element==1){
					Paint_DrawString_EN(50, y_addr, "Sokoban1", &Font16, WHITE, BLACK);
				}
				if(*p_element==2){
					Paint_DrawString_EN(50, y_addr, "Sokoban1", &Font16, BLACK, WHITE);
				}
				if(*p_element==3){
					Paint_DrawString_EN(50, y_addr, "Sokoban2", &Font16, WHITE, BLACK);
				}
				if(*p_element==4){
					Paint_DrawString_EN(50, y_addr, "Sokoban2", &Font16, BLACK, WHITE);
				}
				if(*p_element==5){
					Paint_DrawString_EN(50, y_addr, "Sokoban3", &Font16, WHITE, BLACK);
				}
				if(*p_element==6){
					Paint_DrawString_EN(50, y_addr, "Sokoban3", &Font16, BLACK, WHITE);
				}
				if(*p_element==7){
					Paint_DrawString_EN(50, y_addr, "Sokoban4", &Font16, WHITE, BLACK);
				}
				if(*p_element==8){
					Paint_DrawString_EN(50, y_addr, "Sokoban4", &Font16, BLACK, BLACK);
				}

				p_element++;

		}
}

void UPDATE_MENU_PIC(uint32_t page)
{
		MENU_INIT(page);
		uint8_t Height = 4;
		uint8_t x_addr = 0;
		uint8_t y_addr = 0;;
		uint8_t x_start = 15;
		uint8_t y_start = 80;
	
		Paint_DrawString_EN(5, 10, "Vipsn-Eink", &Font16, WHITE, BLACK);
		Paint_DrawString_EN(5, 30, "Battery-Free", &Font16, WHITE, BLACK);
		Paint_DrawString_EN(5, 50, "GameBoy", &Font16, WHITE, BLACK);
	
		for(uint8_t row = 0; row < Height; row++){
				y_addr = y_start + 20*row;
				if(next_menu[row]==0){
					Paint_DrawString_EN(50, y_addr, "Null", &Font16, WHITE, BLACK);
				}
				if(next_menu[row]==1){
					Paint_DrawString_EN(50, y_addr, "Sokoban1", &Font16, WHITE, BLACK);
				}
				if(next_menu[row]==2){
					Paint_DrawString_EN(50, y_addr, "Sokoban1", &Font16, BLACK, WHITE);
				}
				if(next_menu[row]==3){
					Paint_DrawString_EN(50, y_addr, "Sokoban2", &Font16, WHITE, BLACK);
				}
				if(next_menu[row]==4){
					Paint_DrawString_EN(50, y_addr, "Sokoban2", &Font16, BLACK, WHITE);
				}
				if(next_menu[row]==5){
					Paint_DrawString_EN(50, y_addr, "Sokoban3", &Font16, WHITE, BLACK);
				}
				if(next_menu[row]==6){
					Paint_DrawString_EN(50, y_addr, "Sokoban3", &Font16, BLACK, WHITE);
				}
				if(next_menu[row]==7){
					Paint_DrawString_EN(50, y_addr, "Sokoban4", &Font16, WHITE, BLACK);
				}
				if(next_menu[row]==8){
					Paint_DrawString_EN(50, y_addr, "Sokoban4", &Font16, BLACK, WHITE);
				}
		}
}
void menu_move_up(void)
{
		uint8_t Height = 4;
		uint8_t last_menu[Height];
		uint8_t player_x,player_y;
		for(uint8_t i=0;i<Height;i++){
				last_menu[i] = next_menu[i];
				if(last_menu[i]%2 != 1){
						player_y = i;
				}
		}
		
		/*0:none, 1:wall, 2:box, 3:point, 4:player, 5:box-over-point, 6:player-over-point*/
		next_menu[player_y] = last_menu[player_y]-1;
		if(player_y == 0){
				next_menu[3] = last_menu[3]+1;
		}
		else{
				next_menu[player_y-1] = last_menu[player_y-1]+1;
		}
}
void menu_move_down(void)
{
		uint8_t Height = 4;
		uint8_t last_menu[Height];
		uint8_t player_x,player_y;
		for(uint8_t i=0;i<Height;i++){
				last_menu[i] = next_menu[i];
				if(last_menu[i]%2 != 1){
						player_y = i;
				}
		}
		
		/*0:none, 1:wall, 2:box, 3:point, 4:player, 5:box-over-point, 6:player-over-point*/
		next_menu[player_y] = last_menu[player_y]-1;
		if(player_y == 3){
				next_menu[0] = last_menu[0]+1;
		}
		else{
				next_menu[player_y+1] = last_menu[player_y+1]+1;
		}
}



/*operation:  0-null 1-up 2-down 3-left 4-right 5-enter 6-cancel*/

uint8_t GEN_NEXT_MENU_PIC(uint8_t operation, bool init_next_menu, uint32_t page)
{
		
		uint8_t Height = 4;
		if(init_next_menu){
				
				for(uint8_t i=0;i<Height;i++){
						next_menu[i] = menu_array[page][i];
				}
				GEN_MENU_PIC(page);
				return 1;
		}
		else{
				if(operation==1){
						menu_move_up();
				}
				if(operation==2){
						menu_move_down();
				}
				UPDATE_MENU_PIC(page);
		}
}




