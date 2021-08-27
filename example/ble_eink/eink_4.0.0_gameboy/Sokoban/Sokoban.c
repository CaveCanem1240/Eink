#include <stdint.h>
#include <stdlib.h>
#include <string.h> //memset()
#include <math.h>
#include "GUI_Paint.h"
#include "Sokoban.h"

MAP map;


void MAP_INIT(void){
		int Height = 9;
		int Width = 8;
		map.Height = Height;
		map.Width = Width;
		/*0:none, 1:wall, 2:box, 3:point, 4:player, 5:box-over-point*/
		int elements_array[9][8] = {		
			{0,0,1,1,1,1,1,0},
			{1,1,1,0,0,0,1,0},
			{1,3,4,2,0,0,1,0},
			{1,1,1,0,2,3,1,0},
			{1,3,1,1,2,0,1,0},
			{1,0,1,0,3,0,1,1},
			{1,2,0,5,2,2,3,1},
			{1,0,0,0,3,0,0,1},
			{1,1,1,1,1,1,1,1}
		};
		map.ELEMENTS = elements_array[0];
}

void GEN_PIC(void)
{
		
}




