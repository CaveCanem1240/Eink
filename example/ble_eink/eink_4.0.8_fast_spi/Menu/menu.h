/***************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h> //memset()
#include <math.h>
#include <stdbool.h>

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t



typedef struct {
    uint8_t *ELEMENTS;
    uint8_t Width;
    uint8_t Height;
} MENU;
extern MENU menu;
extern uint8_t next_menu[4];

uint8_t GEN_NEXT_MENU_PIC(uint8_t operation, bool init_next_menu, uint32_t page);


