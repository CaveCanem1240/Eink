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
} MAP;
extern MAP Map;
extern uint8_t next_map[9][9];

uint8_t GEN_NEXT_PIC(uint8_t operation, bool init_next_chapter, uint32_t chapter);
void GEN_MAP_PIC(uint32_t chapter);

