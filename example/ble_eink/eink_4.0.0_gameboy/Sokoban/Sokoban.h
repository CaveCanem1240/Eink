/***************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h> //memset()
#include <math.h>

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t


typedef struct {
    int *ELEMENTS;
    int Width;
    int Height;
} MAP;
extern MAP Map;




