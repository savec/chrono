#include "stm32l1xx_hal.h"
#include <median.h>
#include <stdlib.h>
#include <string.h>

static uint16_t buffer[FLT_SIZE];
static int filled = 0;
static size_t idx = 0;

int cmp(const void *a, const void *b) 
{
	uint16_t a1 = *(uint16_t*)a;
	uint16_t b1 = *(uint16_t*)b;
	return (a1>b1)?(1):((a1<b1)?-1:0); 
}

int flt_median(const uint16_t in, uint16_t *out)
{
	uint16_t b[FLT_SIZE];

	buffer[idx] = in;
	if(++idx > FLT_SIZE-1)
	{
		idx = 0;
		filled = 1;
	}
	memcpy(b, buffer, sizeof(buffer));
	qsort(b, FLT_SIZE, sizeof(b[0]), cmp);
	*out = b[FLT_SIZE/2];

	return filled?0:-1;
}

void init_median(void)
{
	memset(buffer, 0, sizeof(buffer));
	filled = 0;
	idx = 0;
}