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

uint16_t flt_median(uint16_t data)
{
	uint16_t b[FLT_SIZE];

	buffer[idx] = data;
	if(++idx > FLT_SIZE-1)
	{
		idx = 0;
		filled = 1;
	}

	if(!filled)
		return MEDIAN_NON_VALID;

	memcpy(b, buffer, sizeof(buffer));
	qsort(b, FLT_SIZE, sizeof(b[0]), cmp);
	return b[FLT_SIZE/2];
}

void init_median(void)
{
	memset(buffer, 0, sizeof(buffer));
	filled = 0;
	idx = 0;
}