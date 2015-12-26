#include "stm32l1xx_hal.h"
#include <FreeRTOS.h>
#include <task.h>
#include <median.h>
#include <stdlib.h>
#include <string.h>

int cmp(const void *a, const void *b) 
{
	uint16_t a1 = *(uint16_t*)a;
	uint16_t b1 = *(uint16_t*)b;
	return (a1>b1)?(1):((a1<b1)?-1:0); 
}

u16_median_t * u16_median_init(size_t len)
{
	configASSERT(len);
	u16_median_t *entity = (u16_median_t *)pvPortMalloc(sizeof(u16_median_t));
	configASSERT(entity);
	memset(entity, 0, sizeof(*entity));
	entity->dline.buf = (uint16_t *)pvPortMalloc(sizeof(uint16_t)*len);
	configASSERT(entity->dline.buf);
	memset(entity->dline.buf, 0, sizeof(uint16_t)*len);
	entity->dline.len = len;
	entity->sorted = (uint16_t *)pvPortMalloc(sizeof(uint16_t)*len);
	configASSERT(entity->sorted);
	return entity;	
}

void u16_median_free(u16_median_t *entity)
{
	if(entity == NULL)
		return;
	if(entity->dline.buf)
		vPortFree(entity->dline.buf);
	if(entity->sorted)
		vPortFree(entity->sorted);
	vPortFree(entity);
}

int u16_median_flt(u16_median_t * entity, const uint16_t in, uint16_t *out)
{
	entity->dline.buf[entity->dline.pos] = in;
	if(++(entity->dline.pos) > entity->dline.len-1)
	{
		entity->dline.pos = 0;
		entity->filled = 1;	
	}

	memcpy(entity->sorted, entity->dline.buf, entity->dline.len*sizeof(uint16_t));
	qsort(entity->sorted, entity->dline.len, sizeof(entity->sorted[0]), cmp);
	*out = entity->sorted[entity->dline.len/2];

	return entity->filled?1:0;
}
