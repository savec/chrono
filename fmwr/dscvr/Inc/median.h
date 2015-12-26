#ifndef MEDIAN_H
#define MEDIAN_H

typedef struct {
	struct {
		uint16_t *buf;
		size_t pos;
		size_t len;
	} dline;
	uint16_t *sorted;
	uint32_t filled;
} u16_median_t;

u16_median_t * u16_median_init(size_t len); // must be odd
void u16_median_free(u16_median_t *entity);
int u16_median_flt(u16_median_t * entity, const uint16_t in, uint16_t *out);

#endif