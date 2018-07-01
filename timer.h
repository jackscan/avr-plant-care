#pragma once

#include <stdint.h>

#define TIMER_PS 1024UL
#define TIMER_MIN(M) ((uint32_t)M * (60UL * F_CPU / TIMER_PS))
#define TIMER_MS(MS) ((uint32_t)MS * (F_CPU / 1000UL) / TIMER_PS)

void timer_start(void);
uint32_t get_time(void);
