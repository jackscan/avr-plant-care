#pragma once

#include "timer.h"

#include <stdint.h>

#define MAX_WATER_TIME  80 // 20s
#define WATER_TIME_REFILL_INTERVAL TIMER_MIN(30)

typedef struct {
    uint32_t balance;
    uint32_t last;
    uint32_t fract;
} account_t;
extern account_t s_account;

void water_init(void);
uint8_t water_limit(uint8_t t);
void water_start(void);
uint8_t water_stop(void);
