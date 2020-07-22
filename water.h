#pragma once

#include "timer.h"

#include <stdint.h>

#define MAX_WATER_TIME  80 // 20s
#define MIN_WATER_TIME_REFILL_INTERVAL TIMER_MIN(15)

typedef struct {
    uint32_t balance;
    uint32_t last;
    uint32_t fract;
    uint32_t refill_interval;
} account_t;
extern account_t s_account;

void water_init(void);
uint8_t water_limit(uint8_t t);
uint8_t water_get_limit(void);
void water_set_refill(uint8_t m);
uint8_t water_get_refill(void);
void water_start(void);
uint8_t water_stop(void);
