#include "water.h"

#include <avr/interrupt.h>
#include <avr/wdt.h>

#define WATERING_DDR DDRD
#define WATERING_PORT PORTD
#define WATERING_BIT (1 << PD4)

account_t s_account = {};

uint8_t water_limit(uint8_t t) {
    uint32_t curr = get_time();

    uint8_t sreg = SREG;
    cli();
    uint32_t dt = (curr - s_account.last + s_account.fract);
    uint32_t da = dt / s_account.refill_interval;
    uint32_t na = da + s_account.balance;

    s_account.balance = na < MAX_WATER_TIME ? na : MAX_WATER_TIME;
    s_account.fract = dt - da * s_account.refill_interval;
    s_account.last = curr;
    SREG = sreg;

    if (t > s_account.balance)
        t = s_account.balance;

    return t;
}

uint8_t water_get_limit(void) {
    uint32_t curr = get_time();

    uint8_t sreg = SREG;
    cli();
    uint32_t dt = (curr - s_account.last + s_account.fract);
    uint32_t da = dt / s_account.refill_interval;
    uint32_t na = da + s_account.balance;
    SREG = sreg;

    uint8_t limit = na < MAX_WATER_TIME ? na : MAX_WATER_TIME;
    return limit;
}

void water_set_refill(uint8_t m) {
    uint32_t t = TIMER_MIN(m);
    if (t < MIN_WATER_TIME_REFILL_INTERVAL)
        t = MIN_WATER_TIME_REFILL_INTERVAL;
    uint8_t sreg = SREG;
    cli();
    s_account.refill_interval = t;
    SREG = sreg;
}

uint8_t water_get_refill(void) {
    uint8_t sreg = SREG;
    cli();
    uint32_t t = s_account.refill_interval;
    SREG = sreg;
    uint32_t m = t / (F_CPU * 60UL / 1024);
    return (uint8_t)m;
}

static void update_water_account(uint8_t t) {
    uint8_t sreg = SREG;
    cli();
    s_account.balance =
        (t < s_account.balance) ? s_account.balance - t : 0;
    SREG = sreg;
}

static uint32_t s_start_time;

void water_init(void) {
    WATERING_PORT &= ~WATERING_BIT;
    WATERING_DDR |= WATERING_BIT;
    s_account.refill_interval = MIN_WATER_TIME_REFILL_INTERVAL;
}

void water_start(void) {
    s_start_time = get_time();
    WATERING_DDR &= ~WATERING_BIT;
    WATERING_PORT |= WATERING_BIT;
}

uint8_t water_stop(void) {
    uint32_t t1 = get_time();
    WATERING_PORT &= ~WATERING_BIT;
    WATERING_DDR |= WATERING_BIT;

    uint32_t f = (F_CPU / 4UL);
    uint32_t t0 = s_start_time;
    uint32_t dt = ((t1 - t0) * TIMER_PS + f/2) / f;

    update_water_account(dt);

    return (uint8_t)dt;
}
