#include "water.h"

#include <avr/interrupt.h>
#include <avr/wdt.h>

#define WATERING_DDR DDRB
#define WATERING_PORT PORTB
#define WATERING_BIT (1 << PB4)

static __attribute__ ((section (".noinit"))) struct {
    uint32_t balance;
    uint32_t last;
    uint32_t fract;
} s_account;

uint8_t water_limit(uint8_t t) {
    uint32_t curr = get_time();

    uint8_t sreg = SREG;
    cli();
    uint32_t dt = (curr - s_account.last + s_account.fract);
    uint32_t da = dt / WATER_TIME_REFILL_INTERVAL;
    uint32_t na = da + s_account.balance;

    s_account.balance = na < MAX_WATER_TIME ? na : MAX_WATER_TIME;
    s_account.fract = dt - da * WATER_TIME_REFILL_INTERVAL;
    s_account.last = curr;
    SREG = sreg;

    if (t > s_account.balance)
        t = s_account.balance;

    return t;
}

static void update_water_account(uint8_t t) {
    s_account.balance =
        (t < s_account.balance) ? s_account.balance - t : 0;
}

static uint32_t s_start_time;

void water_init(void) {
    WATERING_PORT &= ~WATERING_BIT;
    WATERING_DDR &= ~WATERING_BIT;
}

void water_start(void) {
    s_start_time = get_time();
    WATERING_PORT |= WATERING_BIT;
}

uint8_t water_stop(void) {
    uint32_t t1 = get_time();
    WATERING_PORT &= ~WATERING_BIT;
    wdt_disable();

    uint32_t f = (F_CPU / 4UL);
    uint32_t t0 = s_start_time;
    uint32_t dt = ((t1 - t0) * TIMER_PS + f/2) / f;

    update_water_account(dt);

    return (uint8_t)dt;
}
