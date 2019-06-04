#include "timer.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <stdbool.h>

static __attribute__ ((section (".noinit"))) uint32_t s_count;

void timer_start(void) {
    PRR &= ~(1 << PRTIM2);
    // timer overflow interrupt enabled
    TIMSK2 = (1 << TOIE2);
    // 1/1024
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
    TCNT2 = 0;
}

ISR (TIMER2_OVF_vect) {
    ++s_count;
}

uint32_t get_time(void) {
    uint8_t sreg = SREG;
    cli();
    uint32_t count = s_count;
    uint8_t c = TCNT2;
    bool overflow = (TIFR2 & (1 << TOV2)) != 0;
    SREG = sreg;

    // overflow pending?
    if (overflow && c < 128)
        ++count;

    return count << 8 | (uint32_t)c;
}
