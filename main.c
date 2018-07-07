/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include "debug.h"
#include "timer.h"
#include "motor.h"

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <util/delay.h>

#include <stdlib.h>
#include <stdio.h>

static uint8_t early_init(void) {
    // save reset reason
    uint8_t mcusr = MCUSR;
    // clear reset flags for next reset
    MCUSR = 0;

    cli();

    wdt_disable();

    // disable AIN1 and AIN0
    DIDR1 = 0x03;

    // disable digital inputs on ADC pins
    // except for ADC5 and ADC4 (needed for TWI)
    DIDR0 = 0xFF & ~((1 << ADC4D) | (1 << ADC5D));

    // disable analog comparator
    ACSR |= (1 << ACD);

    // set pins to output
    DDRB = (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5);
    DDRC = (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3);
    DDRD = (1 << DDD2) | (1 << DDD3) | (1 << DDD6) | (1 << DDD7);
    // drive pins low
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;

    // disable all peripheral clocks
    PRR = 0xFF;

    return mcusr;
}

static void measure_timer(void) {
    printf("measure timer\n");
    uint32_t t0 = get_time();
    _delay_ms(1000);
    uint32_t t1 = get_time();
    printf("%lu - %lu = %lu\n", t1, t0, t1 - t0);
    printf("expected: %lu\n", TIMER_MS(1000));
}


static void update_speed(int16_t d) {
    static int16_t spd = 0;
    spd += d;
    if (spd < 0) spd = 0;
    if (spd > 1023) spd = 1023;
    printf("speed: %d, dt: %d\n", spd, spd > 0 ? 0xFFFF / spd : 0xFFFF);
    motor_set_speed(30, (uint16_t)spd);
    motor_unset_target();
}

static void update_target(int16_t a) {
    static int16_t angle = 0;
    angle = (angle + a + MOTOR_FULL_ROTATION) % MOTOR_FULL_ROTATION;
    printf("angle: %d\n", angle);
    motor_set_target(angle);
}

static void update_move(int16_t a) {
    static int16_t angle = 0;
    angle = (angle + a + MOTOR_FULL_ROTATION) % MOTOR_FULL_ROTATION;
    printf("target: %d\n", angle);
    motor_move(30, 300, angle);
}

int main(void) {

    uint8_t mcusr = early_init();

    debug_init();

    if ((mcusr & (EXTRF | WDRF)) != 0) {
        debug_dump_trace();
    }
    debug_init_trace();

    timer_start();

    motor_init();

    sei();
    printf("\nboot: %#x\n", mcusr);

    for (;;) {
        // motor_debug();
        motor_dump_pos_sens();
        // debug_dump_motor();
        if (debug_char_pending()) {
            CHECKPOINT;
            char c = debug_getchar();
            switch (c) {
            case 't': measure_timer(); break;
            case '+': update_speed(1); break;
            case '-': update_speed(-1); break;
            case 'l': update_move(100); break;
            case 'k': update_move(-100); break;
            case 'm': update_target(100); break;
            case 'n': update_target(-100); break;
            case 'o': motor_set_pos_sensor(false); break;
            case 'p': motor_set_pos_sensor(true); break;
            case '.': motor_start(); printf("motor started\n"); break;
            case ',': motor_stop(); printf("motor stopped\n"); break;
            case 'c': printf("count: %d, skip: %d, spd: %u, feed: %u, time: %u\n", motor_get_count(), motor_get_skip(), motor_get_speed(), motor_get_feed(), motor_get_time()); break;
            // case 'd': motor_dump_calc(); break;
            case '?': printf("PRR: %#x, GTCCR: %#x, TCCR1A: %#x TCCR1B: %#x, TCNT1: %u, TCCR2A: %#x TCCR2B: %#x, TCNT2: %u\n", PRR, GTCCR, TCCR1A, TCCR1B, TCNT1, TCCR2A, TCCR2B, TCNT2); break;
            }
        }
    }

    return 0; /* never reached */
}
