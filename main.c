/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include "debug.h"
#include "timer.h"
#include "motor.h"
#include "hx711.h"
#include "water.h"

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <util/delay.h>

#include <stdlib.h>
#include <stdio.h>

#define MOTOR_MIN_FEED 30
#define MOTOR_MAX_SPEED 300

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

    hx711_early_init();

    // disable all peripheral clocks
    PRR = 0xFF;

    return mcusr;
}

static void dump_motor_status(void) {
    int16_t pos, adj;
    motor_get_pos_and_adjust(&pos, &adj);
    printf("pos: %d (%d), skip: %d, spd: %u, feed: %u, time: %u, rev: "
           "%d, cal: %i\n",
           pos, adj, motor_get_skip(), motor_get_spd(), motor_get_feed(),
           motor_get_time(), motor_get_remaining_revolutions(),
           motor_pos_is_calibrated());
}

uint8_t water(uint8_t startuptime, uint8_t watertime) {
    // quaters of second the watering pump is enabled
    uint8_t qt = water_limit(startuptime + watertime);

    printf("watering %ums\n", (uint16_t)qt * 250u);

    uint8_t wt = qt > startuptime ? qt - startuptime : 1;
    uint16_t tspd = motor_calculate_speed(CPR, wt);
    printf("target spd: %u\n", tspd);

    // TODO: use last set target position instead of current
    int16_t mpos, madj;
    motor_get_pos_and_adjust(&mpos, &madj);

    uint32_t tlast = get_time();

    uint32_t interval = TIMER_MS(250);

    // start motor
    // assumptions: <startuptime> + <time for motor to reach its speed> is less
    // than 2x watertime
    motor_move(MOTOR_MIN_FEED, tspd, mpos, 2);

    wdt_enable(WDTO_500MS);
    motor_start();

    // wait for motor to reach about 80% of target speed
    uint16_t spdthreshold = tspd * 4 / 5;
    // wait at most 5s
    for (uint8_t i = 0; i < 4 * 5;) {
        if (motor_get_spd() > spdthreshold)
            break;
        uint32_t t = get_time();
        if (t - tlast > interval) {
            ++i;
            tlast += interval;
            wdt_reset();
        }
        motor_update_feed();
    }

    wdt_reset();
    water_start();

    for (uint8_t i = 0; i < qt;) {
        uint32_t t = get_time();
        if (t - tlast > interval) {
            ++i;
            tlast += interval;
            wdt_reset();
        }
        motor_update_feed();
    }
    uint8_t dt = water_stop();

    // motor is hopefully calibrated now, update target position
    mpos += motor_get_adjust() - madj;
    motor_move(MOTOR_MIN_FEED, MOTOR_MAX_SPEED, mpos, 0);
    wdt_reset();

    // calculate some max time for motor to reach its target
    const uint16_t max_wait_time =
        2 * (uint16_t)motor_get_time_per_revolution(MOTOR_MAX_SPEED);


    // wait for motor to reach its target position
    for (uint16_t i = 0; i < max_wait_time;) {
        if (motor_is_stopped()) break;
        uint32_t t = get_time();
        if (t - tlast > interval) {
            ++i;
            tlast += interval;
            wdt_reset();
        }
        motor_update_feed();
    }

    motor_stop();
    wdt_disable();

    printf("watered %lums\n", (uint32_t)dt * 250);

    return dt;
}

static void measure_weight_2(void) {
    printf("w:\n");
    for (uint8_t i = 0; i < 10; ++i) {
        uint16_t w = hx711_read();
        printf("%u\n", w);
    }
    hx711_powerdown();
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
    angle = (angle + a + CPR) % CPR;
    printf("angle: %d\n", angle);
    motor_set_target(angle);
}

static void update_move(int16_t a) {
    static int16_t angle = 0;
    angle = (angle + a + CPR) % CPR;
    printf("target: %d\n", angle);
    motor_move(30, 300, angle, 1);
}

int main(void) {

    uint8_t mcusr = early_init();

    debug_init();

    if ((mcusr & (EXTRF | WDRF)) != 0) {
        debug_dump_trace();
    }
    debug_init_trace();

    timer_start();
    water_init();
    motor_init();
    hx711_init();

    sei();
    printf("\nboot: %#x\n", mcusr);

    char linebuf[64];
    uint8_t linelen = 0;
    enum  {
        KEY_INPUT = 0,
        CALIBRATION_INPUT,
        QTIME_INPUT,
        WTIME_INPUT,
    } input_mode = KEY_INPUT;

    for (;;) {
        // motor_debug();
        motor_dump_pos_sens();
        motor_update_feed();
        // debug_dump_motor();
        if (debug_char_pending()) {
            CHECKPOINT;
            char c = debug_getchar();
            if (input_mode) {
                if (c != '\n' && c != '\r') {
                    linebuf[linelen++] = c;
                    printf("%c", c);
                } else {
                    printf("\n");
                    linebuf[linelen] = '\0';

                    switch (input_mode) {
                    case CALIBRATION_INPUT: {
                        uint32_t offset, scale;
                        if (sscanf(linebuf, "%lu %lu", &offset, &scale) == 2) {
                            hx711_calib(offset, scale);
                        } else {
                            printf("failed to read offset and scale: '%s'\n",
                                   linebuf);
                        }
                        break;
                    }
                    case QTIME_INPUT: {
                        uint16_t t;
                        if (sscanf(linebuf, "%u", &t) == 1) {
                            printf("target speed: %u\n", motor_calculate_speed(CPR, (uint8_t)(t * 4)));
                        }
                        break;
                    }
                    case WTIME_INPUT: {
                        uint16_t s, t;
                        if (sscanf(linebuf, "%u %u", &s, &t) == 2) {
                            if (s > 255) s = 255;
                            if (t > 255) t = 255;
                            water((uint8_t)s, (uint8_t)t);
                        }
                        break;
                    }
                    default:
                        break;
                    }
                    linelen = 0;
                    input_mode = KEY_INPUT;
                }
            } else {
                switch (c) {
                case 't': measure_timer(); break;
                case '1': input_mode = CALIBRATION_INPUT; printf("enter offset and scale\n> "); break;
                case 's': printf("writing calibration data\n"); hx711_write_calib(); break;
                case 'q': input_mode = QTIME_INPUT; printf("enter seconds\n> "); break;
                case '2': input_mode = WTIME_INPUT; printf("enter startup and watering time\n>"); break;
                case 'w': measure_weight_2(); break;
                case '+': update_speed(1); break;
                case '-': update_speed(-1); break;
                case 'l': update_move(100); break;
                case 'k': update_move(-100); break;
                case 'm': update_target(100); break;
                case 'n': update_target(-100); break;
                case 'u': motor_unset_calibration(); break;
                case 'o': motor_disable_pos_sensor(); break;
                case 'p': motor_enable_pos_sensor(); break;
                case '.': motor_start(); printf("motor started\n"); break;
                case ',': motor_stop(); printf("motor stopped\n"); break;
                case 'c': dump_motor_status(); break;
                case '?': printf("PRR: %#x, GTCCR: %#x, TCCR1A: %#x TCCR1B: %#x, TCNT1: %u, TCCR2A: %#x TCCR2B: %#x, TCNT2: %u\n", PRR, GTCCR, TCCR1A, TCCR1B, TCNT1, TCCR2A, TCCR2B, TCNT2); break;
                }
            }
        }
    }

    return 0; /* never reached */
}
