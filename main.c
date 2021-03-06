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
#include "twi-slave.h"

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
    DDRB = ~0;
    // except SDA, SCL, Reset
    DDRC = ~((1 << DDC4) | (1 << DDC5) | (1 << DDC6));
    DDRD = ~0;
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

static void water(uint8_t startuptime, uint8_t watertime) {
    // quaters of second the watering pump is enabled
    uint8_t qt = water_limit(startuptime + watertime);

    printf("watering %ums (%u, %u)\n", (uint16_t)qt * 250u,
           startuptime, watertime);

    uint8_t wt = qt > startuptime ? qt - startuptime : 1;
    uint16_t tspd = motor_calculate_speed(CPR, wt);
    printf("target spd: %u\n", tspd);

    int16_t mpos = twi_get_target_angle();
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
    // wait at most 2s
    for (uint8_t i = 0; !twi_get_stop_flag() && i < 2 * 4;) {
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

    for (uint8_t i = 0; !twi_get_stop_flag() && i < qt;) {
        uint32_t t = get_time();
        if (t - tlast > interval) {
            ++i;
            tlast += interval;
            wdt_reset();
        }
        motor_update_feed();
    }
    uint8_t dt = water_stop();

    twi_set_last_watering(dt);
    // update speed
    motor_move(MOTOR_MIN_FEED, MOTOR_MAX_SPEED, mpos, 0);
    wdt_reset();

    // calculate some max time for motor to reach its target
    const uint16_t max_wait_time =
        2 * (uint16_t)motor_get_time_per_revolution(MOTOR_MAX_SPEED);


    // wait for motor to reach its target position
    for (uint16_t i = 0; !twi_get_stop_flag() && i < max_wait_time;) {
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
}

static void measure_weight(void) {
    uint32_t t0 = get_time();

    while (true) {
        uint16_t w = hx711_read();
        if (twi_get_cmd() != CMD_GET_WEIGHT ||
            get_time() - t0 >= TIMER_MS(4000)) {
            break;
        }
        twi_add_weight(w);
    }

    uint32_t t1 = get_time();
    uint32_t w = twi_get_weight();

    hx711_powerdown();

    printf("weight measuring time: %lu\n", t1 - t0);

    uint32_t n = (w >> 16);
    uint32_t d = (w & 0xFFFF);
    printf("weight: %u / %u = %u\n",
           (uint16_t)n, (uint16_t)d, (uint16_t)((n + d / 2) / d));
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
    twi_slave_init(0x10);

    sei();
    printf("\nboot: %#x\n", mcusr);

    char linebuf[64];
    uint8_t linelen = 0;
    enum  {
        KEY_INPUT = 0,
        CALIBRATION_INPUT,
        QTIME_INPUT,
        WTIME_INPUT,
        REFILL_INPUT,
    } input_mode = KEY_INPUT;

    for (;;) {
        // motor_debug();
        motor_dump_pos_sens();
        motor_update_feed();
        // debug_dump_motor();

        twi_dump_trace();
        if (twi_cmd_pending()) {
            CHECKPOINT;
            uint8_t cmd = twi_next_cmd();
            printf("command: %#x\n", cmd);
            switch (cmd) {
            case CMD_GET_WEIGHT:
                measure_weight();
                break;
            case CMD_WATERING:
                water(twi_get_watering_startup(), twi_get_watering());
                break;
            case CMD_ROTATE: {
                    int16_t mpos = twi_get_target_angle();
                    printf("rotate to %d\n", mpos);
                    motor_move(MOTOR_MIN_FEED, MOTOR_MAX_SPEED, mpos, 0);
                    motor_start();
                }
                break;
            case CMD_STOP:
                motor_stop();
                twi_unset_stop_flag();
                printf("stopped\n");
                break;
            }
        }


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
                    case REFILL_INPUT: {
                        uint16_t t;
                        if (sscanf(linebuf, "%u", &t) == 1) {
                            uint8_t r = (uint8_t)t;
                            printf("setting refill interval: %u (%lu)\n", r, TIMER_MIN(r));
                            water_set_refill(r);
                        }
                        printf("refill interval: %um (%lu)\n", water_get_refill(), s_account.refill_interval);
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
                case 'a': printf("water account: %ums\n", water_limit(0xFF) * 250); break;
                case 'w': measure_weight_2(); break;
                case 'r': input_mode = REFILL_INPUT; break;
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
