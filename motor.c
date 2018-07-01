#include "motor.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#define MA_DDR DDRB
#define MA_PORT PORTB
#define MA_PHASE_BIT (1 << PB0)
#define MA_ENABLE_BIT (1 << PB1)
#define MA_SENS_DDR DDRC
#define MA_SENS_PORT PORTC
#define MA_SENS_PIN PINC
#define MA_SENSA_BIT (1 << PC0)
#define MA_SENSB_BIT (1 << PC1)
#define MA_SENS_PCMSK PCMSK1
#define MA_SENS_PCIE PCIE1
#define MA_SENS_PCINT_VECT PCINT1_vect
#define MA_SENSA_PCINT PCINT8
#define MA_SENSB_PCINT PCINT9

// 1/1024
// #define CLOCK_SELECT ((1 << CS12) | (1 << CS10))
// 1/256
#define CLOCK_SELECT (1 << CS12)
// 1/64
// #define CLOCK_SELECT ((1 << CS11) | (1 << CS10))

#define FULL_ROT_COUNT (120UL*12UL*10UL)
#define TIMER_INTERVAL(MS) ((F_CPU * (unsigned long)(MS)) / (256UL * 1000UL))
#define MAX_INTERVAL TIMER_INTERVAL(100UL)

#define LOCKI() uint8_t sreg = SREG; cli()
#define RELOCKI() sreg = SREG; cli()
#define UNLOCKI() SREG = sreg

static struct
{
    int16_t count;
    int16_t target;
    uint16_t skip;
    uint8_t sens;
    uint8_t last_dir;
    uint16_t time;

    uint16_t cur_interval;
    uint16_t target_interval;
    uint16_t timer_ovf;
} s_motor_a;

static struct {
    uint16_t ocr;
    uint16_t dt;
    uint16_t interval;
    uint16_t time;
    uint8_t dir;
    bool change;
} dbgm, dbgm_tmp;

static struct {
    uint16_t ocr0, ocr1;
    uint16_t feed;
    uint16_t dt;
    uint16_t dt2;
    uint16_t interval;
    uint16_t time;
    bool change;
} dbgt, dbgt_tmp;

void debug_dump_timer(void) {
    if (dbgt.change) {
        LOCKI();
        memcpy(&dbgt_tmp, &dbgt, sizeof(dbgt));
        dbgt.change = false;
        UNLOCKI();
        printf("t: %u %u %u %u %u %u %u %u\n",
               dbgt_tmp.ocr0, dbgt_tmp.ocr1, dbgt_tmp.feed, dbgt_tmp.dt,
               dbgt_tmp.dt2, dbgt_tmp.interval, s_motor_a.target_interval,
               dbgt_tmp.time);
    }
}

void debug_dump_motor(void) {
    if (dbgm.change) {
        LOCKI();
        memcpy(&dbgm_tmp, &dbgm, sizeof(dbgm));
        dbgm.change = false;
        UNLOCKI();
        printf("m: %u %u %u %d %u\n",
               dbgm_tmp.ocr, dbgm_tmp.dt, dbgm_tmp.interval,
               dbgm_tmp.dir, dbgm_tmp.time);
    }
}

static uint16_t get_time(void) {
    uint16_t ovf = s_motor_a.timer_ovf;
    uint16_t tcnt = TCNT1;
    // overflow pending?
    if ((TIFR1 & (1 << TOV1)) != 0 && tcnt < 128)
        ++ovf;

    return (ovf << 8) | tcnt;
}

ISR (MA_SENS_PCINT_VECT) {
    uint16_t t = get_time();
    uint16_t dt = t - s_motor_a.time;
    s_motor_a.time = t;
    dbgm.ocr = OCR1A;
    dbgm.dt = dt;
    dbgm.time = t;
    dbgm.change = true;

    const uint8_t mask = MA_SENSA_BIT | MA_SENSB_BIT;
    uint8_t sens = MA_SENS_PIN & mask;
    int8_t dir = 0;

    switch (s_motor_a.sens ^ sens) {
        case MA_SENSA_BIT:
            dir = (((sens & MA_SENSA_BIT) ? 1 : 0) ^ ((sens & MA_SENSB_BIT) ? 1 : 0)) * 2 - 1;
            break;
        case MA_SENSB_BIT:
            dir = (((sens & MA_SENSA_BIT) ? 0 : 1) ^ ((sens & MA_SENSB_BIT) ? 1 : 0)) * 2 - 1;
            break;
        case 0:
            return;
        default:
            ++s_motor_a.skip;
            dir = s_motor_a.last_dir;
            break;
    }

    if (s_motor_a.last_dir == dir && dir != 0 && dt <= MAX_INTERVAL)
    {
        s_motor_a.cur_interval = dt;
    }
    else
    {
        s_motor_a.cur_interval = MAX_INTERVAL;
    }
    dbgm.interval = s_motor_a.cur_interval;
    dbgm.dir = dir;
    s_motor_a.last_dir = dir;
    s_motor_a.count += dir;

    if (s_motor_a.count >= FULL_ROT_COUNT)
        s_motor_a.count -= FULL_ROT_COUNT;
    else if (s_motor_a.count < 0)
        s_motor_a.count += FULL_ROT_COUNT;

    s_motor_a.sens = sens;

    if (s_motor_a.count == s_motor_a.target)
        motor_stop();
}

ISR (TIMER1_OVF_vect) {
    // count for 50ms
    // const uint16_t interval = TIMER_INTERVAL(50UL);
    // if (++s_motor_a.ovf_count > interval)

    ++s_motor_a.timer_ovf;
    uint16_t t = (s_motor_a.timer_ovf << 8) | TCNT1;
    uint16_t dt = t - s_motor_a.time;

    dbgt.dt = dt;
    dbgt.time = s_motor_a.time;

    if (dt > MAX_INTERVAL)
    {
        s_motor_a.time = t - MAX_INTERVAL;
        dt = MAX_INTERVAL;
    }

    dbgt.interval = s_motor_a.cur_interval;
    dbgt.dt2 = dt;

    if (dt > s_motor_a.cur_interval)
    {
        s_motor_a.cur_interval = dt;
    }

    // current feed
    uint16_t ocr0 = OCR1A;
    dbgt.ocr0 = ocr0;
    // if (ocr == 0 && s_motor_a.cur_interval > s_motor_a.target_interval) {
    //     dbgt.feed = 0;
    //     ++ocr;
    // } else {
    uint16_t feed =
        (uint16_t)((uint32_t)s_motor_a.cur_interval * (uint32_t)ocr0 /
                   (uint32_t)s_motor_a.target_interval);
    dbgt.feed = feed;
    uint16_t ocr1 = (ocr0 * 31 + feed * 1) / 32;
    if (ocr1 > 255)
        ocr1 = 255;
    // }
    if (ocr1 == ocr0 && s_motor_a.target_interval < s_motor_a.cur_interval) {
        ocr1 = ocr0 + 1;
    }

    dbgt.ocr1 = ocr1;
    dbgt.change = true;

    OCR1A = ocr1;
}

void motor_init(void) {
    PRR &= ~(1 << PRTIM1);

    // MA_DDR |= MA_PHASE_BIT | MA_ENABLE_BIT;
    // MA_PORT |= MA_ENABLE_BIT;

    MA_SENS_PORT &= ~(MA_SENSA_BIT | MA_SENSB_BIT);
    MA_SENS_DDR &= ~(MA_SENSA_BIT | MA_SENSB_BIT);

    PCICR |= (1 << MA_SENS_PCIE);
    MA_SENS_PCMSK = (1 << MA_SENSA_PCINT) | (1 << MA_SENSB_PCINT);

    s_motor_a.sens = MA_SENS_PIN & (MA_SENSA_BIT | MA_SENSB_BIT);
    s_motor_a.target = -1;

    // 8bit fast pwm
    TCCR1A = (1 << WGM10);
    TCCR1B = (1 << WGM12);

    TCNT1 = 0;
}

void motor_start(void) {
    s_motor_a.cur_interval = MAX_INTERVAL;
    s_motor_a.timer_ovf = 0;
    s_motor_a.last_dir = 0;
    s_motor_a.time = (uint16_t)(0 - MAX_INTERVAL);
    s_motor_a.skip = 0;

    // OCR1A = 0xFFFF / s_motor_a.target_interval;

    // reset timer counter
    TCNT1 = 0;
    // enable timer overflow interrupt
    TIMSK1 |= (1 << TOIE1);
    // clear OC on match, set OC
    TCCR1A |= (1 << COM1A1);
    TCCR1B |= CLOCK_SELECT;
}

uint8_t motor_get_feed(void)
{
    return (uint8_t)OCR1A;
}

void motor_set_speed(uint16_t spd) {
    if (spd > 0)
        s_motor_a.target_interval = 0xFFFF / spd;
    else
        s_motor_a.target_interval = 0xFFFF;
}

void motor_move(uint16_t minspd, int16_t delta) {

}

int16_t motor_get_count(void) {
    return s_motor_a.count;
}

uint16_t motor_get_speed(void) {
    return 0xFFFF / s_motor_a.cur_interval;
}

uint16_t motor_get_time(void)
{
    LOCKI();
    uint16_t t = get_time();
    UNLOCKI();
    return t;
}

uint16_t motor_get_skip(void) {
    return s_motor_a.skip;
}

void motor_stop(void) {
    TCCR1B &= ~CLOCK_SELECT;
    TCCR1A &= ~(1 << COM1A1);
    // disable interrupts
    TIMSK1 = 0;
}
