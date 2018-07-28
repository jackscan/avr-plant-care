#include "motor.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

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

#define MA_POS_PCMSK PCMSK2
#define MA_POS_PCIE PCIE2
#define MA_POS_PCINT_VECT PCINT2_vect
#define MA_POS_PCINT PCINT18
#define MA_POS_DDR DDRD
#define MA_POS_PORT PORTD
#define MA_POS_ENABLE_BIT (1 << PD3)
#define MA_POS_BIT (1 << PD2)
#define MA_POS_PIN PIND

// 1/1024
// #define CLOCK_SELECT ((1 << CS12) | (1 << CS10))
// 1/256
#define CLOCK_SELECT (1 << CS12)
#define TIMER_CLOCK_DIV 256UL
// 1/64
// #define CLOCK_SELECT ((1 << CS11) | (1 << CS10))

#define POS_CAL_WIDTH   60
#define POS_CAL_WIDTH_TOL 3
#define TIMER_INTERVAL(MS) ((F_CPU * (unsigned long)(MS)) / (TIMER_CLOCK_DIV * 1000UL))
#define MAX_INTERVAL TIMER_INTERVAL(200UL)
#define MIN_INTERVAL TIMER_INTERVAL(10UL)
#define FORESIGHT MIN_INTERVAL * 64UL
#define SPD_SCALE 4096UL
#define SPEED(DC, DT) (uint16_t)((uint32_t)(DC) * SPD_SCALE / (DT))
#define DISTANCE(SPD, DT) (int16_t)(((uint32_t)(SPD) * (uint32_t)(DT)) / SPD_SCALE)
#define TRAVEL_TIME(DC, SPD) ((uint32_t)(DC) * SPD_SCALE / (uint32_t)(SPD))

#define LOCKI() uint8_t sreg = SREG; cli()
#define RELOCKI() sreg = SREG; cli()
#define UNLOCKI() SREG = sreg

static struct
{
    int16_t target;
    volatile int8_t revolutions;
    volatile uint16_t skip;
    volatile uint8_t sens;
    volatile uint8_t last_dir;
    uint16_t target_spd;
    volatile uint16_t timer_ovf;
    uint16_t max_spd;
    uint8_t min_feed;
    volatile int16_t count[3];
    volatile uint16_t time[3];
    // feed which was current at last counter updates
    volatile uint8_t last_feed;
    uint8_t feed;
    volatile bool feed_updated;

    volatile int16_t pos[2];
    volatile bool pos_sens;
    // speed counter got updated
    volatile bool update;
    volatile bool calibrated;
    volatile int16_t adjust;
} s_motor_a;

static uint16_t get_motor_time(void) {
    uint16_t ovf = s_motor_a.timer_ovf;
    uint16_t tcnt = TCNT1;
    // overflow pending?
    if ((TIFR1 & (1 << TOV1)) != 0 && tcnt < 128)
        ++ovf;

    return (ovf << 8) | tcnt;
}

static void update_count(int8_t dir) {
    s_motor_a.count[2] += dir;

    if (s_motor_a.count[2] >= CPR) {
        for (uint8_t i = 0; i < 3; ++i)
            s_motor_a.count[i] -= CPR;
    } else if (s_motor_a.count[2] < 0) {
        for (uint8_t i = 0; i < 3; ++i)
            s_motor_a.count[i] += CPR;
    }

    if (s_motor_a.count[2] == s_motor_a.target) {
        if (s_motor_a.revolutions > 0)
            --s_motor_a.revolutions;
        else
            motor_stop();
    }
}

ISR (MA_SENS_PCINT_VECT) {
    uint16_t t = get_motor_time();

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
            // if we skipped we need to increment two times
            update_count(dir);
            break;
    }

    s_motor_a.sens = sens;
    s_motor_a.last_dir = dir;
    s_motor_a.time[2] = t;

    update_count(dir);

    int16_t dc = s_motor_a.count[2] - s_motor_a.count[1];
    if (dc >= 12 || dc <= -12)
    {
        s_motor_a.last_feed = OCR1A;
        for (uint8_t i = 0; i < 2; ++i) {
            s_motor_a.count[i] = s_motor_a.count[i + 1];
            s_motor_a.time[i] = s_motor_a.time[i + 1];
        }
        s_motor_a.update = true;
    }
}

ISR (TIMER1_OVF_vect) {
    // count for 50ms
    // const uint16_t interval = TIMER_INTERVAL(50UL);
    // if (++s_motor_a.ovf_count > interval)

    ++s_motor_a.timer_ovf;
    uint16_t t = (s_motor_a.timer_ovf << 8) | TCNT1;
    uint16_t dt2 = t - s_motor_a.time[2];

    // data too old?
    if (dt2 > MAX_INTERVAL) {
        for (uint8_t i = 2; i > 0; --i) {
            s_motor_a.time[i-1] = s_motor_a.time[i] - MAX_INTERVAL;
            s_motor_a.count[i - 1] = s_motor_a.count[i];
        }
    }

    s_motor_a.feed_updated = true;
    OCR1A = s_motor_a.feed;
}

void motor_update_feed(void) {

    // wait for feed update
    if (!s_motor_a.feed_updated)
        return;

    LOCKI();
    uint16_t dt = s_motor_a.time[1] - s_motor_a.time[0];
    int16_t dc = s_motor_a.count[1] - s_motor_a.count[0];
    uint16_t t = get_motor_time();
    // last count time
    uint16_t lastt = s_motor_a.time[2];
    // current position
    int16_t pos = motor_get_pos();
    // current pending revolution count
    uint8_t rev = s_motor_a.revolutions;
    // update of position
    uint8_t last_feed = s_motor_a.last_feed;
    bool update = s_motor_a.update;
    s_motor_a.update = false;
    UNLOCKI();

    uint16_t spd = (dt < MAX_INTERVAL) ? SPEED(dc, dt) : 0;

    // we're are running, wait for next position update
    if (spd > 0 && !update)
        return;

    uint16_t dt2 = t - lastt;
    // wait at least 20ms before updating feed on startup
    if (spd == 0 && dt2 < TIMER_INTERVAL(20))
        return;

    // current feed
    uint16_t ocr0 = s_motor_a.feed;
    uint16_t ocr1 = ocr0;
    uint16_t tspd = s_motor_a.target_spd;

    if (tspd == 0) {
        if (rev > 0) {
            tspd = s_motor_a.max_spd;
        } else {
            int16_t dangle = s_motor_a.target - pos;
            if (dangle < 0) dangle += CPR;
            if (dangle >= CPR) dangle -= CPR;

            // estimate angle at current speed after some time
            int16_t est = dangle - DISTANCE(spd, FORESIGHT);
            if (est < 0) est = 0;
            // calculate target speed
            tspd = SPEED(est, FORESIGHT);
            if (tspd > s_motor_a.max_spd) tspd = s_motor_a.max_spd;
        }
    }

    uint16_t feed = 0;

    if (update) {
        feed =
            (uint16_t)((uint32_t)tspd * (uint32_t)last_feed / (uint32_t)spd);
        ocr1 = (ocr0 * 63 + feed * 1) / 64;

        // microsteps
        if (ocr1 == ocr0 && feed < ocr1)
            --ocr1;
        else if (ocr1 == ocr0 && feed > ocr1)
            ++ocr1;
    } else if (spd < tspd) {
        // startup help
        ocr1 = ocr0 + 1;
    }

    if (ocr1 < s_motor_a.min_feed)
        ocr1 = s_motor_a.min_feed;
    else if (ocr1 > 255)
        ocr1 = 255;

    RELOCKI();
    s_motor_a.feed = ocr1;
    s_motor_a.feed_updated = false;
    UNLOCKI();
}

uint16_t motor_calculate_speed(uint16_t count, uint8_t qsecs) {
    uint32_t dt = qsecs * F_CPU / (4 * TIMER_CLOCK_DIV);
    return SPEED(count, dt);
}

uint8_t motor_get_time_per_revolution(uint16_t spd)
{
    uint32_t dt = TRAVEL_TIME(CPR, spd);
    // quater of a second
    uint32_t qs = F_CPU / (TIMER_CLOCK_DIV * 4);
    uint32_t s = (dt + qs/2) / qs;
    return s < 255 ? (uint8_t)s : 255;
}

void motor_dump_pos_sens(void) {
    static bool init = false;
    static bool sens = false;
    if (sens != s_motor_a.pos_sens) {
        sens = s_motor_a.pos_sens;
        if (!init) {
            init = true;
            return;
        }
        int16_t p0 = s_motor_a.pos[0];
        int16_t p1 = s_motor_a.pos[1];
        if (p0 == p1 || p0 == -1) return;
        if (p1 < p0) p1 += CPR;
        int16_t p = (p0 + p1) / 2;
        if (p >= CPR) p -= CPR;
        printf("p: %d %d %d %d %d %d %d\n", sens ? 1 : 0, s_motor_a.pos[0], s_motor_a.pos[1], p, p1 - p0, s_motor_a.calibrated, s_motor_a.count[2]);
    }
}

ISR (MA_POS_PCINT_VECT) {
    bool sens = !!(MA_POS_PIN & MA_POS_BIT);
    if (sens == s_motor_a.pos_sens)
        return;

    if (!sens)
        s_motor_a.pos[0] = s_motor_a.count[2];
    s_motor_a.pos[1] = s_motor_a.count[2];

    if (sens && !s_motor_a.calibrated && s_motor_a.pos[0] >= 0) {
        // calibrate
        int16_t p0 = s_motor_a.pos[0];
        int16_t p1 = s_motor_a.pos[1];
        if (p1 < p0) p1 += CPR;
        int16_t d = (p1 - p0);
        int16_t delta = d / 2 - s_motor_a.count[2];
        for (uint8_t i = 0; i < 3; ++i)
            s_motor_a.count[i] += delta;

        s_motor_a.adjust += delta;

        // plausibility check
        if (POS_CAL_WIDTH - POS_CAL_WIDTH_TOL <= d &&
            d <= POS_CAL_WIDTH + POS_CAL_WIDTH_TOL) {
            s_motor_a.calibrated = true;
            motor_disable_pos_sensor();
        }
    }

    s_motor_a.pos_sens = sens;
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
    s_motor_a.calibrated = false;
    s_motor_a.pos[0] = -1;
    s_motor_a.pos[1] = -1;

    MA_POS_PORT &= ~(MA_POS_BIT | MA_POS_ENABLE_BIT);
    MA_POS_DDR |= MA_POS_ENABLE_BIT;
    MA_POS_DDR &= ~MA_POS_BIT;

    MA_POS_PCMSK = (1 << MA_POS_PCINT);

    // 8bit fast pwm
    TCCR1A = (1 << WGM10);
    TCCR1B = (1 << WGM12);

    TCNT1 = 0;
}

void motor_enable_pos_sensor(void)
{
    MA_POS_PORT |= MA_POS_ENABLE_BIT;
    // wait for powerup
    _delay_ms(1);
    s_motor_a.pos_sens = !!(MA_POS_PIN & MA_POS_BIT);
    PCICR |= (1 << MA_POS_PCIE);
}

void motor_disable_pos_sensor(void) {
    PCICR &= ~(1 << MA_POS_PCIE);
    MA_POS_PORT &= ~MA_POS_ENABLE_BIT;
}

void motor_start(void) {
    s_motor_a.timer_ovf = 0;
    s_motor_a.last_dir = 0;
    for (uint8_t i = 0; i < 3; ++i)
        s_motor_a.time[i] = (uint16_t)(i - 2) * MAX_INTERVAL;
    s_motor_a.skip = 0;
    s_motor_a.update = false;
    s_motor_a.feed = 0;
    s_motor_a.feed_updated = false;

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

void motor_set_speed(uint8_t minfeed, uint16_t spd) {
    s_motor_a.min_feed = minfeed;
    s_motor_a.max_spd = spd;
    s_motor_a.target_spd = spd;
}

void motor_move(uint8_t minfeed, uint16_t maxspd, int16_t angle,
                int8_t revolutions) {
    s_motor_a.min_feed = minfeed;
    s_motor_a.max_spd = maxspd;
    s_motor_a.target_spd = 0;
    s_motor_a.revolutions = revolutions;
    motor_set_target(angle);
}

int16_t motor_get_pos(void) {
    return s_motor_a.count[2];
}

int16_t motor_get_adjust(void) {
    LOCKI();
    int16_t adj = s_motor_a.adjust;
    UNLOCKI();
    return adj;
}

bool motor_pos_is_calibrated(void) {
    return s_motor_a.calibrated;
}

void motor_unset_calibration(void) {
    s_motor_a.calibrated = false;
}

int8_t motor_get_remaining_revolutions(void) {
    return s_motor_a.revolutions;
}

uint16_t motor_get_speed(void) {
    LOCKI();
    int16_t c0 = s_motor_a.count[0];
    int16_t c1 = s_motor_a.count[1];
    uint16_t t0 = s_motor_a.time[0];
    uint16_t t1 = s_motor_a.time[1];
    UNLOCKI();

    uint16_t dt = t1 - t0;
    if (dt == 0) dt = 1;

    return (c1 - c0) / dt;
}

static uint16_t motor_get_spd(void) {
    LOCKI();
    uint16_t dt = s_motor_a.time[1] - s_motor_a.time[0];
    int16_t dc = s_motor_a.count[1] - s_motor_a.count[0];
    UNLOCKI();

    return (dt < MAX_INTERVAL) ? SPEED(dc, dt) : 0;
}

void motor_set_target(int16_t angle) {
    uint16_t spd = motor_get_spd();
    // stopping distance
    int16_t d = 2 * DISTANCE(spd, FORESIGHT);
    while (angle < 0) angle += CPR;
    while (angle >= CPR) angle -= CPR;
    LOCKI();
    s_motor_a.target = angle;
    int16_t da = angle - motor_get_pos();
    if (da < 0) da += CPR;
    if (da >= CPR) da -= CPR;
    int8_t minrev = 0;
    if (da < d || !s_motor_a.calibrated)
        minrev = 1;
    if (!s_motor_a.calibrated && da < d + POS_CAL_WIDTH * 2)
        minrev = 2;
    if (s_motor_a.revolutions < minrev)
        s_motor_a.revolutions = minrev;
    UNLOCKI();

    if (!s_motor_a.calibrated) {
        motor_enable_pos_sensor();
    }
}

void motor_unset_target(void) {
    s_motor_a.target = -1;
}

uint16_t motor_get_time(void)
{
    LOCKI();
    uint16_t t = get_motor_time();
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
    // reset feed
    OCR1A = 0;
    s_motor_a.feed = 0;

    if (s_motor_a.skip > 0)
        motor_unset_calibration();
}

bool motor_is_stopped(void) {
    return (TCCR1B & CLOCK_SELECT) == 0;
}
