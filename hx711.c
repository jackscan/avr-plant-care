#include "hx711.h"

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

#include <util/delay.h>

#include <stdio.h>

#define SERIAL_DDR  DDRD
#define SERIAL_PORT PORTD
#define SERIAL_PIN  PIND
#define PD_SCK      (1 << PD6)
#define DOUT        (1 << PD7)

struct scale_calib {
    uint32_t offset;
    uint32_t scale;
};

static struct scale_calib s_calib_data;
static EEMEM struct scale_calib eemem_scale_calib;

void hx711_early_init(void) {
    // enable digital input DOUT
    if (&SERIAL_DDR == &DDRD && (DOUT & ((1 << PD7) | (1 << PD6))) != 0) {
        DIDR1 &= ~((DOUT & ((1 << PD7) | (1 << PD6))) >> PD6);
    }
}

void hx711_init(void) {
    SERIAL_DDR |= PD_SCK;
    SERIAL_DDR &= ~DOUT;

    // no pull up on DOUT
    SERIAL_PORT &= ~DOUT;

    hx711_powerdown();
    eeprom_read_block(&s_calib_data, &eemem_scale_calib, sizeof s_calib_data);
}

void hx711_powerdown(void)
{
    SERIAL_PORT |= PD_SCK;
    _delay_us(60);
}

void hx711_calib(uint32_t offset, uint32_t scale) {
    s_calib_data.offset = offset;
    s_calib_data.scale = scale;
}

void hx711_write_calib(void) {
    eeprom_write_block(&s_calib_data, &eemem_scale_calib, sizeof s_calib_data);
}

uint16_t hx711_read(void) {
    // power up
    SERIAL_PORT &= ~PD_SCK;

    // wait for hx711 to become ready
    while ((SERIAL_PIN & DOUT) != 0)
        ;

    _delay_us(0.1);

    uint32_t result = 0;
    uint8_t i;
    // we are only interested in the 17 upper bits
    for (i = 0; i < 17; ++i) {
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);

        result <<= 1;

        if ((SERIAL_PIN & DOUT) != 0)
            result |=  1;

        // flip first bit for unsigned value
        if (i == 0)
            result ^= 1;

        SERIAL_PORT &= ~PD_SCK;
        _delay_us(0.2);
    }

    // pulse 25 times
    for (; i < 25; ++i) {
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);
        SERIAL_PORT &= ~PD_SCK;
        _delay_us(0.2);
    }

    if (s_calib_data.offset != 0xFFFFFFFF && s_calib_data.scale != 0xFFFFFFFF) {
        result -= s_calib_data.offset;
        result *= s_calib_data.scale;
        result /= 256UL;
    } else {
        result >>= 1;
    }

    return (uint16_t)result;
}
