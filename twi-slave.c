#include "twi-slave.h"
#include "water.h"
#include "motor.h"
#include "debug.h"

#include <util/twi.h>
#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define LOCKI() uint8_t sreg = SREG; cli()
#define RELOCKI() sreg = SREG; cli()
#define UNLOCKI() SREG = sreg

#define BUFFER_SIZE 4

#define STATE_TRACE_LEN 16
#define STATE_TRACE_MASK  (STATE_TRACE_LEN-1)

#define TW_RESPONSE(A)                                                         \
    (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | ((!!(A)) << TWEA) |             \
        (0 << TWSTA) | (0 << TWSTO) | (0 << TWWC)

struct accu {
    uint16_t numerator;
    uint16_t denominator;
};

enum twi_state {
    TWI_STATE_BUSY = 0x1,
    TWI_STATE_CMD_PENDING = 0x2,
};

static struct {
    uint8_t buf[BUFFER_SIZE];
    uint8_t buflen;
    uint8_t ptr;
    uint8_t cmd;
    struct {
        struct accu weight;
        uint16_t angle;
        uint16_t target_angle;
        uint8_t last_watering;
        uint8_t watering_startup;
        uint8_t watering;
        bool stop;
    } data;
    uint8_t state;
} s_twi;

static struct {
    uint8_t trace[STATE_TRACE_LEN];
    uint8_t tdata[STATE_TRACE_LEN];
    uint8_t head, tail;
    uint8_t lost;
} s_twi_dbg;

void twi_slave_init(uint8_t addr) {
    PRR &= ~(1 << PRTWI);
    TWAR = addr << TWA0; // Set own TWI slave address.
    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA) |
           (0 << TWSTA) | (0 << TWSTO) | (0 << TWWC);
}

static const char *status_to_str(uint8_t status) {
    switch (status) {
        case TW_ST_SLA_ACK: return "ST_SLA_ACK";
        case TW_ST_ARB_LOST_SLA_ACK: return "ST_ARB_LOST_SLA_ACK";
        case TW_ST_DATA_ACK: return "ST_DATA_ACK";
        case TW_ST_DATA_NACK: return "ST_DATA_NACK";
        case TW_ST_LAST_DATA: return "ST_LAST_DATA";
        case TW_SR_SLA_ACK: return "SR_SLA_ACK";
        case TW_SR_ARB_LOST_SLA_ACK: return "SR_ARB_LOST_SLA_ACK";
        case TW_SR_GCALL_ACK: return "SR_GCALL_ACK";
        case TW_SR_ARB_LOST_GCALL_ACK: return "SR_ARB_LOST_GCALL_ACK";
        case TW_SR_DATA_ACK: return "SR_DATA_ACK";
        case TW_SR_DATA_NACK: return "SR_DATA_NACK";
        case TW_SR_GCALL_DATA_ACK: return "SR_GCALL_DATA_ACK";
        case TW_SR_GCALL_DATA_NACK: return "SR_GCALL_DATA_NACK";
        case TW_SR_STOP: return "SR_STOP";
        case TW_NO_INFO: return "NO_INFO";
        case TW_BUS_ERROR: return "BUS_ERROR";
        default: return "unknown";
    }
}

static void add_accu(struct accu* a, uint16_t v) {
    uint32_t sum = (uint32_t)a->numerator + (uint32_t)v;

    if (sum > 0xFFFF || a->denominator >= 0xFFFE) {
        if (a->denominator & 1) {
            a->numerator = (uint16_t)((sum) / 2);
            a->denominator = (a->denominator + 1) / 2;
        } else {
            a->numerator = a->numerator / 2 + v;
            a->denominator = a->denominator / 2 + 1;
        }
    } else {
        a->numerator = (uint16_t)sum;
        ++a->denominator;
    }
}

static void reset_accu(struct accu* a) {
    a->numerator = 0;
    a->denominator = 0;
}

bool twi_get_stop_flag(void) {
    return s_twi.data.stop;
}
void twi_unset_stop_flag(void) {
    s_twi.data.stop = false;
}

uint8_t twi_get_watering(void) {
    return s_twi.data.watering;
}

uint8_t twi_get_watering_startup(void) {
    return s_twi.data.watering_startup;
}

void twi_set_last_watering(uint8_t w) {
    s_twi.data.last_watering = w;
}

void twi_add_weight(uint16_t value) {
    LOCKI();
    add_accu(&s_twi.data.weight, value);
    UNLOCKI();
}

uint32_t twi_get_weight(void) {
    return ((uint32_t)s_twi.data.weight.numerator << 16) |
           (uint32_t)s_twi.data.weight.denominator;
}

uint16_t twi_get_target_angle(void) {
    return s_twi.data.target_angle;
}

void twi_set_angle(uint16_t angle) {
    LOCKI();
    s_twi.data.angle = angle;
    UNLOCKI();
}

bool twi_busy(void) {
    return (s_twi.state & TWI_STATE_BUSY) != 0;
}

static inline void twi_set_busy(void) {
    s_twi.state |= TWI_STATE_BUSY;
}

static inline void twi_reset_busy(void) {
    s_twi.state &= ~TWI_STATE_BUSY;
}

bool twi_cmd_pending(void) {
    return (s_twi.state & TWI_STATE_CMD_PENDING) != 0;
}

uint8_t twi_get_cmd(void) {
    return s_twi.cmd;
}

uint8_t twi_next_cmd(void) {
    LOCKI();
    uint8_t cmd = s_twi.cmd;
    s_twi.state &= ~(TWI_STATE_CMD_PENDING);
    UNLOCKI();
    return cmd;
}

void twi_dump_trace(void) {
    if (s_twi_dbg.head == s_twi_dbg.tail) return;

    LOCKI();
    uint8_t i = s_twi_dbg.head;
    uint8_t end = s_twi_dbg.tail;
    uint8_t s = s_twi_dbg.lost;
    s_twi_dbg.lost = 0;
    UNLOCKI();

    for (; i != end; i = ((i + 1) & STATE_TRACE_MASK)) {
        printf("%s, %#x\n", status_to_str(s_twi_dbg.trace[i]),
               s_twi_dbg.tdata[i]);
    }

    RELOCKI();
    s_twi_dbg.head = end;
    UNLOCKI();

    switch (s) {
    case 0:
        break;
    case 255:
        printf("255 or more states lost\n");
        break;
    default:
        printf("%d states lost\n", s);
        break;
    }
}

static void load_uint16(uint16_t value) {
    s_twi.buf[0] = (uint8_t)(value & 0xFF);
    s_twi.buf[1] = (uint8_t)((value >> 8) & 0xFF);
    s_twi.buflen = 2;
}

static void load_accu(struct accu *a, uint16_t amplify) {
    uint16_t d = (a->denominator + amplify / 2) / amplify;

    if (d > 0) {
        uint16_t value = (a->numerator + d / 2) / d;
        load_uint16(value);
    } else {
        s_twi.buf[0] = 0xFF;
        s_twi.buflen = 0;
    }
}

static void reset_cmd(void) {
    s_twi.cmd = 0;
    s_twi.state &= ~(TWI_STATE_CMD_PENDING);
}

static void fill_buffer(void) {
    switch (s_twi.cmd) {
    case CMD_GET_WEIGHT:
        load_accu(&s_twi.data.weight, 1);
        break;
    case CMD_WATERING: // fallthrough
    case CMD_GET_LAST_WATERING:
        s_twi.buf[0] = s_twi.data.last_watering;
        s_twi.buflen = 1;
        break;
    case CMD_GET_WATER_LIMIT:
        s_twi.buf[0] = water_limit(MAX_WATER_TIME);
        s_twi.buflen = 1;
        break;
    case CMD_ROTATE: // fallthrough
    case CMD_GET_MOTOR_STATUS: {
        s_twi.buf[0] = motor_get_feed();
        uint16_t skip = motor_get_skip();
        if (skip > 63)
            skip = 63;
        s_twi.buf[1] = (motor_is_stopped() ? 0 : 0x80) |
                       (motor_pos_is_calibrated() ? 0x40 : 0) | (uint8_t)skip;
        s_twi.buflen = 2;
        break;
    }
    case CMD_ECHO:
        break;
    }

}

static void handle_cmd(void) {
    if (s_twi.buflen > 0) {
        s_twi.cmd = s_twi.buf[0];
        s_twi.state |= TWI_STATE_CMD_PENDING;

        switch (s_twi.cmd) {
        case CMD_GET_WEIGHT:
            reset_accu(&s_twi.data.weight);
            break;
        case CMD_WATERING:
            s_twi.data.last_watering = 0;
            if (s_twi.buflen == 3) {
                s_twi.data.watering_startup = s_twi.buf[1];
                s_twi.data.watering = s_twi.buf[2];
            } else {
                reset_cmd();
            }
            break;
        case CMD_ROTATE:
            if (s_twi.buflen == 3) {
                s_twi.data.target_angle =
                    (uint16_t)s_twi.buf[1] | ((uint16_t)s_twi.buf[2] << 8);
            } else {
                reset_cmd();
            }
            break;
        case CMD_STOP:
            s_twi.data.stop = true;
            break;
        case CMD_ECHO:
            break;
        }
    }
}

ISR(TWI_vect) {
    static uint8_t buf_idx;

    uint8_t status = TW_STATUS;
    uint8_t data = TWDR;

    switch (status) {
    case TW_ST_SLA_ACK:  // Own SLA+R has been received; ACK has been returned
        CHECKPOINT;
        fill_buffer();   // Prepare buffer
        buf_idx = 0;     // Set buffer pointer to first data location
    case TW_ST_DATA_ACK: // Data byte in TWDR has been transmitted; ACK has been
                         // received
        CHECKPOINT;
        data = s_twi.buf[buf_idx++];
        TWDR = data;
        _delay_us(0.25);
        TWCR = TW_RESPONSE(buf_idx < s_twi.buflen);
        twi_set_busy();
        break;

    case TW_SR_SLA_ACK: // Own SLA+W has been received ACK has been returned
        CHECKPOINT;
        s_twi.buflen = 0;
        buf_idx = 0;    // Set buffer pointer to first data location
        TWCR = TW_RESPONSE(true);
        twi_set_busy();
        break;

    case TW_SR_DATA_ACK: // Previously addressed with own SLA+W; data has been
                         // received; ACK has been returned
        CHECKPOINT;
        s_twi.buf[buf_idx++] = data;
        TWCR = TW_RESPONSE(buf_idx + 1 < BUFFER_SIZE);
        twi_set_busy();
        break;

    case TW_SR_DATA_NACK:      // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
        CHECKPOINT;
        s_twi.buf[buf_idx++] = data;
    case TW_SR_STOP:       // A STOP condition or repeated START condition has been received while still addressed as Slave
        CHECKPOINT;
        s_twi.buflen = buf_idx;
        TWCR = TW_RESPONSE(true);
        handle_cmd();
        twi_reset_busy();
        break;

    // These cases should not happen, as we don't support general call or master modes
    case TW_SR_ARB_LOST_SLA_ACK:
    case TW_SR_GCALL_ACK:
    case TW_SR_ARB_LOST_GCALL_ACK:
    case TW_SR_GCALL_DATA_ACK:
        CHECKPOINT;
        TWCR = TW_RESPONSE(false);
        twi_set_busy();
        break;

    case TW_BUS_ERROR: // Bus error due to an illegal START or STOP condition
        CHECKPOINT;
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA) |
               (0 << TWSTA) | (1 << TWSTO) | (0 << TWWC);
        twi_reset_busy();
        break;

    case TW_ST_LAST_DATA: // Last data byte in TWDR has been transmitted (TWEA =
                          // '0'); ACK has been received
        CHECKPOINT;
    case TW_ST_DATA_NACK:          // Data byte in TWDR has been transmitted; NACK has been received.
                                     // I.e. this could be the end of the transmission.
        CHECKPOINT;
    case TW_SR_GCALL_DATA_NACK:
        CHECKPOINT;
    default:
        CHECKPOINT;
        TWCR = TW_RESPONSE(true);
        twi_reset_busy();
        break;
    }

    {
        uint8_t next = (s_twi_dbg.tail + 1) & STATE_TRACE_MASK;
        if (next != s_twi_dbg.head) {
            s_twi_dbg.trace[s_twi_dbg.tail] = status;
            s_twi_dbg.tdata[s_twi_dbg.tail] = data;
            s_twi_dbg.tail = next;
        } else {
            if (s_twi_dbg.lost < 255) ++s_twi_dbg.lost;
        }
    }

    CHECKPOINT;
}
