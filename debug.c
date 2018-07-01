#include "debug.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define BAUDRATE 38400UL

static int dbg_putchar(char c, FILE *stream);
static FILE dbgstdout = FDEV_SETUP_STREAM(dbg_putchar, NULL, _FDEV_SETUP_WRITE);

#define DBGBUFSIZE 32
#define DBGBUFMASK (DBGBUFSIZE-1)

static volatile struct {
    char inbuf[DBGBUFSIZE];
    uint8_t head, tail;
} s_dbg;

ISR(USART_RX_vect) {
    uint8_t next = (s_dbg.tail + 1) & DBGBUFMASK;
    if (s_dbg.head != next) {
        s_dbg.inbuf[s_dbg.tail] = UDR0;
        s_dbg.tail = next;
    }
}

void debug_init(void) {

    // enable clock for uart0
    PRR &= ~(1 << PRUSART0);

    // clear status register, especially ensure U2X bit is zero
    UCSR0A = 0;

    // set baudrate
    uint16_t ubrr = (uint16_t)(
        ((uint32_t)F_CPU + 8UL * BAUDRATE) / (16UL * BAUDRATE) - 1UL);
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr);

    // set 8bit character size
    UCSR0C = (3 << UCSZ00);
    // enable receiver and transmitter
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);

    stdout = &dbgstdout;
}

static int dbg_putchar(char c, FILE *stream) {
    while ((UCSR0A & (1 << UDRE0)) == 0)
        ;
    UCSR0A |= (1 << TXC0);
    UDR0 = c;
    return c;
}

bool debug_char_pending(void) {
    return s_dbg.head != s_dbg.tail;
}

char debug_getchar(void) {
    while (!debug_char_pending())
        ;
    char c = s_dbg.inbuf[s_dbg.head];
    s_dbg.head = (s_dbg.head + 1) & DBGBUFMASK;
    return c;
}

// void debug_putchar(char c) {
//     while ((UCSR0A & (1 << UDRE0)) == 0)
//         ;
//     UCSR0A |= (1 << TXC0);
//     UDR0 = c;
// }

// void debug_write(const char *str, uint8_t len) {
//     while (len > 0) {
//         debug_putchar(*str);
//         ++str;
//         --len;
//     }
// }

void debug_finish(void) {
    uint8_t finish_mask = (1 << UDRE0) | (1 << TXC0);
    while ((UCSR0A & finish_mask) != finish_mask)
        ;
}

#if ENABLE_CHECKPOINTS

#define TRACE_LEN 32
static struct {
    uint8_t index;
    uint16_t addr[TRACE_LEN];
} s_trace __attribute__ ((section (".noinit")));

void __attribute__ ((noinline, naked)) checkpoint(void) {
    __asm__ __volatile__(
        "pop r31" "\n\t" // pop return address into Z
        "pop r30" "\n\t"
        "ldi r26, lo8(%[idx])" "\n\t" // X = &s_trace
        "ldi r27, hi8(%[idx])" "\n\t"
        "ld r18, X" "\n\t" // tmp = s_trace.index
        "subi r18, -2" "\n\t" // tmp += 2
        "andi r18, %[msk]" "\n\t" // tmp &= msk
        "st X+, r18" "\n\t" // s_trace.index = tmp, X = s_trace.addr
        "add r26, r18" "\n\t" // X += tmp
        "adc r27, __zero_reg__" "\n\t"
        "st X+, r30" "\n\t"
        "st X, r31" "\n\t"
        "ijmp" "\n\t"
        :
        :
        [idx] "i" (&s_trace.index),
        [msk] "i" ((2*TRACE_LEN)-1)
        :
    );
}

#endif

void debug_init_trace(void) {
#if ENABLE_CHECKPOINTS
    memset(&s_trace, 0, sizeof(s_trace));
#endif
}

void debug_dump_trace(void) {
#if ENABLE_CHECKPOINTS
    if ((s_trace.index & 1) == 0 &&
        s_trace.index < sizeof(s_trace.addr)) {
        printf("trace:");
        uint8_t i = s_trace.index / 2;
        do {
            i = (i + 1) % TRACE_LEN;
            printf(" %#x", s_trace.addr[i] * 2);
        } while (i * 2 != s_trace.index);
        printf("\n");
    }
#endif
}
