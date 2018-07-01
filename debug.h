#pragma once

#include <stdbool.h>

void debug_init(void);
void debug_finish(void);
bool debug_char_pending(void);
char debug_getchar(void);

void debug_init_trace(void);
void debug_dump_trace(void);

#if ENABLE_CHECKPOINTS
void checkpoint(void);
#define CHECKPOINT checkpoint()
// #define ISR_CHECKPOINT __asm__ __volatile__("rcall", )
#else
#define CHECKPOINT
// #define ISR_CHECKPOINT
#endif
