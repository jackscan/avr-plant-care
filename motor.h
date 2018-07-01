#pragma once

#include <stdint.h>

void motor_init(void);
void motor_start(void);
void motor_set_speed(uint16_t spd);
uint16_t motor_get_speed(void);
uint16_t motor_get_skip(void);
int16_t motor_get_count(void);
uint16_t motor_get_time(void);
uint8_t motor_get_feed(void);
void motor_dump_calc(void);
void motor_stop(void);

void debug_dump_timer(void);
void debug_dump_motor(void);
