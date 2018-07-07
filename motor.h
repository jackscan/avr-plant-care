#pragma once

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_FULL_ROTATION (120U*12U*10U)

void motor_init(void);
void motor_start(void);
void motor_set_speed(uint8_t minfeed, uint16_t spd);
void motor_set_target(int16_t angle);
void motor_unset_target(void);
void motor_move(uint8_t minfeed, uint16_t maxspd, int16_t angle);
uint16_t motor_get_speed(void);
uint16_t motor_get_skip(void);
int16_t motor_get_count(void);
uint16_t motor_get_time(void);
uint8_t motor_get_feed(void);
void motor_set_pos_sensor(bool enable);
void motor_dump_calc(void);
void motor_stop(void);

void motor_dump_pos_sens(void);
void debug_dump_timer(void);
void debug_dump_motor(void);
