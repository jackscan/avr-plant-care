#pragma once

#include <stdint.h>
#include <stdbool.h>

// COUNTS PER REVOLUTION
#define CPR 15808

void motor_init(void);
void motor_start(void);
void motor_set_speed(uint8_t minfeed, uint16_t spd);
void motor_set_target(int16_t angle);
void motor_unset_target(void);
void motor_move(uint8_t minfeed, uint16_t maxspd, int16_t angle,
                int8_t revolutions);
void motor_update_feed(void);
int8_t motor_get_remaining_revolutions(void);
uint16_t motor_get_spd(void);
uint16_t motor_get_skip(void);
void motor_get_pos_and_adjust(int16_t *pos, int16_t *adjust);
int16_t motor_get_adjust(void);
uint16_t motor_get_time(void);
/// Returns target speed for rotating count steps in qsec quaters of second.
uint16_t motor_calculate_speed(uint16_t count, uint8_t qsecs);
/// Returns quaters of second it takes for one revolution at given speed.
uint8_t motor_get_time_per_revolution(uint16_t spd);
uint8_t motor_get_feed(void);
bool motor_pos_is_calibrated(void);
bool motor_is_stopped(void);
void motor_enable_pos_sensor(void);
void motor_disable_pos_sensor(void);
void motor_dump_calc(void);
void motor_stop(void);
void motor_unset_calibration(void);

void motor_dump_pos_sens(void);
void debug_dump_motor(void);
