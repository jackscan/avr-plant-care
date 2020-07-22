#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CMD_GET_LAST_WATERING   0x10
#define CMD_GET_WATER_LIMIT     0x11
#define CMD_GET_WEIGHT          0x12
#define CMD_ROTATE              0x13
#define CMD_STOP                0x14
#define CMD_GET_MOTOR_STATUS    0x15
#define CMD_SET_WATER_REFILL    0x16
#define CMD_GET_WATER_REFILL    0x17
#define CMD_WATERING            0x1A
#define CMD_ECHO                0x29

void twi_slave_init(uint8_t addr);
bool twi_busy(void);
bool twi_cmd_pending(void);
uint8_t twi_get_cmd(void);
uint8_t twi_next_cmd(void);
void twi_dump_trace(void);

uint8_t twi_get_watering(void);
uint8_t twi_get_watering_startup(void);
void twi_set_last_watering(uint8_t);
void twi_add_weight(uint16_t value);
uint32_t twi_get_weight(void);
uint16_t twi_get_target_angle(void);
void twi_set_angle(uint16_t angle);

bool twi_get_stop_flag(void);
void twi_unset_stop_flag(void);
