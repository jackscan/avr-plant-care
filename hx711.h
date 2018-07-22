#pragma once

#include <stdint.h>

void hx711_early_init(void);
void hx711_init(void);
void hx711_powerdown(void);
void hx711_calib(uint32_t offset, uint32_t scale);
void hx711_write_calib(void);
uint16_t hx711_read(void);
