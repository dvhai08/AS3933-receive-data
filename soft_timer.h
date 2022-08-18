#ifndef SOFT_TIMER_H
#define SOFT_TIMER_H

#include "stdint.h"
#include "stdbool.h"

#define SOFT_TIMER_OBJ_NUM	1
#define TON_OBJ_NUM					2

void timer_check(uint8_t b_id, uint32_t dw_now);
void timer_start(uint8_t b_id);
void timer_stop(uint8_t b_id);
void timer_set(uint8_t b_id, uint32_t dw_interval, void(*cb)(void));
bool TON(uint8_t b_id, bool o_in, uint32_t dw_now, uint32_t dw_preset_time);
bool TON_16U(uint8_t b_id, bool o_in, uint32_t dw_now, uint32_t dw_preset_time);


#endif /* SOFT_TIMER_H */
