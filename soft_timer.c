#include "soft_timer.h"

#define TIME_PASSED(now, since)	(now >= since) ? (now - since) : (now + (UINT32_MAX - since))
#define TIME_OVER(target,time) ((uint32_t)((time) - (target)) < 0x80000000U)
#define TIME_OVER_U16(target,time) ((uint16_t)((time) - (target)) < 0x8000U) // max target is 32768

typedef struct
{
	bool o_in;
	bool o_aux;
	void(*fp)(void);
	uint32_t dw_interval;
	uint32_t dw_since;
} soft_timer_t;

typedef struct
{
	bool o_aux;
	uint32_t dw_since;
} ton_t;

soft_timer_t s_timer_obj[SOFT_TIMER_OBJ_NUM];
ton_t s_ton_obj[TON_OBJ_NUM];

/**
 * \fn void timer_check(uint8_t b_id, uint32_t dw_now)
 * \brief Start a periodic timer with a specified duration .
 *
 * \param t - variables are stored which function needs
 * \param now - system tick continuously running
 * 
 */
void timer_check(uint8_t b_id, uint32_t dw_now)
{
	if(s_timer_obj[b_id].o_in)
	{
		if(!s_timer_obj[b_id].o_aux)
 		{
 			s_timer_obj[b_id].dw_since = dw_now + s_timer_obj[b_id].dw_interval;
 			s_timer_obj[b_id].o_aux = true;
 		}
		else if(TIME_OVER(s_timer_obj[b_id].dw_since, dw_now))
		{
				s_timer_obj[b_id].dw_since = dw_now + s_timer_obj[b_id].dw_interval;
				s_timer_obj[b_id].fp();
		}
		else
		{}
	}
	else
	{
		s_timer_obj[b_id].o_aux = false;
	}
}

/**
 * \fn void timerStart(timeout_t*, uint32_t)
 * \brief
 * \param s
 * \param interval
 */
void timer_start(uint8_t b_id)
{
	s_timer_obj[b_id].o_in = true;
}

/**
 * \fn void timerStop(timeout_t*)
 * \brief
 * \param s
 */
void timer_stop(uint8_t b_id)
{
	s_timer_obj[b_id].o_in = false;
}

/**
* @brief void timer_set(uint8_t b_id, uint32_t dw_interval, void(*fp)(void))
* @param b_id
* @param uint32_t dw_interval
* @param void(*fp)(void)
* 
* @retval none
**/
void timer_set(uint8_t b_id, uint32_t dw_interval, void(*fp)(void))
{
	s_timer_obj[b_id].dw_interval = dw_interval;
	s_timer_obj[b_id].fp = fp;
}

/**
 * \fn bool TON(uint8_t b_id, bool o_in, uint32_t dw_now, uint32_t dw_preset_time)
 * \brief Start a timer with a specified duration as on-delay.
 * \param b_id active TON obj selection
 * \param o_in - timer is executed when the "in" state changes from 0 to 1
 * \param dw_now - system tick continuously running
 * \param dw_preset_time - timer is started for the time stored in
 * \return if time is over , return value is 1
 */
bool TON(uint8_t b_id, bool o_in, uint32_t dw_now, uint32_t dw_preset_time)
{
	bool ret_val = false;

	if(o_in)
	{
		if(!s_ton_obj[b_id].o_aux)
 		{
			s_ton_obj[b_id].dw_since = dw_now + dw_preset_time;
			s_ton_obj[b_id].o_aux = true;
 		}
		else if(TIME_OVER(s_ton_obj[b_id].dw_since, dw_now))
		{
			ret_val = true;
		}
	}
	else
	{
		s_ton_obj[b_id].o_aux = false;
	}

	return ret_val;
}

bool TON_16U(uint8_t b_id, bool o_in, uint32_t dw_now, uint32_t dw_preset_time)
{
	bool ret_val = false;

	if(o_in)
	{
		if(!s_ton_obj[b_id].o_aux)
 		{
			s_ton_obj[b_id].dw_since = dw_now + dw_preset_time;
			s_ton_obj[b_id].o_aux = true;
 		}
		else if(TIME_OVER_U16(uint16_t)(s_ton_obj[b_id].dw_since, (uint16_t)dw_now))
		{
			ret_val = true;
		}
	}
	else
	{
		s_ton_obj[b_id].o_aux = false;
	}

	return ret_val;
}
