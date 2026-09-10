/*
 * docking_pallet.c
 *
 *  Created on: Sep 8, 2026
 *      Author: syawxyz
 */

#include "docking_pallet.h"

uint8_t limit_sw_routine(limit_switch *p)
{
	uint8_t sw_state;
	uint8_t sw0 = (HAL_GPIO_ReadPin(p->lim_port[0], p->lim_pin[0]) == GPIO_PIN_SET);
	uint8_t sw1 = (HAL_GPIO_ReadPin(p->lim_port[1], p->lim_pin[1]) == GPIO_PIN_SET);

	sw_state= (sw0 || sw1);
	return sw_state;
}
