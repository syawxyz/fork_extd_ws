/*
 * docking_pallet.h
 *
 *  Created on: Sep 8, 2026
 *      Author: syawxyz
 */

#ifndef INC_DOCKING_PALLET_H_
#define INC_DOCKING_PALLET_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
	/*
	 * [0] left
	 * [1] right
	 */
	GPIO_TypeDef* lim_port[2];
	uint16_t lim_pin[2];
	uint8_t docking_state;
} limit_switch;

uint8_t limit_sw_routine (limit_switch *p);
#endif /* INC_DOCKING_PALLET_H_ */
