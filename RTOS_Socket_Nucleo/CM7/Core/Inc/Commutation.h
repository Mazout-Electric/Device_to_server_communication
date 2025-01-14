/*
 * Commutation.h
 *
 *  Created on: Jan 10, 2025
 *      Author: pirda
 */

#ifndef INC_COMMUTATION_H_
#define INC_COMMUTATION_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
#include "stm32h7xx_hal.h"

void SetCommutationStep(uint8_t step, uint16_t duty);
void threeSine(uint16_t degree);

#endif /* INC_COMMUTATION_H_ */
