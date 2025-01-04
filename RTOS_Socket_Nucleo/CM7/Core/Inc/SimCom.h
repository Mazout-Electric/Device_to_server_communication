/*
 * SimCom.h
 *
 *  Created on: Dec 13, 2024
 *      Author: pirda
 */

#ifndef INC_SIMCOM_H_
#define INC_SIMCOM_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

/*
  You can modify this for your own project
*/
#define SIMCOM_LENGTH_TYPE uint16_t
#define SIMCOM_DLENGTH_TYPE uint32_t

bool sl_init(UART_HandleTypeDef *device);

#endif /* INC_SIMCOM_H_ */
