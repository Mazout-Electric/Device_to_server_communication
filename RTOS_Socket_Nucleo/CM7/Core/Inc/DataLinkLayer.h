/*
 * DataLinkLayer.h
 *
 *  Created on: Dec 13, 2024
 *      Author: pirda
 */

#ifndef INC_DATALINKLAYER_H_
#define INC_DATALINKLAYER_H_

#include "CharQueue.h"
#include "SimCom.h"

#include "stm32h7xx_hal.h"

#define DL_BUF_LEN 200
#define DL_RETRY_TIMES 10

bool dl_init(UART_HandleTypeDef *device);
bool dl_receive(char *data, SIMCOM_LENGTH_TYPE *length);
bool dl_send(const char *data, SIMCOM_LENGTH_TYPE length);

#endif /* INC_DATALINKLAYER_H_ */
