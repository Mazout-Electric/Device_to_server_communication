/*
 * Network.h
 *
 *  Created on: Jan 9, 2025
 *      Author: pirda
 */

#ifndef INC_NETWORK_H_
#define INC_NETWORK_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
#include "stm32h7xx_hal.h"

#include "AT_command.h"
#include "Packet.h"
#include "ServiceLayer.h"



//serverProperties serverAttributesN;

void NetworkInit(void);
void OpenSocket(void);
void SocketSendData(void);
void SocketReceiveData(void);

#endif /* INC_NETWORK_H_ */
