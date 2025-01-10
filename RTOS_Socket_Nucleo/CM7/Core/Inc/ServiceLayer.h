/*
 * ServiceLayer.h
 *
 *  Created on: Jan 9, 2025
 *      Author: pirda
 */

#ifndef INC_SERVICELAYER_H_
#define INC_SERVICELAYER_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
#include "stm32h7xx_hal.h"

#include "Packet.h"
#include "AT_command.h"

typedef struct {
    uint8_t immobilizeStatus[1]; // 1 byte
    uint8_t rpmPreset[1];        // 1 byte
    uint8_t gpsData[32];          // 6 bytes
    uint8_t currentData[2];      // 2 bytes
    uint8_t voltageData[2];      // 2 bytes
    uint8_t rpm[1];              // 1 byte
    uint8_t temperature[1];      // 1 byte
    uint8_t networkStrength[1];  // 1 byte
} serverProperties;

typedef enum {
    IMMOBILIZE_STATUS = 0x01,
    RPM_PRESET        = 0x02,
    GPS               = 0x03,
    CURRENT           = 0x04,
    VOLTAGE           = 0x05,
    RPM               = 0x06,
    TEMPERATURE       = 0x07,
    NETWORK_STRENGTH  = 0x08
} ServerPropertyType;

void HandleReceivedData(uint8_t writeIndexS);
uint8_t encodeServerData(ServerPropertyType type, uint8_t *packet);
void decodeServerData(uint8_t *packet, uint8_t length);

#endif /* INC_SERVICELAYER_H_ */
