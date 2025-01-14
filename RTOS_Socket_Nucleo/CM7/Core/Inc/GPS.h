/*
 * GPS.h
 *
 *  Created on: Jan 9, 2025
 *      Author: pirda
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
#include "stm32h7xx_hal.h"

#include "AT_command.h"
#include "Packet.h"
#include "ServiceLayer.h"



/*
typedef struct {
    uint8_t immobilizeStatus[1]; // 1 byte
    uint8_t rpmPreset[1];        // 1 byte
    uint8_t gpsData[32];          // 6 bytes
    uint8_t currentData[2];      // 2 bytes
    uint8_t voltageData[2];      // 2 bytes
    uint8_t rpm[1];              // 1 byte
    uint8_t temperature[1];      // 1 byte
    uint8_t networkStrength[1];  // 1 byte
} serverPropertiesG;

serverPropertiesG serverAttributesG;*/

void gps(void);

#endif /* INC_GPS_H_ */
