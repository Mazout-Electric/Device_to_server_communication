/*
 * ServiceLayer.c
 *
 *  Created on: Jan 9, 2025
 *      Author: pirda
 */

#include "ServiceLayer.h"

char txBufferS[RX_BUFFER_SIZE];        // Buffer for sending AT commands
char rxBufferS[RX_BUFFER_SIZE];        // Buffer for receiving AT responses
char checkBufferS[RX_BUFFER_SIZE];

uint16_t writeIndexS = 0;  // Updated by DMA
uint16_t readIndexS = 0;   // Updated by application


serverProperties serverAttributes;

extern UART_HandleTypeDef huart1;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    //if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
      //  __HAL_UART_CLEAR_IDLEFLAG(&huart1);  // Clear the idle flag

	// Process received data
	writeIndexS = Size;//(RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx));// % RX_BUFFER_SIZE;
	HandleReceivedData(writeIndexS);
}

void HandleReceivedData(uint8_t writeIndexS) {

	uint16_t newDataCount = (writeIndexS >= readIndexS)
	                            ? (writeIndexS - readIndexS)
	                            : (RX_BUFFER_SIZE - readIndexS + writeIndexS);
	memset(checkBufferS, '\0', RX_BUFFER_SIZE);
	for (uint16_t i = 0; i < newDataCount; i++) {
		// Copy new data to the process buffer
		uint8_t newByte = rxBufferS[readIndexS];
		checkBufferS[i] = newByte;

		// Increment read index circularly
		readIndexS = (readIndexS + 1) % RX_BUFFER_SIZE;
	}
	// Check if we have a complete packet
	if (readIndexS >= PACKET_MIN_LENGTH) { // Assume minimum length is 2 bytes (Type + Length)
		if (checkBufferS[0] == 0xAA) {
			decodeServerData((uint8_t *)checkBufferS, readIndexS);
		}
	}
}


uint8_t encodeServerData(ServerPropertyType type, uint8_t *packet) {
    uint8_t payloadLength = 0;
    uint8_t *payload;

    switch (type) {
        case IMMOBILIZE_STATUS:
            payload = serverAttributes.immobilizeStatus;
            payloadLength = sizeof(serverAttributes.immobilizeStatus);
            break;
        case RPM_PRESET:
            payload = serverAttributes.rpmPreset;
            payloadLength = sizeof(serverAttributes.rpmPreset);
            break;
        case GPS:
            payload = serverAttributes.gpsData;
            payloadLength = sizeof(serverAttributes.gpsData);
            break;
        case CURRENT:
            payload = serverAttributes.currentData;
            payloadLength = sizeof(serverAttributes.currentData);
            break;
        case VOLTAGE:
            payload = serverAttributes.voltageData;
            payloadLength = sizeof(serverAttributes.voltageData);
            break;
        case RPM:
            payload = serverAttributes.rpm;
            payloadLength = sizeof(serverAttributes.rpm);
            break;
        case TEMPERATURE:
            payload = serverAttributes.temperature;
            payloadLength = sizeof(serverAttributes.temperature);
            break;
        case NETWORK_STRENGTH:
            payload = serverAttributes.networkStrength;
            payloadLength = sizeof(serverAttributes.networkStrength);
            break;
        default:
            return 0; // Unknown type
    }

    // Create the packet
    uint8_t index = 0;
    packet[index++] = 0xAA;  // Header byte 1
    packet[index++] = 0xBB;  // Header byte 2
    packet[index++] = type;  // Property type
    packet[index++] = payloadLength; // Payload length

    // Copy payload
    memcpy(&packet[index], payload, payloadLength);
    index += payloadLength;

    // Add checksum
    uint8_t checksum = 0;
    for (uint8_t i = 2; i < index; i++) {
        checksum ^= packet[i];
    }
    packet[index++] = checksum;

    return index; // Total packet length
}

void decodeServerData(uint8_t *packet, uint8_t length) {
    if (length < 5) return; // Invalid packet length

    // Validate header
    if (packet[0] != 0xAA || packet[1] != 0xBB) return;

    // Extract type and payload length
    ServerPropertyType type = packet[2];
    uint8_t payloadLength = packet[3];

    // Validate checksum
   // uint8_t checksum = 0;
   // for (uint8_t i = 2; i < 4 + payloadLength; i++) {
   //     checksum ^= packet[i];
   // }
   // if (checksum != packet[4 + payloadLength]) return;

    // Extract payload
    uint8_t *payload = &packet[4];

    // Update serverAttributes
    switch (type) {
        case IMMOBILIZE_STATUS:
            memcpy(serverAttributes.immobilizeStatus, payload, payloadLength);
            break;
        case RPM_PRESET:
            memcpy(serverAttributes.rpmPreset, payload, payloadLength);
            break;
        case GPS:
            memcpy(serverAttributes.gpsData, payload, payloadLength);
            break;
        case CURRENT:
            memcpy(serverAttributes.currentData, payload, payloadLength);
            break;
        case VOLTAGE:
            memcpy(serverAttributes.voltageData, payload, payloadLength);
            break;
        case RPM:
            memcpy(serverAttributes.rpm, payload, payloadLength);
            break;
        case TEMPERATURE:
            memcpy(serverAttributes.temperature, payload, payloadLength);
            break;
        case NETWORK_STRENGTH:
            memcpy(serverAttributes.networkStrength, payload, payloadLength);
            break;
        default:
            // Unknown type
            break;
    }
}

