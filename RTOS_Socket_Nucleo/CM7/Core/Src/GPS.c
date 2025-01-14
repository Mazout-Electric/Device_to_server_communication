/*
 * GPS.c
 *
 *  Created on: Jan 9, 2025
 *      Author: pirda
 */

#include "GPS.h"

#include "cmsis_os.h"

extern osMutexId uart_lockHandle;

char txBufferG[RX_BUFFER_SIZE];        // Buffer for sending AT commands
char rxBufferG[RX_BUFFER_SIZE];        // Buffer for receiving AT responses
char checkBufferG[RX_BUFFER_SIZE];

extern serverProperties serverAttributes;

extern UART_HandleTypeDef huart1;

void gps(void) {
	char *gpsString;
	osMutexAcquire(uart_lockHandle, osWaitForever);

	strcpy(txBufferG, CMD_GPS_DATA);

	while (!strstr((char *)checkBufferG, GPS_ACK)) {

	HAL_UART_Transmit(&huart1, (uint8_t *)txBufferG, strlen(txBufferG), UART_TIMEOUT);
	osDelay(10);  // Wait for the response
	}

	gpsString = strstr((char *)checkBufferG, GPS_ACK);

	memset(txBufferG, NULL_ , sizeof(txBufferG));

	// Move pointer past "+CGPSINFO:"
	gpsString += 10;

	// Example response: 3113.343286,N,12121.234064,E,...
	char latitude[16] = {0};
	char longitude[16] = {0};

	// Extract latitude and longitude strings
	char *token = strtok(gpsString, ".");
	if (token != NULL) strncpy(latitude, token, sizeof(latitude) - 1);
	token = strtok(NULL, ","); // Skip N/S indicator
	token = strtok(NULL, ",");
	if (token != NULL) strncpy(longitude, token, sizeof(longitude) - 1);

	// Format and store in gpsData
	sprintf((char *)serverAttributes.gpsData, "%s,%s", latitude, longitude);

	osMutexRelease(uart_lockHandle);
}
