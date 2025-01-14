/*
 * Network.c
 *
 *  Created on: Jan 9, 2025
 *      Author: pirda
 */

#include "Network.h"

#include "cmsis_os.h"

extern osMutexId uart_lockHandle;

extern UART_HandleTypeDef huart1;

char txBufferN[RX_BUFFER_SIZE];        // Buffer for sending AT commands
char rxBufferN[RX_BUFFER_SIZE];        // Buffer for receiving AT responses
char checkBufferN[RX_BUFFER_SIZE];

uint8_t propertyIndex = 0;

void NetworkInit() {

	strcpy(txBufferN, CMD_ECHO_OFF);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBufferN, strlen(txBufferN), UART_TIMEOUT);
	memset(txBufferN, NULL_ , sizeof(txBufferN));

	memset(rxBufferN, NULL_ , sizeof(rxBufferN));
	HAL_UART_Receive(&huart1, (uint8_t *)rxBufferN, sizeof(rxBufferN), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);

    // Check SIM is ready
    strcpy(txBufferN, CMD_CHECK_SIM);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBufferN, strlen(txBufferN), UART_TIMEOUT);
    memset(txBufferN, NULL_ , sizeof(txBufferN));

    memset(rxBufferN, NULL_ , sizeof(rxBufferN));
    HAL_UART_Receive(&huart1, (uint8_t *)rxBufferN, sizeof(rxBufferN), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);

    // Check network registration
    strcpy(txBufferN, CMD_NETWORK_REG);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBufferN, strlen(txBufferN), UART_TIMEOUT);
    memset(txBufferN, NULL_ , sizeof(txBufferN));

    memset(rxBufferN, NULL_ , sizeof(rxBufferN));
    HAL_UART_Receive(&huart1, (uint8_t *)rxBufferN, sizeof(rxBufferN), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);

    // PDP Context
	strcpy(txBufferN, CMD_ACTIVATE_PDP);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBufferN, strlen(txBufferN), UART_TIMEOUT);
	memset(txBufferN, NULL_ , sizeof(txBufferN));

	memset(rxBufferN, NULL_ , sizeof(rxBufferN));
	HAL_UART_Receive(&huart1, (uint8_t *)rxBufferN, sizeof(rxBufferN), UART_TIMEOUT);

    // Set APN
    strcpy(txBufferN, CMD_SET_APN);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBufferN, strlen(txBufferN), UART_TIMEOUT);
    memset(txBufferN, NULL_ , sizeof(txBufferN));

    memset(rxBufferN, NULL_ , sizeof(rxBufferN));
    HAL_UART_Receive(&huart1, (uint8_t *)rxBufferN, sizeof(rxBufferN), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);

    // Set GPS Mode
	strcpy(txBufferN, CMD_GPS_MODE);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBufferN, strlen(txBufferN), UART_TIMEOUT);
	memset(txBufferN, NULL_ , sizeof(txBufferN));

	memset(rxBufferN, NULL_ , sizeof(rxBufferN));
	HAL_UART_Receive(&huart1, (uint8_t *)rxBufferN, sizeof(rxBufferN), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);


    // Start TCP/IP service
    strcpy(txBufferN, CMD_NETOPEN);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBufferN, strlen(txBufferN), UART_TIMEOUT);
    memset(txBufferN, NULL_ , sizeof(txBufferN));

    memset(rxBufferN, NULL_ , sizeof(rxBufferN));
    HAL_UART_Receive(&huart1, (uint8_t *)rxBufferN, sizeof(rxBufferN), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);

    // Check response
    if (strstr(rxBufferN, RESPONSE_NETOPEN_OK) == NULL) {
        //Error_Handler();
    }
}

void OpenSocket() {
    sprintf(txBufferN, CMD_OPEN_SOCKET_FORMAT, SOCKET_INDEX, SERVER_IP, SERVER_PORT);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBufferN, strlen(txBufferN), UART_TIMEOUT);
    memset(txBufferN, NULL_ , sizeof(txBufferN));

    memset(rxBufferN, NULL_ , sizeof(rxBufferN));
    HAL_UART_Receive(&huart1, (uint8_t *)rxBufferN, sizeof(rxBufferN), UART_TIMEOUT);
    memset(rxBufferN, NULL_ , sizeof(rxBufferN));


    // Check response
    if (strstr(rxBufferN, RESPONSE_SOCKET_OPEN_OK) == NULL) {
        //Error_Handler();
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rxBufferN, RX_BUFFER_SIZE);   /////////Need to change as this will go on
    																		// to all the other files also

}


void SocketSendData(void) {
	uint8_t data[40];

	encodeServerData(propertyIndex, data);

	osMutexAcquire(uart_lockHandle, osWaitForever);

	sprintf(txBufferN, CMD_SEND_DATA_FORMAT, SOCKET_INDEX, sizeof(data));

    // Wait for `>` prompt
    while (!strstr((char *)checkBufferN, SERVER_ACK)) {

    	HAL_UART_Transmit(&huart1, (uint8_t *)txBufferN, strlen(txBufferN), UART_TIMEOUT);
    	osDelay(10);  // Wait for the response
    }
    memset(txBufferN, NULL_ , sizeof(txBufferN));

    // Send data
    HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(data), UART_TIMEOUT);
    memset(txBufferN, NULL_ , sizeof(txBufferN));

    osMutexRelease(uart_lockHandle);

    if(++propertyIndex > 8)	propertyIndex  = 0;
}

void SocketReceiveData(void) {
	int length = sizeof(rxBufferN);

	osMutexAcquire(uart_lockHandle, osWaitForever);

    sprintf(txBufferN, CMD_RECEIVE_DATA_FORMAT, SOCKET_INDEX, length);

    HAL_UART_Transmit(&huart1, (uint8_t *)txBufferN, strlen(txBufferN), UART_TIMEOUT);
    memset(txBufferN, NULL_ , sizeof(txBufferN));

    osMutexRelease(uart_lockHandle);

}
