/*
 * AT_command.h
 *
 *  Created on: Jan 9, 2025
 *      Author: pirda
 */

#ifndef INC_AT_COMMAND_H_
#define INC_AT_COMMAND_H_

#include <string.h>
#include "stm32h7xx_hal.h"

#define NULL_					'\0'
#define SERVER_ACK				">"
#define GPS_ACK					"+CGPSINFO:"

#define CMD_ECHO_OFF            "ATE0\r\n"
#define CMD_CHECK_SIM           "AT+CPIN?\r\n"
#define CMD_NETWORK_REG         "AT+CREG=2\r\n"
#define CMD_ACTIVATE_PDP        "AT+CGACT=1,1\r\n"
#define CMD_SET_APN             "AT+CGDCONT=1,\"IP\",\"airtelgprs.com\"\r\n"
#define CMD_GPS_MODE            "AT+CGPS=1\r\n"
#define CMD_NETOPEN             "AT+NETOPEN\r\n"
#define RESPONSE_NETOPEN_OK     "+NETOPEN: 0"

#define CMD_OPEN_SOCKET_FORMAT  "AT+CIPOPEN=%d,\"TCP\",\"%s\",%d\r\n"
#define RESPONSE_SOCKET_OPEN_OK "+CIPOPEN: 0,0"

#define CMD_SEND_DATA_FORMAT			"AT+CIPSEND=%d,%d\r\n"
#define CMD_RECEIVE_DATA_FORMAT		"AT+CIPRXGET=3,%d,%d\r\n"

#define CMD_GPS_DATA			"AT+CGPSINFO\r\n"


#endif /* INC_AT_COMMAND_H_ */
