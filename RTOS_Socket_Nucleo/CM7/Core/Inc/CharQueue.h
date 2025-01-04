/*
 * CharQueue.h
 *
 *  Created on: Dec 13, 2024
 *      Author: pirda
 */

#ifndef INC_CHARQUEUE_H_
#define INC_CHARQUEUE_H_

#include <stdbool.h>

#include "SimCom.h"

#define CHARQUEUE_INDEX_TYPE SIMCOM_LENGTH_TYPE

typedef struct
{
	char *data;
	CHARQUEUE_INDEX_TYPE front, rear;
	CHARQUEUE_INDEX_TYPE count, max_size;
}	char_queue;

bool init_char_queue(char_queue *p, char buf[], CHARQUEUE_INDEX_TYPE max_size);
bool in_char_queue(char_queue *p, char c);
bool out_char_queue(char_queue *p, char *c);

#endif /* INC_CHARQUEUE_H_ */
