/**
 * Kondo B3M 3.0 Library (Header)
 *
 * Copyright 2016 - Yasuo Hayashibara (yasuo@hayashibara.net)
 * Chiba Institute of Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef B3M_H_
#define B3M_H_

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <sys/time.h>
#include "ftdi.h"

// some options
#define B3M_BAUD 1500000
#define B3M_USB_VID 0x165c
#define B3M_USB_PID 0x0009  // setting for RS485USB
#define B3M_RX_TIMEOUT 1000000
#define B3M_POS_TIMEOUT 2000
#define B3M_GET_TIMEOUT 2000
#define B3M_SET_TIMEOUT 2000
#define B3M_ID_TIMEOUT 2000

// b3m commands
#define B3M_CMD_LOAD		0x01
#define B3M_CMD_SAVE		0x02
#define B3M_CMD_READ		0x03
#define B3M_CMD_WRITE		0x04
#define B3M_CMD_POSITION	0x06
#define B3M_CMD_RESET		0x05

#define B3M_RETURN_ERROR_STATUS		0x000
#define B3M_RETURN_SYSTEM_STATUS	0x001
#define B3M_RETURN_MOTOR_STATUS		0x010
#define B3M_RETURN_UART_STATUS		0x011
#define B3M_RETURN_COMMAND_STATUS	0x100

#define B3M_SC_EEPROM  0
#define B3M_SC_STRETCH 1
#define B3M_SC_SPEED   2
#define B3M_SC_CURRENT 3
#define B3M_SC_TEMPERATURE 4
#define B3M_SC_READ 0
#define B3M_SC_WRITE 1

#ifndef __UCHAR__
#define __UCHAR__
typedef unsigned char UCHAR;
#endif 

#ifndef __UINT__
#define __UINT__
typedef unsigned int UINT;
#endif

// instance data for kondo library
typedef struct
{
	struct ftdi_context ftdic; // ftdi context
	UCHAR swap[128]; // swap space output
	char error[128]; // error messages
	UCHAR debug; // whether to print debug info
} B3MData;

// low level comms
int b3m_init(B3MData * r, int product_id);
int b3m_close(B3MData * r);
int b3m_write(B3MData * r, int n);
int b3m_read(B3MData * r, int n);
int b3m_read_timeout(B3MData * r, int n, long timeout);
int b3m_purge(B3MData * r);
int b3m_trx(B3MData * r, UINT bytes_out, UINT bytes_in);
int b3m_trx_timeout(B3MData * r, UINT bytes_out, UINT bytes_in, long timeout);

// position commands
int b3m_pos(B3MData * r, UINT id, UINT pos);
int b3m_hold(B3MData * r, UINT id);
int b3m_free(B3MData * r, UINT id);

// servo setting commands
int b3m_get_stretch(B3MData * r, UINT id);
int b3m_get_speed(B3MData * r, UINT id);
int b3m_get_current(B3MData * r, UINT id);
int b3m_set_stretch(B3MData * r, UINT id, UCHAR stretch);
int b3m_set_speed(B3MData * r, UINT id, UCHAR speed);
int b3m_set_current_limit(B3MData * r, UINT id, UCHAR curlim);
int b3m_set_temperature_limit(B3MData * r, UINT id, UCHAR templim);

// set servo id (for use when 1 servo is connected)
int b3m_get_id(B3MData * r);
int b3m_set_id(B3MData * r, UINT id);

// low level commands (be careful!)
//int b3m_get_eeprom(B3MData * r, UCHAR ** dest);
//int b3m_set_eeprom(B3MData * r, UCHAR ** eeprom);
//int b3m_get_mem_bytes(B3MData * r, UCHAR addr, UCHAR num_bytes, UCHAR ** dst);
//int b3m_set_mem_bytes(B3MData * r, UCHAR addr, UCHAR num_bytes, UCHAR ** src);

#endif /* B3M_H_ */
