/*
 * 	UDP Configuration and Implementation
 *  Author: GateKeeper
 *  Copyright (C) 2024
*/


#ifndef INC_UDPCLIENTRAW_H_
#define INC_UDPCLIENTRAW_H_

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "main.h"
#include "http_ssi.h"

void udpClient_connect(void);
void udpClient_send(void);
void receiver();

#define keyLen 128
#define numkeys 300

extern int UDP_init;
extern int client_connected;
extern int message_type;
extern char buffer[keyLen];
extern char message[keyLen];
extern const char *message_init;
extern int RFID_Received;
extern int counter_UDP;

#endif /* INC_UDPCLIENTRAW_H_ */
