#ifndef _TCPCLIENT_H_
#define _TCPCLIENT_H_

#include <stdbool.h>

#define SERVER_IP "192.168.137.200"

#define TCP_CLIENT_PORT 8080

#define MAX_RECONNECT_ATTEMPTS 5

extern int reconnect_attempts;

void tcp_client_init(void);

bool isConnected();

void tcp_read_data(void);

void tcp_send_data(const char *data);

#endif
