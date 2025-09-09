#include "tcpclient.h"
#include "lwip/sockets.h"
#include "ctype.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/netdb.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>

int client_socket = -1;

// 定义一个标志变量来控制重连次数
int reconnect_attempts = 0;

void myDelay(int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

bool is_connected = false;

bool isConnected()
{
    return is_connected;
}

void tcp_client_init(void)
{
    struct sockaddr_in serverAddr;
    int enable = 1;

    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket < 0)
    {
        printf("Failed to create socket\n");
    }
    // 设置 socket 选项
    setsockopt(client_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

    // 设置读取超时
    struct timeval timeout;
    timeout.tv_sec = 5; // 5秒超时
    timeout.tv_usec = 0;
    setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(TCP_CLIENT_PORT);
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP);
    int ret = connect(client_socket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    // 连接到服务器
    if (ret < 0)
    {
        printf("Failed to connect to server\n");
        lwip_close(client_socket);
        client_socket = -1;
        is_connected = false;
    }
    else
    {
        printf("TCP connection established\n");
        is_connected = true;
        reconnect_attempts = 0; // 重置重连次数
    }
}

// 读取数据
void tcp_read_data(void)
{
    char buffer[1024];
    ssize_t bytes_read;

    // 读取数据
    bytes_read = lwip_recv(client_socket, buffer, sizeof(buffer) - 1, 0);
    if (bytes_read > 0)
    {
        buffer[bytes_read] = '\0';
        printf("Received data: %s\n", buffer);
    }
    else if (bytes_read == 0)
    {
        printf("Connection closed by server\n");
        lwip_close(client_socket);
        client_socket = -1;
        is_connected = false;
    }
    else
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            printf("Read timed out\n");
        }
        else
        {
            printf("Error reading data: %d\n", errno);
            lwip_close(client_socket);
            client_socket = -1;
            is_connected = false;
        }
    }
}

// 发送数据
void tcp_send_data(const char *data)
{
    if (is_connected)
    {
        ssize_t bytes_sent = lwip_send(client_socket, data, strlen(data), 0);
        if (bytes_sent < 0)
        {
            printf("Error sending data: %d\n", errno);
            lwip_close(client_socket);
            client_socket = -1;
            is_connected = false;
        }
        else
        {
            printf("Sent data: %s\n", data);
        }
    }
    else
    {
        printf("Not connected, cannot send data\n");
    }
}
