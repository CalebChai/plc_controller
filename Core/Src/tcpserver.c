#include "tcpserver.h"
#include "lwip/netif.h"
#include "lwip/ip.h"
#include "lwip/tcp.h"
#include "lwip/init.h"
#include "netif/etharp.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include <stdio.h>
#include <string.h>

static err_t tcpecho_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p,
                          err_t err)
{ // 对应接收数据连接的控制块   接收到的数据
    if (p != NULL)
    {
        /* 更新窗口*/
        tcp_recved(tpcb, p->tot_len); // 读取数据的控制块   得到所有数据的长度

        /* 返回接收到的数据*/
        // tcp_write(tpcb, p->payload, p->tot_len, 1);

        uint8_t send_buf1[] = "[服务端] 收到消息:[";
        uint8_t send_buf2[] = "]\n";
        tcp_write(tpcb, send_buf1, sizeof(send_buf1), 1);
        tcp_write(tpcb, p->payload, p->tot_len, 1);
        tcp_write(tpcb, send_buf2, sizeof(send_buf2), 1);

        memset(p->payload, 0, p->tot_len);
        pbuf_free(p);
    }
    else if (err == ERR_OK) // 检测到对方主动关闭连接时，也会调用recv函数，此时p为空
    {
        return tcp_close(tpcb);
    }
    return ERR_OK;
}

static err_t tcpecho_accept(void *arg, struct tcp_pcb *newpcb, err_t err) // 由于这个函数是*tcp_accept_fn类型的
// 形参的数量和类型必须一致
{

    tcp_recv(newpcb, tcpecho_recv); // 当收到数据时，回调用户自己写的tcpecho_recv
    return ERR_OK;
}

void TCP_Echo_Init(void)
{
    struct tcp_pcb *server_pcb = NULL;
    server_pcb = tcp_new();
    printf("[服务端] 创建了一个控制块\n");
    /* 绑定TCP控制块 */
    tcp_bind(server_pcb, IP_ADDR_ANY, TCP_ECHO_PORT);
    printf("[服务端] 已经绑定一个控制块\n");
    /* 进入监听状态 */
    server_pcb = tcp_listen(server_pcb);
    printf("[服务端] 进入监听状态\n");

    tcp_accept(server_pcb, tcpecho_accept);
}
