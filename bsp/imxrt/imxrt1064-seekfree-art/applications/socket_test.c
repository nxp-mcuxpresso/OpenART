#include <rtdevice.h>
#include "drv_gpio.h"
#include "core_cm7.h"
#include <rtthread.h>

#include <py/compile.h>
#include <py/runtime.h>
#include <py/repl.h>
#include <py/gc.h>
#include <py/mperrno.h>
#include <py/stackctrl.h>
#include <py/frozenmod.h>
#include <lib/mp-readline/readline.h>
#include <lib/utils/pyexec.h>
#include "mpgetcharport.h"
#include "mpputsnport.h"
#include "dfs_fs.h"
#include "fsl_sai.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_sai_edma.h"
#include <dfs_posix.h>
#include <sal_socket.h>
#include <wlan_mgnt.h>
#include <wlan_prot.h>
#include <wlan_cfg.h>
#include "netdev.h"

#define SERVER_HOST   ((uint8_t [4]){192, 168, 191, 3})
#define SERVER_PORT   0x1234

static char send_buffer[1024];
static struct sockaddr server_addr;
static int sockfd = -1;
int socket_connect()
{
	int re_try = 10;
	while(re_try -- >0)
	{
		if (connect(sockfd,&server_addr,sizeof(struct sockaddr_in)) >0)
		{
			return 1;
		}
		rt_thread_mdelay(rt_tick_from_millisecond(1000));
	}
	
	return -1;
}

int socket_send(uint8_t *buffer, int len)
{
	int re_try = 5;
	int sent = 0;
	int st;
	int old_sockfd;
	int total = len;
	while(re_try--)
	{
		sent = send(sockfd,buffer,len,0);
		if(sent <= 0)
		{
			old_sockfd = sockfd;
			sockfd = socket(AF_INET, SOCK_STREAM, 0);
			rt_kprintf("sent %d new socket %d,old socket%d\r\n",sent,sockfd,old_sockfd);
			closesocket(old_sockfd);
			rt_thread_mdelay(rt_tick_from_millisecond(1000));
			st = socket_connect();
			rt_kprintf("connect return:%d re_try=%d\r\n",st,re_try);
		}else if (sent > 0 && sent < len)
		{
			buffer += sent;
			len = len - sent;
			rt_kprintf("sent :%d\r\n",sent);
		}
		else
		{
			return total;
		}
	}
	
	return sent;
}

void socket_test_tcp_send(int argc , void **argv)
{
	int count = 10000;
	
	
	char *ssid;
	char *key;
	char *dev_name = "8266";
	int sent_total = 0;

#if 0	
	if(argc != 4)
	{
		rt_kprintf("socket_test ap password count");
		return;
	}
	ssid = argv[1];
	key = argv[2];
	count = atoi(argv[3]);
//start as ap	
	if (rt_wlan_set_mode(dev_name,RT_WLAN_STATION) != RT_EOK)
		return;
	
	if (rt_wlan_connect(ssid, key) != RT_EOK)
	{
		rt_kprintf("Connect %s failed\r\n",ssid);
		return;
	}
	rt_kprintf("Connect %s success\r\n",ssid);
#else
	if (!rt_wlan_is_connected())
	{
		rt_kprintf("wifi not connected\r\n");
		return;
	}
	
	uint32_t ip_addr;
	uint16_t port;
	if (argc != 4)
	{
		rt_kprintf("socket_test_tcp_send 192.169.1.1 port count\r\n\r\n");
		return;
	}
	ip_addr = nmi_inet_addr(argv[1]);
	port = atoi(argv[2]);
	count = atoi(argv[3]);
#endif
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        rt_kprintf("Socket error\n");
        goto clean;
    }

	server_addr.sa_family = AF_INET;
	server_addr.sa_data[0] = (uint8_t)(port >> 8); 
    server_addr.sa_data[1] = (uint8_t)(port); 
    memcpy(&server_addr.sa_data[2],&ip_addr,4); 
	server_addr.sa_len = 6;
	
	if (socket_connect() <0)
	{
		rt_kprintf("Socket connect failed\r\n");
		goto clean;
	}
	
	rt_kprintf("Socket connect:%s:%d\r\n Sending Test %d...\r\n",argv[1],port,count);
	
	for (int i=0;i<count;i++)
	{
		int sent;
		memset((uint32_t*)send_buffer,i,1024/4);
		if((sent = socket_send(send_buffer,1024)) != 1024)
		{
			rt_kprintf("socket send error %d, try to restart socket\r\n",sent);
			rt_kprintf("already sent %dKB\r\n",sent_total/1024);
			goto clean;
		}
		sent_total += sent;
	}
	
clean:	
	rt_kprintf("Socket Send %d KB\r\n",sent_total/1024);
	closesocket(sockfd);
	
}

MSH_CMD_EXPORT(socket_test_tcp_send, socket_test ap_name password count)



void socket_test_tcp_rev(int argc , void **argv)
{
	int count = 10000;

	int rev_total = 0;

#if 0	
	if(argc != 4)
	{
		rt_kprintf("socket_test ap password count");
		return;
	}
	ssid = argv[1];
	key = argv[2];
	count = atoi(argv[3]);
//start as ap	
	if (rt_wlan_set_mode(dev_name,RT_WLAN_STATION) != RT_EOK)
		return;
	
	if (rt_wlan_connect(ssid, key) != RT_EOK)
	{
		rt_kprintf("Connect %s failed\r\n",ssid);
		return;
	}
	rt_kprintf("Connect %s success\r\n",ssid);
#else
	if (!rt_wlan_is_connected())
	{
		rt_kprintf("wifi not connected\r\n");
		return;
	}
	
	uint16_t port;
	if (argc != 3)
	{
		rt_kprintf("socket_test_tcp_rev port count\r\n\r\n");
		return;
	}
	port = atoi(argv[1]);
	count = atoi(argv[2]);
#endif
	int clientfd = 0;
	struct sockaddr client_addr;
	int client_len;
	int sent = 0;
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        rt_kprintf("Socket error\n");
        goto clean;
    }
	struct netdev *dev = netdev_get_by_name("8266");
	if(dev == NULL)
		goto clean;
	server_addr.sa_family = AF_INET;
	server_addr.sa_data[0] = (uint8_t)(port >> 8); 
    server_addr.sa_data[1] = (uint8_t)(port); 
    memcpy(&server_addr.sa_data[2],&dev->ip_addr.addr,4); 
	server_addr.sa_len = 6;
	
	if(bind(sockfd,&server_addr,sizeof(struct sockaddr_in)) < 0)
	{
		rt_kprintf("Socket bind failed\r\n");
		goto clean;
	}
	
	if(listen(sockfd,10)<0)
	{
		rt_kprintf("Socket listen failed\r\n");
	}
	
	if ((clientfd = accept(sockfd,&client_addr,&client_len)) <0)
	{
		rt_kprintf("Socket accept failed\r\n");
		goto clean;
	}
	
	rt_kprintf("Socket accept from %d.%d.%d.%d:%d\r\n Recieving Test %d...\r\n",client_addr.sa_data[2],client_addr.sa_data[3],
	client_addr.sa_data[4],client_addr.sa_data[5],client_addr.sa_data[0]<<8|client_addr.sa_data[1],count);
	
	for (int i=0;i<count;i++)
	{
		int re;

		if((re = recv(clientfd,send_buffer,1024)) < 0)
		{
			rt_kprintf("socket rev error %d, try to restart socket\r\n",re);
			rt_kprintf("already rev %dKB\r\n",rev_total);
			goto clean;
		}
		
		if(re > 0)
			sent += send(clientfd,send_buffer,re,0);
		rev_total += re;
	}
	
clean:	
	rt_kprintf("Socket rev %d sent %d\r\n",rev_total,sent);
	closesocket(sockfd);
	
}

MSH_CMD_EXPORT(socket_test_tcp_rev, socket_test ap_name password count)


void socket_test_udp_send(int argc , void **argv)
{
	int count = 10000;
	
	
	char *ssid;
	char *key;
	char *dev_name = "8266";
	int sent_total = 0;

#if 0	
	if(argc != 4)
	{
		rt_kprintf("socket_test ap password count");
		return;
	}
	ssid = argv[1];
	key = argv[2];
	count = atoi(argv[3]);
//start as ap	
	if (rt_wlan_set_mode(dev_name,RT_WLAN_STATION) != RT_EOK)
		return;
	
	if (rt_wlan_connect(ssid, key) != RT_EOK)
	{
		rt_kprintf("Connect %s failed\r\n",ssid);
		return;
	}
	rt_kprintf("Connect %s success\r\n",ssid);
#else
	if (!rt_wlan_is_connected())
	{
		rt_kprintf("wifi not connected\r\n");
		return;
	}
	
	uint32_t ip_addr;
	uint16_t port;
	if (argc != 4)
	{
		rt_kprintf("socket_test_tcp_send 192.169.1.1 port count\r\n\r\n");
		return;
	}
	ip_addr = nmi_inet_addr(argv[1]);
	port = atoi(argv[2]);
	count = atoi(argv[3]);
#endif
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        rt_kprintf("Socket error\n");
        goto clean;
    }

	server_addr.sa_family = AF_INET;
	server_addr.sa_data[0] = (uint8_t)(port >> 8); 
    server_addr.sa_data[1] = (uint8_t)(port); 
    memcpy(&server_addr.sa_data[2],&ip_addr,4); 
	server_addr.sa_len = 6;
	
	
	for (int i=0;i<count;i++)
	{
		int sent;
		memset((uint32_t*)send_buffer,i,1024/4);
		if((sent = sendto(sockfd,send_buffer,1024,0,&server_addr,sizeof(struct sockaddr_in))) != 1024)
		{
			rt_kprintf("socket udp send error %d, try to restart socket\r\n",sent);
			rt_kprintf("already sent %dKB\r\n",sent_total/1024);
			goto clean;
		}
		sent_total += sent;
	}
	
clean:	
	rt_kprintf("Socket Send %d KB\r\n",sent_total/1024);
	closesocket(sockfd);
	
}

MSH_CMD_EXPORT(socket_test_udp_send, socket_test ap_name password count)

void socket_test_udp_rev(int argc , void **argv)
{
	int count = 10000;
	
	
	char *ssid;
	char *key;
	char *dev_name = "8266";
	int rev_total = 0;

#if 0	
	if(argc != 4)
	{
		rt_kprintf("socket_test ap password count");
		return;
	}
	ssid = argv[1];
	key = argv[2];
	count = atoi(argv[3]);
//start as ap	
	if (rt_wlan_set_mode(dev_name,RT_WLAN_STATION) != RT_EOK)
		return;
	
	if (rt_wlan_connect(ssid, key) != RT_EOK)
	{
		rt_kprintf("Connect %s failed\r\n",ssid);
		return;
	}
	rt_kprintf("Connect %s success\r\n",ssid);
#else
	if (!rt_wlan_is_connected())
	{
		rt_kprintf("wifi not connected\r\n");
		return;
	}
	
	uint8_t ip_addr[4] = {192,168,1.1};
	uint16_t port;
	if (argc != 3)
	{
		rt_kprintf("socket_test_tcp_send 192.169.1.1 port count\r\n\r\n");
		return;
	}
	port = atoi(argv[1]);
	count = atoi(argv[2]);
#endif
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        rt_kprintf("Socket error\n");
        goto clean;
    }

	struct netdev *dev = netdev_get_by_name("8266");
	if(dev == NULL)
		goto clean;
	server_addr.sa_family = AF_INET;
	server_addr.sa_data[0] = (uint8_t)(port >> 8); 
    server_addr.sa_data[1] = (uint8_t)(port); 
    memcpy(&server_addr.sa_data[2],&dev->ip_addr.addr,4); 
	server_addr.sa_len = 6;
	
	if(bind(sockfd,&server_addr,sizeof(struct sockaddr_in))<0)
	{
		rt_kprintf("Socket listen failed\r\n");
	}
	
	static struct sockaddr peer_addr;
	for (int i=0;i<count;i++)
	{
		int re;

		if((re = recvfrom(sockfd,send_buffer,1024,0,&peer_addr,sizeof(struct sockaddr_in))) < 0)
		{
			rt_kprintf("socket rev error %d, try to restart socket\r\n",re);
			rt_kprintf("already rev %d\r\n",rev_total);
			goto clean;
		}
		rev_total += re;
	}
	
clean:	
	rt_kprintf("Socket rev %d from %d.%d.%d.%d:%d\r\n",rev_total,peer_addr.sa_data[2],peer_addr.sa_data[3],
	peer_addr.sa_data[4],peer_addr.sa_data[5],peer_addr.sa_data[0]<<8|peer_addr.sa_data[1]);
	closesocket(sockfd);
	
}

MSH_CMD_EXPORT(socket_test_udp_rev, socket_test ap_name password count)


