/*
 * tcp.h
 * Han Le
 */

#ifndef TCP_H_
#define TCP_H_

#include <stdint.h>
#include <stdbool.h>
#include "eth0.h"
#include "uart0.h"
#include "timer.h"

typedef struct _socket
{
  uint8_t remoteIpAddress[4];
  uint8_t remoteHwAddress[6];
  uint16_t remotePort;
  uint32_t sequenceNumber;
} socket;

// TCP Message
#define TCPFIN    0x01
#define TCPSYN    0x02
#define TCPACK    0x10
#define TCPRST    0x04

// TCP State
#define TCP_CLOSED       0
#define TCP_LISTEN       1
#define TCP_SYN_SENT     2
#define TCP_SYN_RECEIVED 3
#define TCP_ESTABLISHED  4
#define TCP_FINWAIT_1    5
#define TCP_FINWAIT_2    6
#define TCP_TIME_WAIT    7
#define TCP_CLOSE_WAIT   8
#define TCP_LAST_ACK     9

void restrictPort(uint16_t port);
void tcpSetState(uint8_t state);
uint8_t tcpGetState();
bool tcpIsPortOpen(etherHeader *ether);
bool tcpIsSyn(etherHeader *ether);
bool tcpIsAck(etherHeader *ether);
void tcpSendResponse(etherHeader *ether, socket *s, uint8_t flags);


#endif /* TCP_H_ */
