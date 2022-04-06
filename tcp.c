/*
 * tcp.c
 * Han Le
 */

#include <stdio.h>
#include "tcp.h"

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

// TCP Message
#define TCPSYN    1
#define TCPSYNACK 2
#define TCPACK    3
#define TCPFIN    4

// TCP State
#define TCP_CLOSED       0
#define TCP_LISTEN       1
#define TCP_SYN_SEND     2
#define TCP_SYN_REVEIVED 3
#define TCP_ESTABLISHED  4
#define TCP_FINWAIT_1    5
#define TCP_FINWAIT_2    6
#define TCP_TIME_WAIT    7
#define TCP_CLOSE_WAIT   8
#define TCP_LAST_ACK     9

// ------------------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------------------

uint8_t tcpState = TCP_CLOSED;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// State functions
void tcpSetState(uint8_t state)
{
    tcpState = state;
}

uint8_t tcpGetState()
{
    return tcpState;
}

// Send TCP message
void tcpSendMessage(etherHeader *ether, uint8_t type)
{
    uint32_t sum;
    uint8_t i;
    uint16_t tmp16;
    uint8_t mac[6];

    // Ether frame
    etherGetMacAddress(mac);
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = mac[i];
    }
    ether->frameType = htons(0x800);

    // IP header
    ipHeader* ip = (ipHeader*)ether->data;
    ip->revSize = 0x45;
    ip->typeOfService = 0;
    ip->id = 0;
    ip->flagsAndOffset = 0;
    ip->ttl = 128;
    ip->protocol = 6;
    ip->headerChecksum = 0;
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = ip->sourceIp[i];
    }
    etherGetIpAddress(ip->sourceIp);

    // TCP
    tcpHeader* tcp = (tcpHeader*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    //tmp16 =
    tcp->sourcePort = tcp->destPort;



}
