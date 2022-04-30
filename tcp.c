/*
 * tcp.c
 * Han Le
 */

#include <stdio.h>
#include <stdlib.h>
#include "tcp.h"
#include "wait.h"
#include "uart0.h"

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------------------

uint8_t tcpState = TCP_CLOSED;
uint16_t sourcePort = 0;
uint32_t sequenceNum = 0, ackNum = 0, mss = 0;

enum tcpFlags {FIN = 0, SYN, RST, PSH, ACK};
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void printBinary(uint16_t num)
{
      uint16_t i = 0x8000;
      while(i)
      {
          putcUart0((num & i) ? '1' : '0');
          i >>= 1;
      }
}

// State functions
void tcpSetState(uint8_t state)
{
    tcpState = state;
}

uint8_t tcpGetState()
{
    return tcpState;
}

bool tcpIsPortOpen(etherHeader *ether)
{
    ipHeader* ip = (ipHeader*)ether->data;
    tcpHeader* tcp = (tcpHeader*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    if(tcp->destPort == 443 || tcp->destPort == 80)
    {
        tcpSetState(TCP_LISTEN);
        return 1;
    }
    return 0;
}

bool tcpIsSyn(etherHeader *ether)
{
    ipHeader* ip = (ipHeader*)ether->data;
    tcpHeader* tcp = (tcpHeader*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    if((htons(tcp->offsetFields) & 0xFF) == (TCPSYN))
    {
        return 1;
    }
    return 0;
}

bool tcpIsAck(etherHeader *ether)
{
    ipHeader* ip = (ipHeader*)ether->data;
    tcpHeader* tcp = (tcpHeader*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    if((htons(tcp->offsetFields) & 0xFF) == (TCPACK))
    {
        return 1;
    }
    return 0;
}

// Send TCP message
void tcpSendResponse(etherHeader *ether, socket *s, uint8_t flags)
{
    uint32_t sum;
    uint8_t i, tmp8;
    uint16_t tmp16, tcpLength = 0, tcpSize = 0;
    uint8_t mac[6];

    // Ether frame
    etherGetMacAddress(mac);
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        tmp8 = ether->destAddress[i];
        ether->destAddress[i] = s->remoteHwAddress[i];
        ether->sourceAddress[i] = tmp8; //mac[i];
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
        tmp8 = ip->destIp[i];
        ip->destIp[i] = s->remoteIpAddress[i];
        ip->sourceIp[i] = tmp8;
    }
    //etherGetIpAddress(ip->sourceIp);

    // TCP
    uint8_t ipHeaderLength = (ip->revSize & 0xF) * 4;
    tcpHeader *tcp = (tcpHeader*)((uint8_t*)ip + ipHeaderLength);
    tmp16 = tcp->destPort;
    tcp->destPort = s->remotePort;
    tcp->sourcePort = tmp16;
    tcp->windowSize = htons(256);
    tcp->urgentPointer = htons(0);
    tcp->sequenceNumber = htonl(s->sequenceNumber);
    tcp->acknowledgementNumber = htonl(tcp->sequenceNumber);

    if(flags == TCPACK | TCPSYN)
    {
        tcp->offsetFields = htons((sizeof(tcpHeader) + 12)/4 << 12) | htons(TCPACK | TCPSYN);
        tcpSize = 20;
    }
    if(flags == TCPACK)
    {
        tcp->offsetFields = htons(sizeof(tcpHeader)/4 << 12) | htons(TCPACK);
        tcpSize = 20;
    }
    if(flags == TCPFIN)
    {
        tcp->offsetFields = htons(sizeof(tcpHeader)/4 << 12) | htons(TCPFIN);
        tcpSize = 20;
    }
    //sequenceNum++;
    tcpLength = htons(tcpSize);
    // 32-bit sum over ip header
    ip->length = htons(ipHeaderLength + tcpSize);
    etherCalcIpChecksum(ip);
    // 32-bit sum over pseudo-header
    sum = 0;
    etherSumWords(ip->sourceIp, 8, &sum);
    tmp16 = ip->protocol;
    sum += (tmp16 & 0xff) << 8;
    etherSumWords(&tcpLength, 2, &sum);
    // add tcp header
    tcp->checksum = 0;
    etherSumWords(tcp, tcpSize, &sum);
    tcp->checksum = getEtherChecksum(sum);

    // send packet with size = ether + tcp length + ip header
    etherPutPacket(ether, sizeof(etherHeader) + ipHeaderLength + tcpSize);
}
/*
void etherSendTcpResponse(etherHeader *ether)
{

    if(flags == TCPSYN)
    {
        tcp->offsetFields = htons((sizeof(tcpHeader) + 12)/4 << 12) | htons(1 << SYN);
        printBinary(tcp->offsetFields); putcUart0('\n');tcp->data[0] = 2;
        printBinary(htons(tcp->offsetFields)); putcUart0('\n');
        tcp->data[1] = 4;
        tcp->data[2] = 0x05;
        tcp->data[3] = 0xB4;
        tcp->data[4] = 1;
        tcp->data[5] = 3;
        tcp->data[6] = 3;
        tcp->data[7] = 8;
        tcp->data[8] = 1;
        tcp->data[9] = 1;
        tcp->data[10] = 4;
        tcp->data[11] = 2;
        tcpSize = 32;
    }

    ipHeader* ip = (ipHeader*)ether->data;
    tcpHeader* tcp = (tcpHeader*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    if(tcpGetState() == TCP_CLOSED)
    {
        srand(time(NULL));
        sequenceNum = 0;//rand();
        sourcePort = (rand() % 16383) + 49152;
        tcpSendMessage(ether, TCPSYN);
        tcpSetState(TCP_SYN_SENT);
    }
    printBinary(tcp->offsetFields); putcUart0('\n');
    printBinary(htons(tcp->offsetFields)); putcUart0('\n');
  if(tcpGetState() == TCP_ESTABLISHED)
    {
        tcpSendMessage(ether, TCPFIN);
        tcpSetState(TCP_FINWAIT_1);
    }
    if(tcpGetState() == TCP_TIME_WAIT)
    {
        waitMicrosecond(10000);
        tcpSetState(TCP_CLOSED);
    }
    if((htons(tcp->offsetFields) & 0x012) == (1<<SYN | 1<<ACK) & tcpGetState() == TCP_SYN_SENT)
    {
        ackNum = htonl(tcp->sequenceNumber) + 1;
        tcpSendMessage(ether, TCPACK);
        tcpSetState(TCP_ESTABLISHED);
    }
    if((htons(tcp->offsetFields) & 0x010) == 1<<ACK & tcpGetState() == TCP_FINWAIT_1)
    {
        tcpSetState(TCP_FINWAIT_2);
    }
    if((htons(tcp->offsetFields) & 0x011) == (1<<FIN | 1<<ACK) & tcpGetState() == TCP_FINWAIT_2)
    {
        ackNum++;
        tcpSendMessage(ether, TCPACK);
        tcpSetState(TCP_TIME_WAIT);
    }
    if((htons(tcp->offsetFields) & 0x014) == (1<<RST | 1<<ACK))
    {
        ackNum = htonl(tcp->sequenceNumber) + 1;
        tcpSetState(TCP_CLOSED);
    }
}*/



