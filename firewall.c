/*
 * firewall.c
 */

#include <stdio.h>
#include <stdlib.h>
#include "firewall.h"
#include "eth0.h"

bool checkPacket(etherHeader* ether, firewall *f)
{
    bool ok = true;
    uint8_t i = 0;
    uint8_t ipAddr[4], dnsAddr[4];
    uint8_t macAddr[6];

    ipHeader* ip = (ipHeader*)ether->data;

    etherGetMacAddress(macAddr);
    etherGetIpAddress(ipAddr);
    etherGetIpDnsAddress(dnsAddr);

    while (ok & (i < IP_ADD_LENGTH))
    {
        ok |= (f->ipAddress[i] == ipAddr[i]);
        i++;
    }
    while (ok & (i < HW_ADD_LENGTH))
    {
        ok |= (f->ipAddress[i] == macAddr[i]);
        i++;
    }
    while (ok & (i < IP_ADD_LENGTH))
    {
        ok |= (f->dnsAddress[i] == dnsAddr[i]);
        i++;
    }
    // deny if udp
    if((ip->protocol == 17) & ((f->udpTcpFlag & 0x02) >> 1))
    {
        ok |= 0;
    }
    else if ((ip->protocol == 17) & !((f->udpTcpFlag & 0x02) >> 1))
    {

    }
    // deny if tcp
    if(f->udpTcpFlag & 0x01)
    {
        ok |= (ip->protocol == 6);
    }

    return ~ok;
}

//bool enableFirewall
