// Han Le
// DHCP Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: -
// Target uC:       -
// System Clock:    -

// Hardware configuration:
// -

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdio.h>
#include "dhcp.h"
#include "wait.h"
#include "eeprom.h"

#define DHCPDISCOVER 1
#define DHCPOFFER    2
#define DHCPREQUEST  3
#define DHCPDECLINE  4
#define DHCPACK      5
#define DHCPNAK      6
#define DHCPRELEASE  7
#define DHCPINFORM   8

#define DHCP_DISABLED   0
#define DHCP_INIT       1
#define DHCP_SELECTING  2
#define DHCP_REQUESTING 3
#define DHCP_TESTING_IP 4
#define DHCP_BOUND      5
#define DHCP_RENEWING   6
#define DHCP_REBINDING  7
#define DHCP_INITREBOOT 8 // not used since ip not stored over reboot
#define DHCP_REBOOTING  9 // not used since ip not stored over reboot

// ------------------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------------------

uint8_t dhcpState = DHCP_DISABLED;

uint32_t  xid = 0;
uint32_t leaseSeconds = 0;
uint32_t leaseT1 = 0;
uint32_t leaseT2 = 0;

uint8_t dhcpOfferedIpAdd[4];
uint8_t dhcpServerIpAdd[4];

bool discoverNeeded = false;
bool requestNeeded = false;
bool releaseNeeded = false;

bool ipConflictDetection = false;
// ------------------------------------------------------------------------------
//  Structures
// ------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// State functions

void dhcpSetState(uint8_t state)
{
    dhcpState = state;
}

uint8_t dhcpGetState()
{
    return dhcpState;
}

// Timer functions
void T1()
{
    putsUart0("T1 expired!\n");
    dhcpSetState(DHCP_RENEWING);
    requestNeeded = true;
}

void T2()
{
    putsUart0("T2 expired!\n");
    dhcpSetState(DHCP_REBINDING);
    requestNeeded = true;
}

void T3()
{
    putsUart0("Lease expired!\n");
    dhcpSetState(DHCP_INIT);
    discoverNeeded = true;
}

// Send DHCP message
void dhcpSendMessage(etherHeader *ether, uint8_t type)
{
    uint32_t sum;
    uint8_t i;
    uint16_t tmp16, dhcpSize = 0;
    uint8_t mac[6];

    // Ether frame
    etherGetMacAddress(mac);
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = 0xFF;
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
    ip->protocol = 17;
    ip->headerChecksum = 0;
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = 0xFF;
        ip->sourceIp[i] = 0x0;
    }

    // UDP header
    udpHeader* udp = (udpHeader*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    udp->sourcePort = htons(68);
    udp->destPort = htons(67);

    // DHCP
    dhcpFrame* dhcp = (dhcpFrame*)udp->data;
    dhcp->op = 1;
    dhcp->htype = 1;
    dhcp->hlen = 6;
    dhcp->hops = 0;
    dhcp->xid = htonl(0x12345678);
    dhcp->secs = 0;
    dhcp->flags = 0;
    for(i=0; i<4; i++)
    {
        dhcp->ciaddr[i] = 0;
        dhcp->yiaddr[i] = 0;
        dhcp->siaddr[i] = 0;
        dhcp->giaddr[i] = 0;
        dhcp->chaddr[i] = 0;
        dhcp->chaddr[i+4] = 0;
        dhcp->chaddr[i+8] = 0;
        dhcp->chaddr[i+12] = 0;
    }
    etherGetMacAddress(&dhcp->chaddr[0]);
    for(i=0; i<192; i++)
    {
        dhcp->data[i] = 0;
    }
    // continue dhcp message here

    // add magic cookie
    dhcp->magicCookie = htonl(0x63825363);
    // send dhcp message type (53)
    dhcp->options[0] = 53;
    dhcp->options[1] = 1;
    if(type == DHCPDISCOVER)
    {
        dhcp->options[2] = 1;
        // send requested ip (50) as needed
        dhcp->options[3] = 50;
        dhcp->options[4] = 4;
        etherGetIpAddress(&dhcp->options[5]);
        // send parameter request list (55)
        dhcp->options[9] = 55;
        dhcp->options[10] = 4;
        dhcp->options[11] = 1;
        dhcp->options[12] = 2;
        dhcp->options[13] = 3;
        dhcp->options[14] = 6;
        dhcp->options[15] = 255;
        dhcpSize = 256;
        xid = dhcp->xid;
    }
    if(type == DHCPREQUEST)
    {
        dhcp->options[2] = 3;
        if(dhcpGetState() == DHCP_REBINDING)
        {
            etherGetIpAddress(dhcp->ciaddr);
        }
        // send requested ip (50) as needed
        dhcp->options[3] = 50;
        dhcp->options[4] = 4;
        dhcp->options[5] = dhcpOfferedIpAdd[0];
        dhcp->options[6] = dhcpOfferedIpAdd[1];
        dhcp->options[7] = dhcpOfferedIpAdd[2];
        dhcp->options[8] = dhcpOfferedIpAdd[3];
        // send server ip (54) as needed
        dhcp->options[9] = 54;
        dhcp->options[10] = 4;
        dhcp->options[11] = dhcpServerIpAdd[0];
        dhcp->options[12] = dhcpServerIpAdd[1];
        dhcp->options[13] = dhcpServerIpAdd[2];
        dhcp->options[14] = dhcpServerIpAdd[3];
        // send parameter request list (55)
        dhcp->options[15] = 55;
        dhcp->options[16] = 4;
        dhcp->options[17] = 1;
        dhcp->options[18] = 2;
        dhcp->options[19] = 3;
        dhcp->options[20] = 6;
        dhcp->options[21] = 255;
        dhcpSize = 262;
        xid = dhcp->xid;
    }
    if(type == DHCPDECLINE)
    {
        dhcp->options[2] = 4;
        dhcp->options[3] =255;
        dhcpSize = 244;
    }
    if(type == DHCPRELEASE)
    {
        dhcp->options[2] = 7;
        // send server ip (54) as needed
        dhcp->options[3] = 54;
        dhcp->options[4] = 4;
        dhcp->options[5] = dhcpServerIpAdd[0];
        dhcp->options[6] = dhcpServerIpAdd[1];
        dhcp->options[7] = dhcpServerIpAdd[2];
        dhcp->options[8] = dhcpServerIpAdd[3];
        dhcp->options[9] = 255;
        dhcpSize = 250;
    }
    // calculate dhcp size, update ip and udp lengths
    uint8_t ipHeaderLength = (ip->revSize & 0xF) * 4;
    ip->length = htons(ipHeaderLength + 8 + dhcpSize);
    udp->length = htons(8 + dhcpSize);
    // calculate ip header checksum, calculate udp checksum
    etherCalcIpChecksum(ip);
    // 32-bit sum over pseudo-header
    sum = 0;
    etherSumWords(ip->sourceIp, 8, &sum);
    tmp16 = ip->protocol;
    sum += (tmp16 & 0xff) << 8;
    etherSumWords(&udp->length, 2, &sum);
    // add udp header
    udp->check = 0;
    etherSumWords(udp, 8 + dhcpSize, &sum);
    udp->check = getEtherChecksum(sum);

    // send packet with size = ether hdr + ip header + udp hdr + dhcp_size
    // send packet
    etherPutPacket(ether, sizeof(etherHeader) + ipHeaderLength + 8 + dhcpSize);
}

uint8_t* getOption(etherHeader *ether, uint8_t option, uint8_t* length)
{
    ipHeader* ip = (ipHeader*)ether->data;
    udpHeader* udp = (udpHeader*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    dhcpFrame* dhcp = (dhcpFrame*)&udp->data;
    static uint8_t opt[10];
    uint8_t i = 0, k = 0;
    // suggest this function to programatically extract the field pointer and length
    while(dhcp->options[0] != 255)
    {
        if(*(&(dhcp->options[0])+i) == option)
        {
            length = (uint8_t*)(&(dhcp->options[0])+i+1);
            for(k = 0; k < *(length); k++)
            {
                i++;
                opt[k] = *((uint8_t*)(&(dhcp->options[0])+i+1));
            }
            return opt;
        }
        else
            i++;
    }
    return NULL;
}

// Determines whether packet is DHCP offer response to DHCP discover
// Must be a UDP packet
bool dhcpIsOffer(etherHeader *ether, uint8_t ipOfferedAdd[])
{
    ipHeader* ip = (ipHeader*)ether->data;
    udpHeader* udp = (udpHeader*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    dhcpFrame* dhcp = (dhcpFrame*)&udp->data;

    int8_t i;
    uint8_t len = 0;
    uint8_t msg = dhcp->options[2];
    uint8_t* opt = getOption(ether, 54, &len);
    for(i=0; i<4; i++)
    {
        dhcpOfferedIpAdd[i] = dhcp->yiaddr[i];
        dhcpServerIpAdd[i] = *(opt+i);
    }
    // return true if destport=68 and sourceport=67, op=2, xid correct, and offer msg
    bool ok = 0;
    ok = (udp->sourcePort == htons(67)) & (udp->destPort == htons(68)) & (dhcp->op == 2) & (dhcp->xid == xid) & (msg == 2);
    return ok;
}

// Determines whether packet is DHCP ACK response to DHCP request
// Must be a UDP packet
bool dhcpIsAck(etherHeader *ether)
{
    ipHeader* ip = (ipHeader*)ether->data;
    udpHeader* udp = (udpHeader*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    dhcpFrame* dhcp = (dhcpFrame*)&udp->data;

    uint8_t msg = dhcp->options[2];
    // return true if destport=68 and sourceport=67, op=2, xid correct, and ack msg
    bool ok = 0;
    ok = (udp->sourcePort == htons(67)) & (udp->destPort == htons(68)) & (dhcp->op == 2) & (dhcp->xid == xid) & (msg == 5);
    return ok;
}

// Handle a DHCP ACK
void dhcpHandleAck(etherHeader *ether)
{
    ipHeader* ip = (ipHeader*)ether->data;
    udpHeader* udp = (udpHeader*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    dhcpFrame* dhcp = (dhcpFrame*)&udp->data;

    uint8_t i;
    uint8_t len = 0;
    uint8_t* opt;
    uint8_t addr[4];
    // extract offered IP address
    opt = getOption(ether, 54, &len);
    for(i=0; i<4; i++)
    {
        dhcpOfferedIpAdd[i] = dhcp->yiaddr[i];
        dhcpServerIpAdd[i] = *(opt+i);
    }
    etherSetIpAddress(dhcpOfferedIpAdd);

    // store sn, gw, dns, and time from options
    opt = getOption(ether, 1, &len);
    for(i=0; i<4; i++)
    {
        addr[i] = *(opt+i);
    }
    etherSetIpSubnetMask(addr);

    // gw
    opt = getOption(ether, 3, &len);
    for(i=0; i<4; i++)
    {
        addr[i] = *(opt+i);
    }
    etherSetIpGatewayAddress(addr);

    // store dns server address for later use
    opt = getOption(ether, 6, &len);
    for(i=0; i<4; i++)
    {
        addr[i] = *(opt+i);
    }
    etherSetIpDnsAddress(addr);

    // store lease, t1, and t2
    opt = getOption(ether, 51, &len);
    for(i=0; i<4; i++)
    {
        addr[i] = *(opt+i);
    }
    leaseSeconds = 20; //(addr[0]<<0) | (addr[1]<<8) | (addr[2]<<16) | (addr[3]<<24);
    leaseT1 = 5; //leaseSeconds * 0.5;
    leaseT2 = leaseSeconds * 0.875;
    // stop new address needed timer, t1 timer, t2 timer
    // start t1, t2, and lease end timers
    startOneshotTimer(T1, leaseT1);
    startOneshotTimer(T2, leaseT2);
    startOneshotTimer(T3, leaseSeconds);
}

void dhcpSendPendingMessages(etherHeader *ether)
{
    // if discover needed, send discover, enter selecting state
    if(discoverNeeded && dhcpGetState() == DHCP_INIT)
    {
        dhcpSendMessage(ether, DHCPDISCOVER);
        dhcpSetState(DHCP_SELECTING);
        discoverNeeded = false;
    }
    // if request needed, send request
    if(requestNeeded && dhcpGetState() == DHCP_REQUESTING)
    {
        dhcpSendMessage(ether, DHCPREQUEST);
        requestNeeded = false;
    }
    if(requestNeeded && dhcpGetState() == DHCP_RENEWING)
    {
        dhcpSendMessage(ether, DHCPREQUEST);
        requestNeeded = false;
    }
    if(requestNeeded && dhcpGetState() == DHCP_REBINDING)
    {
        dhcpSendMessage(ether, DHCPREQUEST);
        requestNeeded = false;
    }
    if(ipConflictDetection && dhcpGetState() == DHCP_TESTING_IP)
    {
        dhcpSendMessage(ether, DHCPDECLINE);
        waitMicrosecond(15000000);
        dhcpSendMessage(ether, DHCPREQUEST);
        requestNeeded = false;
        ipConflictDetection = false;
    }
    // if release needed, send release
    if(releaseNeeded)
    {
        dhcpSendMessage(ether, DHCPRELEASE);
        releaseNeeded = false;
    }
    //waitMicrosecond(10000);
}

void dhcpProcessDhcpResponse(etherHeader *ether)
{
    // if offer, send request and enter requesting state
    if(dhcpIsOffer(ether, dhcpOfferedIpAdd) & dhcpGetState() == DHCP_SELECTING)
    {
        requestNeeded = true;
        dhcpSetState(DHCP_REQUESTING);
    }
    // if ack, call handle ack, send arp request, enter ip conflict test state
    if(dhcpIsAck(ether) && dhcpGetState() == DHCP_REQUESTING)
    {
        dhcpHandleAck(ether);
        dhcpSetState(DHCP_TESTING_IP);
    }
    if(!ipConflictDetection && dhcpGetState() == DHCP_TESTING_IP)
    {
        dhcpHandleAck(ether);
        dhcpSetState(DHCP_BOUND);
    }
    if(dhcpIsAck(ether) && dhcpGetState() == DHCP_RENEWING)
    {
        dhcpHandleAck(ether);
        dhcpSetState(DHCP_BOUND);
    }
    if(dhcpIsAck(ether) && dhcpGetState() == DHCP_REBINDING)
    {
        dhcpHandleAck(ether);
        dhcpSetState(DHCP_BOUND);
    }
    //waitMicrosecond(10000);
}

void dhcpProcessArpResponse(etherHeader *ether)
{
    // if in conflict resolution, if a response matches the offered add,
    //  send decline and request new address
    if(dhcpGetState() == DHCP_TESTING_IP)
    {
        ipConflictDetection = true;
        requestNeeded = true;
    }
}

// DHCP control functions

void dhcpEnable()
{
    // request new address
    dhcpSetState(DHCP_INIT);
    discoverNeeded = true;
}

void dhcpDisable()
{
    // set state to disabled, stop all timers
    dhcpSetState(DHCP_DISABLED);
    stopTimer(T1);
    stopTimer(T2);
    stopTimer(T3);
}

bool dhcpIsEnabled()
{
    return (dhcpState != DHCP_DISABLED);
}

void dhcpRequestRenew()
{
    dhcpSetState(DHCP_RENEWING);
    requestNeeded = true;
}

void dhcpRequestRebind()
{
    dhcpSetState(DHCP_REBINDING);
    requestNeeded = true;
}

void dhcpRequestRelease()
{
    releaseNeeded = true;
}

uint32_t dhcpGetLeaseSeconds()
{
    return leaseSeconds;
}

