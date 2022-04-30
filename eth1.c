// ETH1 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL w/ ENC28J60
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// ENC28J60 Ethernet controller on SPI1
//   MOSI (SSI1Tx) on PD3
//   MISO (SSI1Rx) on PD2
//   SCLK (SSI1Clk) on PD0
//   ~CS (SW controlled) on PD1
//   WOL on PB3
//   INT on PC6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "gpio.h"
#include "spi1.h"
#include "eth1.h"

// Pins
#define CS PORTD,1
#define WOL PORTB,3
#define INT PORTC,6

// Ether registers
#define ERDPTL      0x00
#define ERDPTH      0x01
#define EWRPTL      0x02
#define EWRPTH      0x03
#define ETXSTL      0x04
#define ETXSTH      0x05
#define ETXNDL      0x06
#define ETXNDH      0x07
#define ERXSTL      0x08
#define ERXSTH      0x09
#define ERXNDL      0x0A
#define ERXNDH      0x0B
#define ERXRDPTL    0x0C
#define ERXRDPTH    0x0D
#define ERXWRPTL    0x0E
#define ERXWRPTH    0x0F
#define EIE         0x1B
#define EIR         0x1C
#define RXERIF  0x01
#define TXERIF  0x02
#define TXIF    0x08
#define PKTIF   0x40
#define ESTAT       0x1D
#define CLKRDY  0x01
#define TXABORT 0x02
#define ECON2       0x1E
#define PKTDEC  0x40
#define ECON1       0x1F
#define RXEN    0x04
#define TXRTS   0x08
#define ERXFCON     0x38
#define EPKTCNT     0x39
#define MACON1      0x40
#define MARXEN  0x01
#define RXPAUS  0x04
#define TXPAUS  0x08
#define MACON2      0x41
#define MARST   0x80
#define MACON3      0x42
#define FULDPX  0x01
#define FRMLNEN 0x02
#define TXCRCEN 0x10
#define PAD60   0x20
#define MACON4      0x43
#define MABBIPG     0x44
#define MAIPGL      0x46
#define MAIPGH      0x47
#define MACLCON1    0x48
#define MACLCON2    0x49
#define MAMXFLL     0x4A
#define MAMXFLH     0x4B
#define MICMD       0x52
#define MIIRD   0x01
#define MIREGADR    0x54
#define MIWRL       0x56
#define MIWRH       0x57
#define MIRDL       0x58
#define MIRDH       0x59
#define MAADR1      0x60
#define MAADR0      0x61
#define MAADR3      0x62
#define MAADR2      0x63
#define MAADR5      0x64
#define MAADR4      0x65
#define MISTAT      0x6A
#define MIBUSY  0x01
#define ECOCON      0x75

// Ether phy registers
#define PHCON1      0x00
#define PDPXMD 0x0100
#define PHSTAT1     0x01
#define LSTAT  0x0400
#define PHCON2      0x10
#define HDLDIS 0x0100
#define PHLCON      0x14

// ------------------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------------------

uint8_t nextPacketLsb1 = 0x00;
uint8_t nextPacketMsb1 = 0x00;
uint8_t sequenceId1 = 1;
uint8_t hwAddress1[HW_ADD_LENGTH] = {2,3,4,5,6,7};
uint8_t ipAddress1[IP_ADD_LENGTH] = {0,0,0,0};
uint8_t ipSubnetMask1[IP_ADD_LENGTH] = {0,0,0,0};
uint8_t ipGwAddress1[IP_ADD_LENGTH] = {0,0,0,0};
uint8_t ipDnsAddress1[IP_ADD_LENGTH] = {0,0,0,0};
uint8_t ipTimeServerAddress1[IP_ADD_LENGTH] = {0,0,0,0};
bool    dhcpEnabled1 = true;

// ------------------------------------------------------------------------------
//  Structures
// ------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Buffer is configured as follows
// Receive buffer starts at 0x0000 (bottom 6666 bytes of 8K space)
// Transmit buffer at 01A0A (top 1526 bytes of 8K space)

void ether1CsOn(void)
{
    setPinValue(CS, 0);
    _delay_cycles(4);                    // allow line to settle
}

void ether1CsOff(void)
{
    setPinValue(CS, 1);
}

void ether1WriteReg(uint8_t reg, uint8_t data)
{
    ether1CsOn();
    writeSpi1Data(0x40 | (reg & 0x1F));
    readSpi1Data();
    writeSpi1Data(data);
    readSpi1Data();
    ether1CsOff();
}

uint8_t ether1ReadReg(uint8_t reg)
{
    uint8_t data;
    ether1CsOn();
    writeSpi1Data(0x00 | (reg & 0x1F));
    readSpi1Data();
    writeSpi1Data(0);
    data = readSpi1Data();
    ether1CsOff();
    return data;
}

void ether1SetReg(uint8_t reg, uint8_t mask)
{
    ether1CsOn();
    writeSpi1Data(0x80 | (reg & 0x1F));
    readSpi1Data();
    writeSpi1Data(mask);
    readSpi1Data();
    ether1CsOff();
}

void ether1ClearReg(uint8_t reg, uint8_t mask)
{
    ether1CsOn();
    writeSpi1Data(0xA0 | (reg & 0x1F));
    readSpi1Data();
    writeSpi1Data(mask);
    readSpi1Data();
    ether1CsOff();
}

void ether1SetBank(uint8_t reg)
{
    ether1ClearReg(ECON1, 0x03);
    ether1SetReg(ECON1, reg >> 5);
}

void ether1WritePhy(uint8_t reg, uint16_t data)
{
    ether1SetBank(MIREGADR);
    ether1WriteReg(MIREGADR, reg);
    ether1WriteReg(MIWRL, data & 0xFF);
    ether1WriteReg(MIWRH, (data >> 8) & 0xFF);
}

uint16_t ether1ReadPhy(uint8_t reg)
{
    uint16_t data, dataH;
    ether1SetBank(MIREGADR);
    ether1WriteReg(MIREGADR, reg);
    ether1WriteReg(MICMD, MIIRD);
    waitMicrosecond(11);
    ether1SetBank(MISTAT);
    while ((ether1ReadReg(MISTAT) & MIBUSY) != 0);
    ether1SetBank(MICMD);
    ether1WriteReg(MICMD, 0);
    data = ether1ReadReg(MIRDL);
    dataH = ether1ReadReg(MIRDH);
    data |= (dataH << 8);
    return data;
}

void ether1WriteMemStart(void)
{
    ether1CsOn();
    writeSpi1Data(0x7A);
    readSpi1Data();
}

void ether1WriteMem(uint8_t data)
{
    writeSpi1Data(data);
    readSpi1Data();
}

void ether1WriteMemStop(void)
{
    ether1CsOff();
}

void ether1ReadMemStart(void)
{
    ether1CsOn();
    writeSpi1Data(0x3A);
    readSpi1Data();
}

uint8_t ether1ReadMem(void)
{
    writeSpi1Data(0);
    return readSpi1Data();
}

void ether1ReadMemStop(void)
{
    ether1CsOff();
}

// Initializes ethernet device
// Uses order suggested in Chapter 6 of datasheet except 6.4 OST which is first here
void ether1Init(uint16_t mode)
{
    // Initialize SPI1
    initSpi1(USE_SSI1_RX);
    setSpi1BaudRate(10e6, 40e6);
    setSpi1Mode(0, 0);

    // Enable clocks
    enablePort(PORTA);
    enablePort(PORTB);
    enablePort(PORTC);

    // Configure pins for ethernet module
    selectPinPushPullOutput(CS);
    selectPinDigitalInput(WOL);
    selectPinDigitalInput(INT);

    // make sure that oscillator start-up timer has expired
    while ((ether1ReadReg(ESTAT) & CLKRDY) == 0) {}

    // disable transmission and reception of packets
    ether1ClearReg(ECON1, RXEN);
    ether1ClearReg(ECON1, TXRTS);

    // initialize receive buffer space
    ether1SetBank(ERXSTL);
    ether1WriteReg(ERXSTL, LOBYTE(0x0000));
    ether1WriteReg(ERXSTH, HIBYTE(0x0000));
    ether1WriteReg(ERXNDL, LOBYTE(0x1A09));
    ether1WriteReg(ERXNDH, HIBYTE(0x1A09));

    // initialize receiver write and read ptrs
    // at startup, will write from 0 to 1A08 only and will not overwrite rd ptr
    ether1WriteReg(ERXWRPTL, LOBYTE(0x0000));
    ether1WriteReg(ERXWRPTH, HIBYTE(0x0000));
    ether1WriteReg(ERXRDPTL, LOBYTE(0x1A09));
    ether1WriteReg(ERXRDPTH, HIBYTE(0x1A09));
    ether1WriteReg(ERDPTL, LOBYTE(0x0000));
    ether1WriteReg(ERDPTH, HIBYTE(0x0000));

    // setup receive filter
    // always check CRC, use OR mode
    ether1SetBank(ERXFCON);
    ether1WriteReg(ERXFCON, (mode | ETHER_CHECKCRC) & 0xFF);

    // bring mac out of reset
    ether1SetBank(MACON2);
    ether1WriteReg(MACON2, 0);

    // enable mac rx, enable pause control for full duplex
    ether1WriteReg(MACON1, TXPAUS | RXPAUS | MARXEN);

    // enable padding to 60 bytes (no runt packets)
    // add crc to tx packets, set full or half duplex
    if ((mode & ETHER_FULLDUPLEX) != 0)
        ether1WriteReg(MACON3, FULDPX | FRMLNEN | TXCRCEN | PAD60);
    else
        ether1WriteReg(MACON3, FRMLNEN | TXCRCEN | PAD60);

    // leave MACON4 as reset

    // set maximum rx packet size
    ether1WriteReg(MAMXFLL, LOBYTE(1518));
    ether1WriteReg(MAMXFLH, HIBYTE(1518));

    // set back-to-back inter-packet gap to 9.6us
    if ((mode & ETHER_FULLDUPLEX) != 0)
        ether1WriteReg(MABBIPG, 0x15);
    else
        ether1WriteReg(MABBIPG, 0x12);

    // set non-back-to-back inter-packet gap registers
    ether1WriteReg(MAIPGL, 0x12);
    ether1WriteReg(MAIPGH, 0x0C);

    // leave collision window MACLCON2 as reset

    // initialize phy duplex
    if ((mode & ETHER_FULLDUPLEX) != 0)
        ether1WritePhy(PHCON1, PDPXMD);
    else
        ether1WritePhy(PHCON1, 0);

    // disable phy loopback if in half-duplex mode
    ether1WritePhy(PHCON2, HDLDIS);

    // Flash LEDA and LEDB
    ether1WritePhy(PHLCON, 0x0880);
    waitMicrosecond(100000);

    // set LEDA (link status) and LEDB (tx/rx activity)
    // stretch LED on to 40ms (default)
    ether1WritePhy(PHLCON, 0x0472);

    // enable reception
    ether1SetReg(ECON1, RXEN);
}

// Returns true if link is up
bool ether1IsLinkUp(void)
{
    return (ether1ReadPhy(PHSTAT1) & LSTAT) != 0;
}

// Returns TRUE if packet received
bool ether1IsDataAvailable(void)
{
    return ((ether1ReadReg(EIR) & PKTIF) != 0);
}

// Returns true if rx buffer overflowed after correcting the problem
bool ether1IsOverflow(void)
{
    bool err;
    err = (ether1ReadReg(EIR) & RXERIF) != 0;
    if (err)
        ether1ClearReg(EIR, RXERIF);
    return err;
}

// Returns up to max_size characters in data buffer
// Returns number of bytes copied to buffer
// Contents written are 16-bit size, 16-bit status, payload excl crc
uint16_t ether1GetPacket(etherHeader *ether, uint16_t maxSize)
{
    uint16_t i = 0, size, tmp16, status;
    uint8_t *packet = (uint8_t*)ether;

    // enable read from FIFO buffers
    ether1ReadMemStart();

    // get next packet information
    nextPacketLsb1 = ether1ReadMem();
    nextPacketMsb1 = ether1ReadMem();

    // calc size
    // don't return crc, instead return size + status, so size is correct
    size = ether1ReadMem();
    tmp16 = ether1ReadMem();
    size |= (tmp16 << 8);

    // get status (currently unused)
    status = ether1ReadMem();
    tmp16 = ether1ReadMem();
    status |= (tmp16 << 8);

    // copy data
    if (size > maxSize)
        size = maxSize;
    while (i < size)
        packet[i++] = ether1ReadMem();

    // end read from FIFO buffers
    ether1ReadMemStop();

    // advance read pointer
    ether1SetBank(ERXRDPTL);
    ether1WriteReg(ERXRDPTL, nextPacketLsb1); // hw ptr
    ether1WriteReg(ERXRDPTH, nextPacketMsb1);
    ether1WriteReg(ERDPTL, nextPacketLsb1);   // dma rd ptr
    ether1WriteReg(ERDPTH, nextPacketMsb1);

    // decrement packet counter so that PKTIF is maintained correctly
    ether1SetReg(ECON2, PKTDEC);

    return size;
}

// Writes a packet
bool ether1PutPacket(etherHeader *ether, uint16_t size)
{
    uint16_t i;
    uint8_t *packet = (uint8_t*) ether;

    // clear out any tx errors
    if ((ether1ReadReg(EIR) & TXERIF) != 0)
    {
        ether1ClearReg(EIR, TXERIF);
        ether1SetReg(ECON1, TXRTS);
        ether1ClearReg(ECON1, TXRTS);
    }

    // set DMA start address
    ether1SetBank(EWRPTL);
    ether1WriteReg(EWRPTL, LOBYTE(0x1A0A));
    ether1WriteReg(EWRPTH, HIBYTE(0x1A0A));

    // start FIFO buffer write
    ether1WriteMemStart();

    // write control byte
    ether1WriteMem(0);

    // write data
    for (i = 0; i < size; i++)
        ether1WriteMem(packet[i]);

    // stop write
    ether1WriteMemStop();

    // request transmit
    ether1WriteReg(ETXSTL, LOBYTE(0x1A0A));
    ether1WriteReg(ETXSTH, HIBYTE(0x1A0A));
    ether1WriteReg(ETXNDL, LOBYTE(0x1A0A+size));
    ether1WriteReg(ETXNDH, HIBYTE(0x1A0A+size));
    ether1ClearReg(EIR, TXIF);
    ether1SetReg(ECON1, TXRTS);

    // wait for completion
    while ((ether1ReadReg(ECON1) & TXRTS) != 0);

    // determine success
    return ((ether1ReadReg(ESTAT) & TXABORT) == 0);
}

// Calculate sum of words
// Must use getEther1Checksum to complete 1's compliment addition
void ether1SumWords(void* data, uint16_t sizeInBytes, uint32_t* sum)
{
    uint8_t* pData = (uint8_t*)data;
    uint16_t i;
    uint8_t phase = 0;
    uint16_t data_temp;
    for (i = 0; i < sizeInBytes; i++)
    {
        if (phase)
        {
            data_temp = *pData;
            *sum += data_temp << 8;
        }
        else
          *sum += *pData;
        phase = 1 - phase;
        pData++;
    }
}

// Completes 1's compliment addition by folding carries back into field
uint16_t getEther1Checksum(uint32_t sum)
{
    uint16_t result;
    // this is based on rfc1071
    while ((sum >> 16) > 0)
      sum = (sum & 0xFFFF) + (sum >> 16);
    result = sum & 0xFFFF;
    return ~result;
}

void ether1CalcIpChecksum(ipHeader* ip)
{
    // 32-bit sum over ip header
    uint32_t sum = 0;
    ip->headerChecksum = 0;
    ether1SumWords(ip, ((ip->revSize & 0xF) * 4), &sum);
    ip->headerChecksum = getEther1Checksum(sum);
}


// Converts from host to network order and vice versa
uint16_t htons1(uint16_t value)
{
    return ((value & 0xFF00) >> 8) + ((value & 0x00FF) << 8);
}


uint32_t htonl1(uint32_t value)
{
    return ((value & 0xFF000000) >> 24) + ((value & 0x00FF0000) >> 8) +
           ((value & 0x0000FF00) << 8) + ((value & 0x000000FF) << 24);
}

// Determines whether packet is IP datagram
bool ether1IsIp(etherHeader *ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = (ip->revSize & 0xF) * 4;
    uint32_t sum = 0;
    bool ok;
    ok = (ether->frameType == htons1(0x0800));
    if (ok)
    {
        ether1SumWords(&ip->revSize, ipHeaderLength, &sum);
        ok = (getEther1Checksum(sum) == 0);
    }
    return ok;
}

// Determines whether packet is unicast to this ip
// Must be an IP packet
bool ether1IsIpUnicast(etherHeader *ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t i = 0;
    bool ok = true;
    while (ok & (i < IP_ADD_LENGTH))
    {
        ok = (ip->destIp[i] == ipAddress1[i]);
        i++;
    }
    return ok;
}

// Determines whether packet is ping request
// Must be an IP packet
bool ether1IsPingRequest(etherHeader *ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = (ip->revSize & 0xF) * 4;
    icmpHeader *icmp = (icmpHeader*)((uint8_t*)ip + ipHeaderLength);
    return (ip->protocol == 0x01 & icmp->type == 8);
}

// Sends a ping response given the request data
void ether1SendPingResponse(etherHeader *ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = (ip->revSize & 0xF) * 4;
    icmpHeader *icmp = (icmpHeader*)((uint8_t*)ip + ipHeaderLength);
    uint8_t i, tmp;
    uint16_t icmp_size;
    uint32_t sum = 0;
    // swap source and destination fields
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip ->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }
    // this is a response
    icmp->type = 0;
    // calc icmp checksum
    icmp->check = 0;
    icmp_size = ntohs(ip->length) - ipHeaderLength;
    ether1SumWords(icmp, icmp_size, &sum);
    icmp->check = getEther1Checksum(sum);
    // send packet
    ether1PutPacket(ether, sizeof(etherHeader) + ntohs(ip->length));
    etherGetPacket(ether, sizeof(etherHeader) + ntohs(ip->length));
    etherPutPacket(ether, sizeof(etherHeader) + ntohs(ip->length));
}

// Determines whether packet is ARP
bool ether1IsArpRequest(etherHeader *ether)
{
    arpPacket *arp = (arpPacket*)ether->data;
    bool ok;
    uint8_t i = 0;
    ok = (ether->frameType == htons1(0x0806));
    while (ok & (i < IP_ADD_LENGTH))
    {
        ok = (arp->destIp[i] == ipAddress1[i]);
        i++;
    }
    if (ok)
        ok = (arp->op == htons1(1));
    return ok;
}

// Determines whether packet is ARP response
bool ether1IsArpResponse(etherHeader *ether)
{
    arpPacket *arp = (arpPacket*)ether->data;
    bool ok;
    ok = (ether->frameType == htons1(0x0806));
    if (ok)
        ok = (arp->op == htons1(2));
    return ok;
}

// Sends an ARP response given the request data
void ether1SendArpResponse(etherHeader *ether)
{
    arpPacket *arp = (arpPacket*)ether->data;
    uint8_t i, tmp;
    // set op to response
    arp->op = htons1(2);
    // swap source and destination fields
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        arp->destAddress[i] = arp->sourceAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = arp->sourceAddress[i] = hwAddress1[i];
    }
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        tmp = arp->destIp[i];
        arp->destIp[i] = arp->sourceIp[i];
        arp->sourceIp[i] = tmp;
    }
    // send packet
    ether1PutPacket(ether, sizeof(etherHeader) + sizeof(arpPacket));
    etherGetPacket(ether, sizeof(etherHeader) + sizeof(arpPacket));
    etherPutPacket(ether, sizeof(etherHeader) + sizeof(arpPacket));
}

// Sends an ARP request
void ether1SendArpRequest(etherHeader *ether, uint8_t ipFrom[], uint8_t ipTo[])
{
    arpPacket *arp = (arpPacket*)ether->data;
    uint8_t i;
    // fill ethernet frame
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = 0xFF;
        ether->sourceAddress[i] = hwAddress1[i];
    }
    ether->frameType = htons1(0x0806);
    // fill arp frame
    arp->hardwareType = htons1(1);
    arp->protocolType = htons1(0x0800);
    arp->hardwareSize = HW_ADD_LENGTH;
    arp->protocolSize = IP_ADD_LENGTH;
    arp->op = htons1(1);
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        arp->sourceAddress[i] = hwAddress1[i];
        arp->destAddress[i] = 0xFF;
    }
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        arp->sourceIp[i] = ipFrom[i];
        arp->destIp[i] = ipTo[i];
    }
    // send packet
    ether1PutPacket(ether, sizeof(etherHeader) + sizeof(arpPacket));
    etherGetPacket(ether, sizeof(etherHeader) + sizeof(arpPacket));
    etherPutPacket(ether, sizeof(etherHeader) + sizeof(arpPacket));
}

// Determines whether packet is UDP datagram
// Must be an IP packet
bool ether1IsUdp(etherHeader *ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = (ip->revSize & 0xF) * 4;
    udpHeader *udp = (udpHeader*)((uint8_t*)ip + ipHeaderLength);
    bool ok;
    uint16_t tmp16;
    uint32_t sum = 0;
    ok = (ip->protocol == 0x11);
    if (ok)
    {
        // 32-bit sum over pseudo-header
        ether1SumWords(ip->sourceIp, 8, &sum);
        tmp16 = ip->protocol;
        sum += (tmp16 & 0xff) << 8;
        ether1SumWords(&udp->length, 2, &sum);
        // add udp header and data
        ether1SumWords(udp, ntohs(udp->length), &sum);
        ok = (getEther1Checksum(sum) == 0);
    }
    return ok;
}

// Gets pointer to UDP payload of frame
uint8_t * ether1GetUdpData(etherHeader *ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = (ip->revSize & 0xF) * 4;
    udpHeader *udp = (udpHeader*)((uint8_t*)ip + ipHeaderLength);
    return udp->data;
}

// Send responses to a udp datagram
// destination port, ip, and hardware address are extracted from provided data
// uses destination port of received packet as destination of this packet
void ether1SendUdpResponse(etherHeader *ether, uint8_t *udpData, uint8_t udpSize)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = (ip->revSize & 0xF) * 4;
    udpHeader *udp = (udpHeader*)((uint8_t*)ip + ipHeaderLength);
    uint16_t udpLength;
    uint8_t *copyData;
    uint32_t sum = 0;
    uint8_t i, tmp8;
    uint16_t tmp16;
    // swap source and destination fields
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        tmp8 = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp8;
    }
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        tmp8 = ip->destIp[i];
        ip->destIp[i] = ip->sourceIp[i];
        ip->sourceIp[i] = tmp8;
    }
    // set source port of resp will be dest port of req
    // dest port of resp will be left at source port of req
    // unusual nomenclature, but this allows a different tx
    // and rx port on other machine
    udp->sourcePort = udp->destPort;
    // adjust lengths
    udpLength = 8 + udpSize;
    ip->length = htons1(ipHeaderLength + udpLength);
    // 32-bit sum over ip header
    ether1CalcIpChecksum(ip);
    // set udp length
    udp->length = htons1(udpLength);
    // copy data
    copyData = udp->data;
    for (i = 0; i < udpSize; i++)
        copyData[i] = udpData[i];
    // 32-bit sum over pseudo-header
    sum = 0;
    ether1SumWords(ip->sourceIp, 8, &sum);
    tmp16 = ip->protocol;
    sum += (tmp16 & 0xff) << 8;
    ether1SumWords(&udp->length, 2, &sum);
    // add udp header
    udp->check = 0;
    ether1SumWords(udp, udpLength, &sum);
    udp->check = getEther1Checksum(sum);

    // send packet with size = ether + udp hdr + ip header + udp_size
    ether1PutPacket(ether, sizeof(etherHeader) + ipHeaderLength + udpLength);
    etherGetPacket(ether, sizeof(etherHeader) + ipHeaderLength + udpLength);
    etherPutPacket(ether, sizeof(etherHeader) + ipHeaderLength + udpLength);
}

// Determines whether packet is DHCP
// Must be a UDP packet
bool ether1IsDhcpResponse(etherHeader* ether)
{
    ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = (ip->revSize & 0xF) * 4;
    udpHeader* udp = (udpHeader*)((uint8_t*)ip + ipHeaderLength);
    bool ok;
    ok = (udp->sourcePort == htons1(67)) & (udp->destPort == htons1(68));
    return ok;
}

// Determines whether packet is TCP packet
// Must be an IP packet
bool ether1IsTcp(etherHeader* ether)
{
   ipHeader *ip = (ipHeader*)ether->data;
    uint8_t ipHeaderLength = (ip->revSize & 0xF) * 4;
    tcpHeader* tcp = (tcpHeader*)((uint8_t*)ip + ipHeaderLength);
    uint32_t sum = 0;
    bool ok;
    uint16_t tmp16;
    ok = (ip->protocol == 6);
    if (ok)
    {
        // 32-bit sum over pseudo-header
        ether1SumWords(ip->sourceIp, 8, &sum);
        tmp16 = ip->protocol;
        sum += (tmp16 & 0xff) << 8;
        tmp16 = htons1(ntohs(ip->length) - (ip->revSize & 0xF) * 4);
        ether1SumWords(&tmp16, 2, &sum);
        // add tcp header and data
        ether1SumWords(tcp, ntohs(ip->length) - (ip->revSize & 0xF) * 4, &sum);
        ok = (getEther1Checksum(sum) == 0);
    }
    return ok;
}

uint16_t ether1GetId(void)
{
    return htons1(sequenceId1);
}

void ether1IncId(void)
{
    sequenceId1++;
}

// Determines if the IP address is valid
bool ether1IsIpValid()
{
    return ipAddress1[0] || ipAddress1[1] || ipAddress1[2] || ipAddress1[3];
}

// Sets IP address
void ether1SetIpAddress(const uint8_t ip[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ipAddress1[i] = ip[i];
}

// Gets IP address
void ether1GetIpAddress(uint8_t ip[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ip[i] = ipAddress1[i];
}

// Sets IP subnet mask
void ether1SetIpSubnetMask(const uint8_t mask[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ipSubnetMask1[i] = mask[i];
}

// Gets IP subnet mask
void ether1GetIpSubnetMask(uint8_t mask[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        mask[i] = ipSubnetMask1[i];
}

// Sets IP gateway address
void ether1SetIpGatewayAddress(const uint8_t ip[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ipGwAddress1[i] = ip[i];
}

// Gets IP gateway address
void ether1GetIpGatewayAddress(uint8_t ip[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ip[i] = ipGwAddress1[i];
}

// Sets IP DNS address
void ether1SetIpDnsAddress(const uint8_t ip[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ipDnsAddress1[i] = ip[i];
}

// Gets IP gateway address
void ether1GetIpDnsAddress(uint8_t ip[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ip[i] = ipDnsAddress1[i];
}

// Sets IP time server address
void ether1SetIpTimeServerAddress(const uint8_t ip[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ipTimeServerAddress1[i] = ip[i];
}

// Gets IP time server address
void ether1GetIpTimeServerAddress(uint8_t ip[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ip[i] = ipTimeServerAddress1[i];
}

// Sets MAC address
void ether1SetMacAddress(uint8_t mac0, uint8_t mac1, uint8_t mac2, uint8_t mac3, uint8_t mac4, uint8_t mac5)
{
    hwAddress1[0] = mac0;
    hwAddress1[1] = mac1;
    hwAddress1[2] = mac2;
    hwAddress1[3] = mac3;
    hwAddress1[4] = mac4;
    hwAddress1[5] = mac5;
    ether1SetBank(MAADR0);
    ether1WriteReg(MAADR5, mac0);
    ether1WriteReg(MAADR4, mac1);
    ether1WriteReg(MAADR3, mac2);
    ether1WriteReg(MAADR2, mac3);
    ether1WriteReg(MAADR1, mac4);
    ether1WriteReg(MAADR0, mac5);
}

// Gets MAC address
void ether1GetMacAddress(uint8_t mac[6])
{
    uint8_t i;
    for (i = 0; i < 6; i++)
        mac[i] = hwAddress1[i];
}


