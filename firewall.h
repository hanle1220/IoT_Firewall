/*
 * firewall.h
 */

#ifndef FIREWALL_H_
#define FIREWALL_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "firewall.h"
#include "eth0.h"

typedef struct _firewall
{
  uint8_t ipAddress[4];
  uint8_t macAddress[4];
  uint8_t dnsAddress[4];
  uint8_t udpTcpFlag;
} firewall;

bool checkPacket(etherHeader* ether, firewall *f);

#endif /* FIREWALL_H_ */
