#ifndef ETHERNET_LPC_H
#define ETHERNET_LPC_H

//Wrapper to function like Ethernet_sam

#include <stdbool.h>
#include "lwip/netif.h"

#include "eth_arch.h"


#ifdef __cplusplus
extern "C" {
#endif

bool init_ethernet(const uint8_t ipAddress[], const uint8_t netMask[], const uint8_t gateWay[]);
void ethernet_configure_interface(const u8_t macAddress[], const char *hostname);
bool ethernet_establish_link(void);
bool ethernet_link_established(void);
//void start_ethernet(const uint8_t ipAddress[], const uint8_t netMask[], const uint8_t gateWay[], netif_status_callback_fn status_cb);
void ethernet_set_configuration(const unsigned char ipAddress[], const unsigned char netMask[], const unsigned char gateWay[]);
void ethernet_timers_update(void);
void ethernet_task(void);
void ethernet_set_rx_callback(emac_dev_tx_cb_t callback);
const uint8_t *ethernet_get_ipaddress();

#ifdef __cplusplus
}
#endif


#endif
