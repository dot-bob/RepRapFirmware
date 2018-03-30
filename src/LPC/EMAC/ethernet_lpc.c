

#include <string.h>
#include "ethernet_lpc.h"
#include "lpc_phy.h"

#include "lwip/init.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/memp.h"
#include "lwip/tcpip.h"
#include "lwip/tcp.h"
//#include "ipv4/lwip/ip_frag.h"
//#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"

#include "lpc17xx_emac.h"
#include "eth_arch.h"

#include "lpc_phy.h"

#include <stdbool.h>

//lwip v2
#include "lwip/timeouts.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"

#include "Core.h"


/* NETIF data */
struct netif gs_net_if;
bool hwInit = false;

void ethernet_timers_update(void)
{
    // LWIP timers - ARP, DHCP, TCP, etc.
    if(hwInit) sys_check_timeouts();
}


// This sets the IP configuration on-the-fly
void ethernet_set_configuration(const uint8_t ipAddress[], const uint8_t netMask[], const uint8_t gateWay[])
{

    ip_addr_t x_ip_addr, x_net_mask, x_gateway;
    IP4_ADDR(&x_ip_addr, ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]);
    IP4_ADDR(&x_net_mask, netMask[0], netMask[1], netMask[2], netMask[3]);
    IP4_ADDR(&x_gateway, gateWay[0], gateWay[1], gateWay[2], gateWay[3]);

    
    // use static IP address
    netif_set_ipaddr(&gs_net_if, &x_ip_addr);
    netif_set_netmask(&gs_net_if, &x_net_mask);
    netif_set_gw(&gs_net_if, &x_gateway);
    
    
    /* Set it up */
    netif_set_up(&gs_net_if);

    
}


/** \brief Initialize the Ethernet subsystem.
 *
 */
bool init_ethernet(const uint8_t ipAddress[], const uint8_t netMask[], const uint8_t gateWay[])
{

    ip4_addr_t x_ip_addr, x_net_mask, x_gateway;
    IP4_ADDR(&x_ip_addr, ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]);
    IP4_ADDR(&x_net_mask, netMask[0], netMask[1], netMask[2], netMask[3]);
    IP4_ADDR(&x_gateway, gateWay[0], gateWay[1], gateWay[2], gateWay[3]);

    lwip_init();

    hwInit = lpc_hardware_init(); // Init Hardware (eth PHY to detect Link Status etc)
    if(hwInit == false){
        //PHY failed.....
        LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE, ("init_ethernet: PHY Failed - No network Available\n"));
        return false; //nothing left to do now
    } else {
        LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE, ("init_ethernet: PHY Success\n"));
        
    }
    
    /* Add data to netif */
    netif_add(&gs_net_if, &x_ip_addr, &x_net_mask, &x_gateway, NULL, eth_arch_enetif_init, ethernet_input);
    
    /* Make it the default interface */
    netif_set_default(&gs_net_if);
    

    /* Set it up */
    //netif_set_up(&gs_net_if); //SD:: commented out to stop it trying to send packets before link is up. moved to ethernet_set_configuration
    return true;

}

/** \brief Configure the Ethernet subsystem. Should be called after init_ethernet()
 *
 */
void ethernet_configure_interface(const u8_t macAddress[], const char *hostname)
{
    LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE, ("Configure Interface()\n"));
    LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE, (" - MAC Address: %02lX:%02lX:%02lX:%02lX:%02lX:%02lX\n", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]));
    LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE, (" - HostName: %s\n", hostname));

    lpc_set_mac_address(macAddress);

#if LWIP_NETIF_HOSTNAME
    netif_set_hostname(&gs_net_if, hostname);
#endif
}

/* \brief Perform ethernet auto-negotiation and establish link. Returns true when ready
 *
 */

bool ethernet_establish_link(void)
{
    if(hwInit){ //ensure HW was successful
        return check_interface(&gs_net_if);
    }
    return false;
}

/* \brief Is the link still up? Also updates the interface status if the link has gone down
 *
 */
bool ethernet_link_established(void)
{
    if(hwInit){ //ensure HW was successful
        return check_interface(&gs_net_if);
    }
    return false;
    
}

/**
 *  \brief Manage the Ethernet packets, if any received process them.
 *  After processing any packets, manage the lwIP timers.
 *
 *  \return Returns true if data has been processed.
 */



volatile bool taskled = false;;
volatile uint32_t cn = 0;
void ethernet_task(void)
{
    if(!hwInit){
        return;
    }
    
#ifdef LED3
    if(cn >3000 ){
        if(taskled == true){
            pinModeDuet(LED3, OUTPUT_LOW, 0);
            taskled = false;
        }
        else{
            pinModeDuet(LED3, OUTPUT_HIGH, 0);
            taskled = true;
        }
        cn=0;
    } else {
        cn++;
    }
#endif
    
    //lpc_rx_queue will re-qeueu receive buffers. This normally occurs
    // automatically, but in systems were memory is constrained, pbufs
    // may not always be able to get allocated, so this function can be
    // optionally enabled to re-queue receive buffers.
#if 1
    while (lpc_rx_queue(&gs_net_if)) {}
#endif

    while(lpc_enetif_input(&gs_net_if)); //read all packets

    // Free TX buffers that are done sending
    lpc_tx_reclaim(&gs_net_if);
    
    sys_check_timeouts();

    
}

/*
 * \brief Sets the EMAC RX callback. It will be called when a new packet
 * can be processed and should be called with a NULL parameter inside
 * the actual callback.
 *
 * \param callback The callback to be called when a new packet is ready
 */
void ethernet_set_rx_callback(emac_dev_tx_cb_t callback)
{

    lpc_set_rx_callback(callback);
}

/*
 * \brief Returns the current IP address
 */

const uint8_t *ethernet_get_ipaddress()
{
    return (uint8_t*)&gs_net_if.ip_addr.addr;
}
