#include <stdio.h>
#include "lwip/netif.h"
#include "lwip/init.h"
#include "lwip/etharp.h"
#include "eth_driver.h"

//versatilepb maps LAN91C111 registers here
void * const eth0_addr = (void *) 0x10010000;

s_lan91c111_state sls = {.phy_address = 0,
                         .ever_sent_packet = 0, 
                         .tx_packet = 0, 
                         .irq_onoff = 0};

struct netif netif;

//feed frames from driver to LwIP
int process_frames(r16 * frame, int frame_len) {
  struct pbuf* p = pbuf_alloc(PBUF_RAW, frame_len, PBUF_POOL);
  if(p != NULL) {
    pbuf_take(p, frame, frame_len);
  }
  if(netif.input(p, &netif) != ERR_OK) {
    pbuf_free(p);
  }
}

//transmit frames from LwIP using driver
static err_t 
netif_output(struct netif *netif, struct pbuf *p)
{
  unsigned char mac_send_buffer[p->tot_len];
  pbuf_copy_partial(p, (void*)mac_send_buffer, p->tot_len, 0);
  nr_lan91c111_tx_frame(eth0_addr, &sls, mac_send_buffer, p->tot_len);
  return ERR_OK;
}

static err_t
netif_set_opts(struct netif *netif)
{
  netif->linkoutput = netif_output;
  netif->output     = etharp_output;
  netif->mtu        = 1500;
  netif->flags      = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;
  netif->hwaddr_len = 6;
  netif->hwaddr[0] = 0x00;
  netif->hwaddr[1] = 0x23;
  netif->hwaddr[2] = 0xC1;
  netif->hwaddr[3] = 0xDE;
  netif->hwaddr[4] = 0xD0;
  netif->hwaddr[5] = 0x0D;

  return ERR_OK;
}

void 
c_entry() {

  ip4_addr_t addr;
  ip4_addr_t netmask;
  ip4_addr_t gw;

  IP4_ADDR(&addr, 10, 0, 2, 99);
  IP4_ADDR(&netmask, 255, 255, 0, 0);
  IP4_ADDR(&gw, 10, 0, 0, 1);

  lwip_init();
  netif_add(&netif, &addr, &netmask, &gw, 
              NULL, netif_set_opts, netif_input);

  netif.name[0] = 'e';
  netif.name[1] = '0';
  netif_set_default(&netif);
  netif_set_up(&netif);

  nr_lan91c111_reset(eth0_addr, &sls, &sls);
  nr_lan91c111_set_promiscuous(eth0_addr,&sls,1);

  while(1) {

    nr_lan91c111_check_for_events(eth0_addr, &sls, process_frames);

  }
}

