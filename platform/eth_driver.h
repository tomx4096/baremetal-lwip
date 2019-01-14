#ifndef __lan91c111__
#define __lan91c111__

typedef unsigned char r8;
typedef unsigned short r16;
typedef unsigned long r32;

typedef void (ns_plugs_adapter_storage);
typedef void (ns_plugs_network_settings);

typedef struct {
  int phy_address;
  int ever_sent_packet;
  int tx_packet;
  int irq_onoff;
} s_lan91c111_state;

int nr_lan91c111_dump_registers
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage
        );

int nr_lan91c111_reset
        (
        void *hw_base_address,
        //ns_plugs_adapter_storage *adapter_storage,
        //ns_plugs_network_settings *s
        void *adapter_storage,
        void *s
        );

int nr_lan91c111_set_promiscuous
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage,
        int promiscuous_onoff
        );

int nr_lan91c111_check_for_events
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage,
        int (*process_frame)(r16 *, int)
        );

int nr_lan91c111_tx_frame
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage,
        const unsigned char *ethernet_frame,
        int frame_length
        );

#endif
