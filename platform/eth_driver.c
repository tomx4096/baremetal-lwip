// file: lan91c111.c
//
// This file implements the
// low level routines for an
// adapter that the "plugs"
// embedded tcp/ip stack can
// use.
//
// CBV - the memory in this device is divided into
// 4 segments of 2 k bytes each. Since the receive 
// part of the driver is interrupt driven and the
// transmit is not, one buffer is reserved for
// transmit, which never gets freed. The receive
// used the other 3 remaining buffers. The transmit
// buffer is allocated after a reset.
//
// ex:set tabstop=4:
// ex:set shiftwidth=4:
// ex:set expandtab:

//#include "excalibur.h"
//#include "plugs.h"
#include "eth_driver.h"
#include <stdio.h> 

// +--------------------------
// | Debug printing things
#define PLUGS_DEBUG 1

#if PLUGS_DEBUG
    #define dprint0 printf(" [lan91c111] ")
    #define dprint(x)    dprint0, printf(x), printf("\n")
    #define dprint1(x,y) dprint0, printf(x,y), printf("\n")
#else
    #define dprint0
    #define dprint(x)
    #define dprint1(x,y)
#endif

//typedef void (*ns_plugs_adapter_storage);
//typedef void (*ns_plugs_network_settings);

// +------------------------
// | Global storage allocation:
// | the size of one buffer.
// | We allocate it globally
// | (well, statically) so that
// | it's a fixed memory cost. It
// | used to be on the stack, but
// | that was obnoxious, because it
// | made sudden spikes on the
// | stack pointer. If you're doing
// | Ethernet, you need the buffer!
// |

       static r16 g_frame_buffer[768]; // 750 should be enough, 1500 byte max ethernet frame.

// +-------------------------
// | Here, a "frame" is the readable
// | bytes of an ethernet message. The
// | cs8900 data sheet defines a
// | "packet" as the whole ethernet
// | wire transmission, including
// | some link layer protocol that
// | we software folk can never see.
// | The "frame" is all the bytes,
// | including ethernet addresses,
// | type field, sometimes the CRC.
// |
// | (In the protocol stack code, we
// | just call any ol' bunch of bytes
// | a packet.)
// |

// +-----------------------------------------
// | Memory map of lan91c111 peripheral
// | It's a slightly interesting peripheral
// | because the registers use 4 overlays,
// | and the last register, np_bank, stays
// | in the same position for each overlay.
// |



#define LAN91C111_REGISTERS_OFFSET 0x0300
#define LAN91C111_DATA_BUS_WIDTH 16

/*
#if (!defined(LAN91C111_DATA_BUS_WIDTH)) || ((LAN91C111_DATA_BUS_WIDTH != 16) && (LAN91C111_DATA_BUS_WIDTH != 32))
    #error _LAN91C111_DATA_BUS_WIDTH must be defined to 16 or 32
#endif

#if (!defined(LAN91C111_REGISTERS_OFFSET))
    #error _LAN91C111_REGISTERS_OFFSET must be defined to 0 or 0x0300
#endif

#if LAN91C111_DATA_BUS_WIDTH == 32
    #define __lan91c111_register__ unsigned short
    #define __lan91c111_data_word_type__ volatile unsigned long
    #define __lan91c111_data_word_size__ 4
#else
*/
    #define __lan91c111_register__ uint16_t
    #define __lan91c111_data_word_type__ volatile unsigned short
    #define __lan91c111_data_word_size__ 2
//#endif

#if LAN91C111_REGISTERS_OFFSET > 0
    unsigned char blank[LAN91C111_REGISTERS_OFFSET];
#endif


typedef volatile struct 
    {
    // |
    // | There are two ways this chip can be wired up.
    // | One of the ways leaves the registers at
    // | integer boundaries: data bus width 16
    // |
    // | The other leaves the registers bunched together
    // | in pairs: data bus width 32
    // | 
    // | Also, the registers are located 0x0300 bytes into
    // | the address range of the device. In some designs,
    // | we have hardwired the upper bits of the address
    // | bus to 300, so that the software doesn't notice
    // | this. In other designs, we wire up all the address
    // | lines to the PLD. 
    // |
    // | Whatever.
    // |

    union
        {
        struct
            {
            __lan91c111_register__ np_tcr;
            __lan91c111_register__ np_eph_status;
            __lan91c111_register__ np_rcr;
            __lan91c111_register__ np_counter;
            __lan91c111_register__ np_mir;
            __lan91c111_register__ np_rpcr;
            __lan91c111_register__ np_reserved;
            __lan91c111_register__ np_bank;
            } bank_0;
        struct
            {
            __lan91c111_register__ np_config;
            __lan91c111_register__ np_base;
            __lan91c111_register__ np_ia0_1;
            __lan91c111_register__ np_ia2_3;
            __lan91c111_register__ np_ia4_5;
            __lan91c111_register__ np_general;
            __lan91c111_register__ np_control;
            __lan91c111_register__ np_bank;
            } bank_1;
        struct
            {
            __lan91c111_register__ np_mmu_command;
            __lan91c111_register__ np_pnr;
            __lan91c111_register__ np_fifo_ports;
            __lan91c111_register__ np_pointer;
            __lan91c111_register__ np_data;          // these two registers can be accessed together as
            __lan91c111_register__ np_data_2;        // 8, 16, or 32-bit reads, from the lowest address
            __lan91c111_register__ np_interrupt;
            __lan91c111_register__ np_bank;
            } bank_2;
        struct
            {
            __lan91c111_register__ np_mt0_1;
            __lan91c111_register__ np_mt2_3;
            __lan91c111_register__ np_mt4_5;
            __lan91c111_register__ np_mt6_7;
            __lan91c111_register__ np_mgmt;
            __lan91c111_register__ np_revision;
            __lan91c111_register__ np_ercv;
            __lan91c111_register__ np_bank;
            } bank_3;
        }; // unnamed union
    } np_lan91c111;

// Bit assignments:
/*
 . Bank Select Register: 
 .
 .        yyyy yyyy 0000 00xx  
 .        xx         = bank number
 .        yyyy yyyy    = 0x33, for identification purposes.
*/

// Transmit Control Register
/* BANK 0  */
#define TCR_ENABLE   0x0001    // When 1 we can transmit
#define TCR_LOOP     0x0002    // Controls output pin LBK
#define TCR_FORCOL   0x0004    // When 1 will force a collision
#define TCR_PAD_EN   0x0080    // When 1 will pad tx frames < 64 bytes w/0
#define TCR_NOCRC    0x0100    // When 1 will not append CRC to tx frames
#define TCR_MON_CSN  0x0400    // When 1 tx monitors carrier
#define TCR_FDUPLX   0x0800    // When 1 enables full duplex operation
#define TCR_STP_SQET 0x1000    // When 1 stops tx if Signal Quality Error
#define TCR_EPH_LOOP 0x2000    // When 1 enables EPH block loopback
#define TCR_SWFDUP   0x8000    // When 1 enables Switched Full Duplex mode

#define    TCR_CLEAR    0    /* do NOTHING */
#define    TCR_DEFAULT      (TCR_ENABLE | TCR_PAD_EN)

// EPH Status Register
/* BANK 0  */
#define ES_TX_SUC   0x0001    // Last TX was successful
#define ES_SNGL_COL 0x0002    // Single collision detected for last tx
#define ES_MUL_COL  0x0004    // Multiple collisions detected for last tx
#define ES_LTX_MULT 0x0008    // Last tx was a multicast
#define ES_16COL    0x0010    // 16 Collisions Reached
#define ES_SQET     0x0020    // Signal Quality Error Test
#define ES_LTXBRD   0x0040    // Last tx was a broadcast
#define ES_TXDEFR   0x0080    // Transmit Deferred
#define ES_LATCOL   0x0200    // Late collision detected on last tx
#define ES_LOSTCARR 0x0400    // Lost Carrier Sense
#define ES_EXC_DEF  0x0800    // Excessive Deferral
#define ES_CTR_ROL  0x1000    // Counter Roll Over indication
#define ES_LINK_OK  0x4000    // Driven by inverted value of nLNK pin
#define ES_TXUNRN   0x8000    // Tx Underrun

// Receive Control Register
/* BANK 0  */
#define RCR_RX_ABORT  0x0001 // Set if a rx frame was aborted
#define RCR_PRMS      0x0002 // Enable promiscuous mode
#define RCR_ALMUL     0x0004 // When set accepts all multicast frames
#define RCR_RXEN      0x0100 // IFF this is set, we can receive packets
#define RCR_STRIP_CRC 0x0200 // When set strips CRC from rx packets
#define RCR_ABORT_ENB 0x2000 // When set will abort rx on collision 
#define RCR_FILT_CAR  0x4000 // When set filters leading 12 bit s of carrier
#define RCR_SOFTRST   0x8000 // resets the chip

/* the normal settings for the RCR register : */
#define    RCR_DEFAULT    (RCR_STRIP_CRC | RCR_RXEN )
#define RCR_CLEAR    0x0    // set it to a base state

// Receive/Phy Control Register
/* BANK 0  */
#define RPC_SPEED        0x2000    // When 1 PHY is in 100Mbps mode.
#define RPC_DPLX         0x1000    // When 1 PHY is in Full-Duplex Mode
#define RPC_ANEG         0x0800    // When 1 PHY is in Auto-Negotiate Mode
#define RPC_LSXA_SHFT    5         // Bits to shift LS2A,LS1A,LS0A to lsb
#define RPC_LSXB_SHFT    2         // Bits to get LS2B,LS1B,LS0B to lsb
#define RPC_LED_100_10   (0x00)    // LED = 100Mbps OR's with 10Mbps link detect
#define RPC_LED_RES      (0x01)    // LED = Reserved
#define RPC_LED_10       (0x02)    // LED = 10Mbps link detect
#define RPC_LED_FD       (0x03)    // LED = Full Duplex Mode
#define RPC_LED_TX_RX    (0x04)    // LED = TX or RX packet occurred
#define RPC_LED_100      (0x05)    // LED = 100Mbps link dectect
#define RPC_LED_RX       (0x06)    // LED = TX packet occurred
#define RPC_LED_TX       (0x07)    // LED = RX packet occurred

#define RPC_DEFAULT  (RPC_ANEG                   \
            | (RPC_LED_100_10 << RPC_LSXB_SHFT)  \
            | (RPC_LED_RX << RPC_LSXA_SHFT)      \
            | RPC_SPEED | RPC_DPLX)


// Configuration Reg
/* BANK 1 */
#define CONFIG_EXT_PHY      0x0200 // 1=external MII, 0=internal Phy
#define CONFIG_GPCNTRL      0x0400 // Inverse value drives pin nCNTRL
#define CONFIG_NO_WAIT      0x1000 // When 1 no extra wait states on ISA bus
#define CONFIG_EPH_POWER_EN 0x8000 // When 0 EPH is placed into low power mode.

// Default is powered-up, Internal Phy, Wait States, and pin nCNTRL=low

#define CONFIG_DEFAULT    (CONFIG_EPH_POWER_EN)

// Control Register
/* BANK 1 */

#define CTL_RCV_BAD       0x4000 // When 1 bad CRC packets are received
#define CTL_AUTO_RELEASE  0x0800 // When 1 tx pages are released automatically
#define CTL_LE_ENABLE     0x0080 // When 1 enables Link Error interrupt
#define CTL_CR_ENABLE     0x0040 // When 1 enables Counter Rollover interrupt
#define CTL_TE_ENABLE     0x0020 // When 1 enables Transmit Error interrupt
#define CTL_EEPROM_SELECT 0x0004 // Controls EEPROM reload & store
#define CTL_RELOAD        0x0002 // When set reads EEPROM into registers
#define CTL_STORE         0x0001 // When set stores registers into EEPROM


// MMU Command Register
/* BANK 2 */

#define MC_BUSY      1      // When 1 the last release has not completed
#define MC_NOP       (0<<5) // No Op
#define MC_ALLOC     (1<<5) // OR with number of 256 byte packets
#define MC_RESET     (2<<5) // Reset MMU to initial state
#define MC_REMOVE    (3<<5) // Remove the current rx packet
#define MC_RELEASE   (4<<5) // Remove and release the current rx packet
#define MC_FREEPKT   (5<<5) // Release packet in PNR register
#define MC_ENQUEUE   (6<<5) // Enqueue the packet for transmit
#define MC_RSTTXFIFO (7<<5) // Reset the TX FIFOs


// Allocation Result Register
/* BANK 2 */
#define AR_FAILED    0x80    // Alocation Failed


// RX FIFO Ports Register
/* BANK 2 */
#define RXFIFO_REMPTY    0x8000    // RX FIFO Empty


// TX FIFO Ports Register
/* BANK 2 */
#define TXFIFO_TEMPTY    0x80    // TX FIFO Empty

// Pointer Register
/* BANK 2 */
#define    PTR_RCV        0x8000 // 1=Receive area, 0=Transmit area
#define    PTR_AUTOINC     0x4000 // Auto increment the pointer on each access
#define PTR_READ    0x2000 // When 1 the operation is a read


// Interrupt Mask Register
/* BANK 2 */
#define IM_MDINT        0x80 // PHY MI Register 18 Interrupt
#define IM_ERCV_INT     0x40 // Early Receive Interrupt
#define IM_EPH_INT      0x20 // Set by Etheret Protocol Handler section
#define IM_RX_OVRN_INT  0x10 // Set by Receiver Overruns
#define IM_ALLOC_INT    0x08 // Set when allocation request is completed
#define IM_TX_EMPTY_INT 0x04 // Set if the TX FIFO goes empty
#define IM_TX_INT       0x02 // Transmit Interrrupt
#define IM_RCV_INT      0x01 // Receive Interrupt


// Management Interface Register (MII)
/* BANK 3 */
#define MII_MSK_CRS100 0x4000 // Disables CRS100 detection during tx half dup
#define MII_MDOE       0x0008 // MII Output Enable
#define MII_MCLK       0x0004 // MII Clock, pin MDCLK
#define MII_MDI        0x0002 // MII Input, pin MDI
#define MII_MDO        0x0001 // MII Output, pin MDO


// Early RCV Register
/* BANK 3 */
/* this is NOT on SMC9192 */
#define ERCV_RCV_DISCRD 0x0080 // When 1 discards a packet being received
#define ERCV_THRESHOLD  0x001F // ERCV Threshold Mask

#define CHIP_9194    4
#define CHIP_9195    5
#define CHIP_9196    6
#define CHIP_91100   7
#define CHIP_91100FD 8
#define CHIP_91111FD 9

static const char * chip_ids[ 16 ] =
    { 
    0,0,0,
    /* 3 */ "SMC91C90/91C92",
    /* 4 */ "SMC91C94",
    /* 5 */ "SMC91C95",
    /* 6 */ "SMC91C96",
    /* 7 */ "SMC91C100", 
    /* 8 */ "SMC91C100FD", 
    /* 9 */ "SMC91C11xFD", 
    0,0,0,0,0,0
    };

/* 
 . Transmit status bits 
*/
#define TS_SUCCESS 0x0001
#define TS_LOSTCAR 0x0400
#define TS_LATCOL  0x0200
#define TS_16COL   0x0010

/*
 . Receive status bits
*/
#define RS_ALGNERR   0x8000
#define RS_BRODCAST  0x4000
#define RS_BADCRC    0x2000
#define RS_ODDFRAME  0x1000    // bug: the LAN91C111 never sets this on receive
#define RS_TOOLONG   0x0800
#define RS_TOOSHORT  0x0400
#define RS_MULTICAST 0x0001
#define RS_ERRORS    (RS_ALGNERR | RS_BADCRC | RS_TOOLONG | RS_TOOSHORT) 


// PHY Types
enum {
    PHY_LAN83C183 = 1,    // LAN91C111 Internal PHY
    PHY_LAN83C180
};


// PHY Register Addresses (LAN91C111 Internal PHY)

// PHY Control Register
#define PHY_CNTL_REG      0x00
#define PHY_CNTL_RST      0x8000 // 1=PHY Reset
#define PHY_CNTL_LPBK     0x4000 // 1=PHY Loopback
#define PHY_CNTL_SPEED    0x2000 // 1=100Mbps, 0=10Mpbs
#define PHY_CNTL_ANEG_EN  0x1000 // 1=Enable Auto negotiation
#define PHY_CNTL_PDN      0x0800 // 1=PHY Power Down mode
#define PHY_CNTL_MII_DIS  0x0400 // 1=MII 4 bit interface disabled
#define PHY_CNTL_ANEG_RST 0x0200 // 1=Reset Auto negotiate
#define PHY_CNTL_DPLX     0x0100 // 1=Full Duplex, 0=Half Duplex
#define PHY_CNTL_COLTST   0x0080 // 1= MII Colision Test

// PHY Status Register
#define PHY_STAT_REG      0x01
#define PHY_STAT_CAP_T4   0x8000    // 1=100Base-T4 capable
#define PHY_STAT_CAP_TXF  0x4000    // 1=100Base-X full duplex capable
#define PHY_STAT_CAP_TXH  0x2000    // 1=100Base-X half duplex capable
#define PHY_STAT_CAP_TF   0x1000    // 1=10Mbps full duplex capable
#define PHY_STAT_CAP_TH   0x0800    // 1=10Mbps half duplex capable
#define PHY_STAT_CAP_SUPR 0x0040    // 1=recv mgmt frames with not preamble
#define PHY_STAT_ANEG_ACK 0x0020    // 1=ANEG has completed
#define PHY_STAT_REM_FLT  0x0010    // 1=Remote Fault detected
#define PHY_STAT_CAP_ANEG 0x0008    // 1=Auto negotiate capable
#define PHY_STAT_LINK     0x0004    // 1=valid link
#define PHY_STAT_JAB      0x0002    // 1=10Mbps jabber condition
#define PHY_STAT_EXREG    0x0001    // 1=extended registers implemented

// PHY Identifier Registers
#define PHY_ID1_REG       0x02    // PHY Identifier 1
#define PHY_ID2_REG       0x03    // PHY Identifier 2

// PHY Auto-Negotiation Advertisement Register
#define PHY_AD_REG        0x04
#define PHY_AD_NP         0x8000    // 1=PHY requests exchange of Next Page
#define PHY_AD_ACK        0x4000    // 1=got link code word from remote
#define PHY_AD_RF         0x2000    // 1=advertise remote fault
#define PHY_AD_T4         0x0200    // 1=PHY is capable of 100Base-T4
#define PHY_AD_TX_FDX     0x0100    // 1=PHY is capable of 100Base-TX FDPLX
#define PHY_AD_TX_HDX     0x0080    // 1=PHY is capable of 100Base-TX HDPLX
#define PHY_AD_10_FDX     0x0040    // 1=PHY is capable of 10Base-T FDPLX
#define PHY_AD_10_HDX     0x0020    // 1=PHY is capable of 10Base-T HDPLX
#define PHY_AD_CSMA       0x0001    // 1=PHY is capable of 802.3 CMSA

// PHY Auto-negotiation Remote End Capability Register
#define PHY_RMT_REG        0x05
// Uses same bit definitions as PHY_AD_REG

// PHY Configuration Register 1
#define PHY_CFG1_REG        0x10
#define PHY_CFG1_LNKDIS     0x8000    // 1=Rx Link Detect Function disabled
#define PHY_CFG1_XMTDIS     0x4000    // 1=TP Transmitter Disabled
#define PHY_CFG1_XMTPDN     0x2000    // 1=TP Transmitter Powered Down
#define PHY_CFG1_BYPSCR     0x0400    // 1=Bypass scrambler/descrambler
#define PHY_CFG1_UNSCDS     0x0200    // 1=Unscramble Idle Reception Disable
#define PHY_CFG1_EQLZR      0x0100    // 1=Rx Equalizer Disabled
#define PHY_CFG1_CABLE      0x0080    // 1=STP(150ohm), 0=UTP(100ohm)
#define PHY_CFG1_RLVL0      0x0040    // 1=Rx Squelch level reduced by 4.5db
#define PHY_CFG1_TLVL_SHIFT 2         // Transmit Output Level Adjust
#define PHY_CFG1_TLVL_MASK  0x003C
#define PHY_CFG1_TRF_MASK   0x0003    // Transmitter Rise/Fall time


// PHY Configuration Register 2
#define PHY_CFG2_REG        0x11
#define PHY_CFG2_APOLDIS    0x0020    // 1=Auto Polarity Correction disabled
#define PHY_CFG2_JABDIS     0x0010    // 1=Jabber disabled
#define PHY_CFG2_MREG       0x0008    // 1=Multiple register access (MII mgt)
#define PHY_CFG2_INTMDIO    0x0004    // 1=Interrupt signaled with MDIO pulseo

// PHY Status Output (and Interrupt status) Register
#define PHY_INT_REG      0x12    // Status Output (Interrupt Status)
#define PHY_INT_INT      0x8000  // 1=bits have changed since last read
#define PHY_INT_LNKFAIL  0x4000  // 1=Link Not detected
#define PHY_INT_LOSSSYNC 0x2000  // 1=Descrambler has lost sync
#define PHY_INT_CWRD     0x1000  // 1=Invalid 4B5B code detected on rx
#define PHY_INT_SSD      0x0800  // 1=No Start Of Stream detected on rx
#define PHY_INT_ESD      0x0400  // 1=No End Of Stream detected on rx
#define PHY_INT_RPOL     0x0200  // 1=Reverse Polarity detected
#define PHY_INT_JAB      0x0100  // 1=Jabber detected
#define PHY_INT_SPDDET   0x0080  // 1=100Base-TX mode, 0=10Base-T mode
#define PHY_INT_DPLXDET  0x0040  // 1=Device in Full Duplex

// PHY Interrupt/Status Mask Register
#define PHY_MASK_REG        0x13    // Interrupt Mask
// Uses the same bit definitions as PHY_INT_REG

#define MEMORY_WAIT_TIME    1000000

#define swap_bytes(_r) ( (((_r) & 0xff00) >> 8) | (((_r) & 0x00ff) << 8) )

/*-------------------------------------------------------------------------
 .  I define some macros to make it easier to do somewhat common
 . or slightly complicated, repeated tasks. 
 --------------------------------------------------------------------------*/

/* select a register bank, 0 to 3  */

#define LAN91C111_SELECT_BANK(x, lan_base_addr)  { \
        ((np_lan91c111*) (lan_base_addr))->bank_0.np_bank = (x); }


/* 
   Acknowledging interrupts is tricky.  The "np_interrupt" register
   appears in our structure as a 16-bit register, but it's really
   two 8-bit registers (interrupt-mask and interrupt-acknowledge)
   when you write to it.  When we acknowledge an interrupt, we don't 
   want to mess with the mask-byte.  That's OK: Just pretend, just 
   this once, that the acknowledge-register was a byte, and do 
   a byte-write to it.
*/   
#define LAN91C111_ACKNOWLEDGE_INTERRUPT(lan_base_addr, ack_mask)  { \
        LAN91C111_SELECT_BANK(2, lan_base_addr);                    \
        *( (volatile unsigned char*) &(                             \
          (((np_lan91c111*) (lan_base_addr))->bank_2.np_interrupt)  \
         ))= (ack_mask); }

/*----------------------------------------------------------------------
 . Define the interrupts that I want to receive from the card
 . 
 . I want: 
 .  IM_EPH_INT, for nasty errors
 .  IM_RCV_INT, for happy received packets
 --------------------------------------------------------------------------*/

#define LAN91C111_INTERRUPT_MASK   (IM_EPH_INT | IM_RCV_INT ) 


// +--------------------------------
// | Adapter Storage
// |
// | Each adapter gets a 4 longs
// | of storage, passed to each of
// | the adapter routines.
// |
// | We alway map it to our little
// | structure here.
// |
// | (If your adapter needed more than
// | four longs, you could malloc a
// | block and store the pointer.)
// |
/*
typedef struct {
  int phy_address;
  int ever_sent_packet;
  int tx_packet;
  int irq_onoff;
} s_lan91c111_state;
*/
// +-------------------------------
// | Local Prototypes
// |

static void sft_loop_delay (int multiplier);
static void nr_set_multicast (void *hw_base_address);

static int r_allocate_tx_packet
        (
        np_lan91c111 *e,
        s_lan91c111_state *sls
        );

static int r_lan91c111_detect_phy
        (
        void *hardware_base_address,
        int *phy_address_out
        );

static int r_lan91c111_init_phy
        (
        void *hardware_base_address,
        int *phy_address_out
        );

static void r_write_phy_register
        (
        void *hardware_base_address,
        int phy_address,
        r8 phyreg,
        r16 phydata
        );

static r16 r_read_phy_register
        (
        void *hardware_base_address,
        int phy_address,
        r8 phyreg
        );

//TODO .... -tomx
void nr_delay(int num) {return ;}
int nr_setirqenable(int num) {return 0;}

/*
 . Function: r_lan91c111_enable
 . Purpose: let the chip talk to the outside work
 . Method:
 .    1.  Enable the transmitter
 .    2.  Enable the receiver
*/
static void r_lan91c111_enable (void *hw_base_address)
{
    np_lan91c111 *e = hw_base_address;

    LAN91C111_SELECT_BANK( 0 , hw_base_address);
    /* see the header file for options in TCR/RCR DEFAULT*/
    
    e->bank_0.np_tcr = TCR_DEFAULT;
    e->bank_0.np_rcr = RCR_DEFAULT;

    e->bank_0.np_bank = 2;
    e->bank_2.np_interrupt = 0; // polled mode
}

void sft_loop_delay (int multiplier)
{
    long volatile count, c;

    count = 100 * multiplier;

    for (c = 0; c < count; c++ ) {
    }
}

void nr_set_multicast (void *hw_base_address)
{
    np_lan91c111 *e = hw_base_address;
    LAN91C111_SELECT_BANK( 3 , hw_base_address);

    e->bank_3.np_mt0_1 = 0xFFFF;
    e->bank_3.np_mt2_3 = 0xFFFF;
    e->bank_3.np_mt4_5 = 0xFFFF;
    e->bank_3.np_mt6_7 = 0xFFFF;
}

// CBV - reset function. This function combines about 3 functions
// from the original driver: probe, init and reset.

int nr_lan91c111_reset
        (
        void *hw_base_address,
        //ns_plugs_adapter_storage *adapter_storage,
        //ns_plugs_network_settings *s
        void *adapter_storage,
        void *s
        )
{
    np_lan91c111 *e = hw_base_address;
    int timeout;
    r16        status;
    int result = 0;
    s_lan91c111_state *sls = (s_lan91c111_state *)adapter_storage;

        {
        // | First thing: disable CPU interrupts for just a moment,
        // | and poke the chip registers that should disable its
        // | interrupt generation...

        //int old_irqenable = nr_setirqenable(0);  // turn of IRQs until we are ready!

        e->bank_0.np_bank = 0;
        e->bank_0.np_rcr = RCR_SOFTRST;  // soft reset
        //nr_delay(1);                     // a very generous reset pulse
        e->bank_0.np_rcr = 0;            // done with soft reset
        //nr_delay(1);                     // and wait a while afterwards

        //nr_setirqenable(old_irqenable);  // restore IRQs to their previous state.
        }


    /* Initialize the chip: first, lets make sure that we have
     * what we think we have:
     */

    LAN91C111_SELECT_BANK(0, hw_base_address);
    if ((e->bank_0.np_bank & 0xFF00) != 0x3300)
    {
        dprint("nr_lan91c111_reset: illegal bank register value");
        result = -1;
        goto go_home;
    }
    
    /* check the revision register. */

    LAN91C111_SELECT_BANK(3, hw_base_address);
    // printf("revision is: %X\n", e->bank_3.np_revision);

    if (!chip_ids[(e->bank_3.np_revision>> 4) & 0x0F])
    {
        dprint("nr_lan91c111_reset: unrecognized chip revision");
        result = -1;
        goto go_home;
    }

    printf("nr_lan91c111_reset: chip id = %s\n",
            chip_ids[(e->bank_3.np_revision>> 4) & 0x0F]);

    

    // +------------------------------------

    LAN91C111_SELECT_BANK(0, hw_base_address);
    
    e->bank_0.np_rcr = RCR_SOFTRST;  /* soft reset */ 

    /* Setup the Configuration Register */
    /* This is necessary because the CONFIG_REG is not affected */
    /* by a soft reset */

    LAN91C111_SELECT_BANK(1, hw_base_address);
    e->bank_1.np_config = CONFIG_DEFAULT; /* NOT in power save mode */

    LAN91C111_SELECT_BANK(0, hw_base_address);

    /* this should pause enough for the chip to be happy */
    //nr_delay(5);

    /* Disable transmit and receive functionality */
    e->bank_0.np_rcr = RCR_CLEAR;
    e->bank_0.np_tcr = TCR_CLEAR;

    /* DONT set the control register to automatically
       release successfully transmitted packets.
           We like it ever so much when we have our one-and-only
           packet allocated for transmit.
           */
    LAN91C111_SELECT_BANK(1, hw_base_address);
    e->bank_1.np_control &= (~CTL_AUTO_RELEASE);
    nr_delay(5);

    /* Reset the MMU */
    LAN91C111_SELECT_BANK(2, hw_base_address);
    e->bank_2.np_mmu_command = MC_RESET;

    /* Disable all interrupts */
    e->bank_2.np_interrupt = 0;

    // | 1: set the mac address

    //not part of port -tomx
/*
        {
        host_32 u32;
        host_16 l16;

        u32 = nr_n2h32(s->ethernet_address.u32);
        l16 = nr_n2h16(s->ethernet_address.l16);

        LAN91C111_SELECT_BANK(1, hw_base_address);
        e->bank_1.np_ia0_1 = swap_bytes(((u32 >> 16) & 0x0000ffff));
        e->bank_1.np_ia2_3 = swap_bytes((u32 & 0x0000ffff)); 
        e->bank_1.np_ia4_5 = swap_bytes(l16);
        }
*/

    /* Intialize the rpcr register */

    //nr_lan91c111_set_led(hw_base_address,adapter_storage,0);

    /* Now that its stable, enable the chip: */

    r_lan91c111_enable(hw_base_address);


    /* no phy to init in qemu ! tomx
    result = r_lan91c111_init_phy
            (
            hw_base_address,
            &sls->phy_address
            );
    */

    if(result) {
          dprint ("Phy initialization failed.");
          goto go_home;
        }

        /* Allocate a transmit-packet inside the MAC.
           This is the only one we're ever going to get..take good
           care of it.  */
        result = r_allocate_tx_packet(hw_base_address, sls);
        if (result) 
          {
            printf ("TX-packet allocation failed.\n");
            goto go_home;
          }

go_home:
    return result;
}

int nr_lan91c111_dump_registers
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage
        )
{
    np_lan91c111 *e = hardware_base_address;
    r16 phy_reg_val;
    s_lan91c111_state *sls = (s_lan91c111_state *)adapter_storage;
    int pa = sls->phy_address;

#if PLUGS_DEBUG
    printf("-------------------------\n");

    printf("Hardware base address = 0x%08x \n",hardware_base_address);
    printf("\nBANK 0 registers:\n\n");
    e->bank_0.np_bank = 0;               // select bank zero:
    printf("np_tcr         = 0x%04x\n", e->bank_0.np_tcr);
    printf("np_eph_status  = 0x%04x\n", e->bank_0.np_eph_status);
    printf("np_rcr         = 0x%04x\n", e->bank_0.np_rcr);
    printf("np_counter     = 0x%04x\n", e->bank_0.np_counter);
    printf("np_mir         = 0x%04x\n", e->bank_0.np_mir);
    printf("np_rpcr        = 0x%04x\n", e->bank_0.np_rpcr);

    printf("\nBANK 1 registers:\n\n");
    e->bank_0.np_bank = 1;                       // select bank one:
    printf("np_config      = 0x%04x\n", e->bank_1.np_config);
    printf("np_base        = 0x%04x\n", e->bank_1.np_base);
    printf("np_ia0_1       = 0x%04x\n", e->bank_1.np_ia0_1);
    printf("np_ia2_3       = 0x%04x\n", e->bank_1.np_ia2_3);
    printf("np_ia4_5       = 0x%04x\n", e->bank_1.np_ia4_5);
    printf("np_general     = 0x%04x\n", e->bank_1.np_general);
    printf("np_control     = 0x%04x\n", e->bank_1.np_control);

    printf("\nBANK 2 registers:\n\n");
    e->bank_1.np_bank = 2;                       // select bank two:
    printf("np_mmu_command = 0x%04x\n", e->bank_2.np_mmu_command);
    printf("np_pnr         = 0x%04x\n", e->bank_2.np_pnr);
    printf("np_fifo_ports  = 0x%04x\n", e->bank_2.np_fifo_ports);
    printf("np_pointer     = 0x%04x\n", e->bank_2.np_pointer);
    printf("np_data        = 0x%04x\n", e->bank_2.np_data);
    printf("np_data_2      = 0x%04x\n", e->bank_2.np_data_2);
    printf("np_interrupt   = 0x%04x\n", e->bank_2.np_interrupt);

    printf("\nBANK 3 registers:\n\n");
    e->bank_2.np_bank = 3;                       // select bank three:
    printf("np_mt0_1       = 0x%04x\n", e->bank_3.np_mt0_1);
    printf("np_mt2_3       = 0x%04x\n", e->bank_3.np_mt2_3);
    printf("np_mt4_5       = 0x%04x\n", e->bank_3.np_mt4_5);
    printf("np_mt6_7       = 0x%04x\n", e->bank_3.np_mt6_7);
    printf("np_mgmt        = 0x%04x\n", e->bank_3.np_mgmt);
    printf("np_revision    = 0x%04x\n", e->bank_3.np_revision);
    printf("np_ercv        = 0x%04x\n", e->bank_3.np_ercv);

    printf("\n PHY registers: \n");
    phy_reg_val = r_read_phy_register(hardware_base_address,pa, PHY_CNTL_REG);
    printf("Control        = 0x%04x \n", phy_reg_val);
    
    phy_reg_val = r_read_phy_register(hardware_base_address,pa, PHY_STAT_REG);
    printf("Status         = 0x%04x \n", phy_reg_val);

    phy_reg_val = r_read_phy_register(hardware_base_address,pa, PHY_ID1_REG);
    printf("ID1            = 0x%04x \n", phy_reg_val);

    phy_reg_val = r_read_phy_register(hardware_base_address,pa, PHY_ID2_REG);
    printf("ID2            = 0x%04x \n", phy_reg_val);

    phy_reg_val = r_read_phy_register(hardware_base_address,pa, PHY_AD_REG);
    printf("Autoneg        = 0x%04x \n", phy_reg_val);

    phy_reg_val = r_read_phy_register(hardware_base_address,pa, PHY_RMT_REG);
    printf("Autoneg Remote = 0x%04x \n", phy_reg_val);

    phy_reg_val = r_read_phy_register(hardware_base_address,pa, PHY_CFG1_REG);
    printf("Config 1       = 0x%04x \n", phy_reg_val);

    phy_reg_val = r_read_phy_register(hardware_base_address,pa, PHY_CFG2_REG);
    printf("Config 2       = 0x%04x \n", phy_reg_val);

    phy_reg_val = r_read_phy_register(hardware_base_address,pa, PHY_INT_REG);
    printf("Interrupt Sts  = 0x%04x \n", phy_reg_val);

    phy_reg_val = r_read_phy_register(hardware_base_address,pa, PHY_MASK_REG);
    printf("Interrupt Mask = 0x%04x \n\n", phy_reg_val);

#endif
    return 0;
}



// +-------------------------------
// | r_lan91c111_init_phy(hw,phy_out)
// |
// | Invokes several other routines to:
// |  locate the phy, using one of 32
// |  "phy addresses", and then some
// |  serializing bit-twiddling routines
// |  that talk to the phy through some
// |  obscure registers.
// |
// |  we return the phy address, in case
// |  whoever called us wants to talk to
// |  it again. (They shouldnt need to.)
// |
int r_lan91c111_init_phy
        (
        void *hardware_base_address,
        int *phy_address_out
        )
{
    int timeout;
    int timeout1;
    r16 status = 0;
    r16 i;
    int failed = 0;
    r16 my_phy_caps; // My PHY capabilities
    r16 my_ad_caps; // My Advertised capabilities        
    r16 rpc_cur_mode = RPC_DEFAULT;
    int result = 0; 
    int pa; // short name for phy address since we use it lots

    r16 lastPhy18; // used to be a global, thanks cbv

    np_lan91c111 *e = hardware_base_address;

    // Find the address and type of our phy

    result = r_lan91c111_detect_phy(hardware_base_address,phy_address_out);
    if(result)
        {
        dprint("r_lan91c111_init_phy: phy address not found");
        goto go_home;
        }

    pa = *phy_address_out;

    // Reset the PHY, setting all other bits to zero

    r_write_phy_register (hardware_base_address,pa,PHY_CNTL_REG,PHY_CNTL_RST);
    
    // Wait for the reset to complete, or time out

    timeout = 60; // Wait up to 3 seconds
    while (timeout--)
        {
        if (!(r_read_phy_register(hardware_base_address,pa,PHY_CNTL_REG)
            & PHY_CNTL_RST))
            {
            // reset complete
            break;
            }
        }

    if (timeout < 1)
        {
        dprint("r_lan91c111_init_phy: phy stuck in reset");
        result = -1;
        goto go_home;
        }
    
    // Disable Autoneg - it should be zero already, but lets follow
    // the algorithm:
    r_write_phy_register(hardware_base_address,pa, PHY_CNTL_REG, 0);

    // Start the Autonegotiation:
    LAN91C111_SELECT_BANK(0, hardware_base_address);
    e->bank_0.np_rpcr = RPC_DEFAULT | RPC_ANEG;

    my_ad_caps = r_read_phy_register(hardware_base_address,pa,PHY_AD_REG);
    my_ad_caps |= PHY_AD_T4 | PHY_AD_TX_FDX | PHY_AD_TX_HDX |
            PHY_AD_10_FDX | PHY_AD_10_HDX | PHY_AD_CSMA;

    // Update our Auto-Neg Advertisement Register
    r_write_phy_register(hardware_base_address,pa, PHY_AD_REG, my_ad_caps);

    // Restart auto-negotiation process in order to advertise my caps
    r_write_phy_register (hardware_base_address,pa, PHY_CNTL_REG,
        PHY_CNTL_ANEG_EN | PHY_CNTL_ANEG_RST | PHY_CNTL_SPEED |
        PHY_CNTL_DPLX);

    // Wait for the auto-negotiation to complete.  This may take from
    // 2 to 3 seconds.
    // Wait for the reset to complete, or time out
    timeout = 600; // Wait up to 10 seconds
    timeout1 = 0;
    while (timeout--)
    {
        status = r_read_phy_register(hardware_base_address,pa,PHY_STAT_REG);
        if (status & PHY_STAT_ANEG_ACK)
            {
            // auto-negotiate complete
            timeout1 = -1;
            }

        // Restart auto-negotiation if remote fault
        if (status & PHY_STAT_REM_FLT)
            {
            timeout1 = 0;  /* dont break out quite yet... */
            dprint("r_lan91c111_init_phy: phy remote fault, renegotiating");

            // Restart auto-negotiation!
            r_write_phy_register(hardware_base_address,pa, PHY_CNTL_REG,
                PHY_CNTL_ANEG_EN | PHY_CNTL_ANEG_RST |
                PHY_CNTL_SPEED | PHY_CNTL_DPLX);
            }
        if (timeout1) break;
    }

    // CBV - this register takes a few reads to stabilize - if we
    // read it only once we get the result it had BEFORE autoneg.
    // 5 times seems to do the trick

    nr_delay(3);    // a little while to settle
    for (i=0; i< 5; i++)
    {
        nr_delay(1);    // a little while to settle
        lastPhy18 = r_read_phy_register(hardware_base_address,pa,PHY_INT_REG);
    }

    if (timeout < 1)
    {
        dprint("r_lan91c111_init_phy: phy negotiation timed out");
        failed = 1;
    }

    // Fail if we detected an auto-negotiate remote fault
    if (status & PHY_STAT_REM_FLT)
        {
        dprint("r_lan91c111_init_phy: remote fault");
        failed = 1;
        }

    // Set our sysctl parameters to match auto-negotiation results
    if ( lastPhy18 & PHY_INT_SPDDET )
        {
        dprint("r_lan91c111_init_phy: 100bt");
        rpc_cur_mode |= RPC_SPEED;
        }
    else
        {
        dprint("r_lan91c111_init_phy: 10bt");
        rpc_cur_mode &= ~RPC_SPEED;
        }

    if ( lastPhy18 & PHY_INT_DPLXDET )
        {
        dprint("r_lan91c111_init_phy: full duplex");
        rpc_cur_mode |= RPC_DPLX;
        }
    else
        {
        dprint("r_lan91c111_init_phy: half duplex");
        rpc_cur_mode &= ~RPC_DPLX;
        }

    // Re-Configure the Receive/Phy Control register
    e->bank_0.np_rpcr = rpc_cur_mode;


go_home:
    return result;
}

/*------------------------------------------------------------
 . Reads a register from the MII Management serial interface
 .-------------------------------------------------------------*/
static r16 r_read_phy_register
        (
        void *hardware_base_address,
        int phy_address,
        r8 phyreg
        )
    {
    int old_bank;    // | so we may politely restore it
    unsigned int i;
    r8 mask;
    r16 mii_reg;
    r8 bits[64];
    int clk_idx = 0;
    int input_idx;
    r16 phydata;
    np_lan91c111 *e = hardware_base_address;

    // 32 consecutive ones on MDO to establish sync
    for (i = 0; i < 32; ++i)
        bits[clk_idx++] = MII_MDOE | MII_MDO;

    // Start code <01>
    bits[clk_idx++] = MII_MDOE;
    bits[clk_idx++] = MII_MDOE | MII_MDO;

    // Read command <10>
    bits[clk_idx++] = MII_MDOE | MII_MDO;
    bits[clk_idx++] = MII_MDOE;

    // Output the PHY address, msb first
    mask = (r8)0x10;
    for (i = 0; i < 5; ++i)
        {
        if (phy_address & mask)
            bits[clk_idx++] = MII_MDOE | MII_MDO;
        else
            bits[clk_idx++] = MII_MDOE;

        // Shift to next lowest bit
        mask >>= 1;
        }

    // Output the phy register number, msb first
    mask = (r8)0x10;
    for (i = 0; i < 5; ++i)
        {
        if (phyreg & mask)
            bits[clk_idx++] = MII_MDOE | MII_MDO;
        else
            bits[clk_idx++] = MII_MDOE;

        // Shift to next lowest bit
        mask >>= 1;
        }

    // Tristate and turnaround (2 bit times)
    bits[clk_idx++] = 0;
    //bits[clk_idx++] = 0;

    // Input starts at this bit time
    input_idx = clk_idx;

    // Will input 16 bits
    for (i = 0; i < 16; ++i)
        bits[clk_idx++] = 0;

    // Final clock bit
    bits[clk_idx++] = 0;

    /* save the currently selected bank :*/
    

    old_bank = e->bank_0.np_bank;
    e->bank_0.np_bank = 3;

    // Get the current MII register value
    mii_reg = e->bank_3.np_mgmt;

    // Turn off all MII Interface bits
    mii_reg &= ~(MII_MDOE|MII_MCLK|MII_MDI|MII_MDO);

    // Clock all 64 cycles
    for (i = 0; i < sizeof(bits); ++i)
        {
        // Clock Low - output data
        e->bank_3.np_mgmt = mii_reg | bits[i];
        sft_loop_delay(1);

        // Clock Hi - input data
        e->bank_3.np_mgmt = mii_reg | bits[i] | MII_MCLK;
        sft_loop_delay(1);
        bits[i] |= e->bank_3.np_mgmt & MII_MDI;
        }

    // Return to idle state
    // Set clock to low, data to low, and output tristated
    e->bank_3.np_mgmt = mii_reg;
    sft_loop_delay(1);

    // Restore original bank select. Isnt that polite?

    e->bank_0.np_bank = old_bank;

    // Recover input data
    phydata = 0;
    for (i = 0; i < 16; ++i)
        {
        phydata <<= 1;

        if (bits[input_idx++] & MII_MDI)
            phydata |= 0x0001;
        }

    return(phydata);    
}

/*------------------------------------------------------------
 . Writes a register to the MII Management serial interface
 .-------------------------------------------------------------*/
static void r_write_phy_register
        (
        void *hardware_base_address,
        int phy_address,
        r8 phyreg,
        r16 phydata
        )
{
    int old_bank;
    unsigned int i;
    r16 mask;
    r16 mii_reg;
    r8 bits[65];
    int clk_idx = 0;
    np_lan91c111 *e = hardware_base_address;

    // 32 consecutive ones on MDO to establish sync
    for (i = 0; i < 32; ++i)
        bits[clk_idx++] = MII_MDOE | MII_MDO;

    // Start code <01>
    bits[clk_idx++] = MII_MDOE;
    bits[clk_idx++] = MII_MDOE | MII_MDO;

    // Write command <01>
    bits[clk_idx++] = MII_MDOE;
    bits[clk_idx++] = MII_MDOE | MII_MDO;

    // Output the PHY address, msb first
    mask = (r8)0x10;
    for (i = 0; i < 5; ++i)
        {
        if (phy_address & mask)
            bits[clk_idx++] = MII_MDOE | MII_MDO;
        else
            bits[clk_idx++] = MII_MDOE;

        // Shift to next lowest bit
        mask >>= 1;
        }

    // Output the phy register number, msb first
    mask = (r8)0x10;
    for (i = 0; i < 5; ++i)
        {
        if (phyreg & mask)
            bits[clk_idx++] = MII_MDOE | MII_MDO;
        else
            bits[clk_idx++] = MII_MDOE;

        // Shift to next lowest bit
        mask >>= 1;
        }

    // Tristate and turnaround (2 bit times)
    bits[clk_idx++] = 0;
    bits[clk_idx++] = 0;

    // Write out 16 bits of data, msb first
    mask = 0x8000;
    for (i = 0; i < 16; ++i)
        {
        if (phydata & mask)
            bits[clk_idx++] = MII_MDOE | MII_MDO;
        else
            bits[clk_idx++] = MII_MDOE;

        // Shift to next lowest bit
        mask >>= 1;
        }

    // Final clock bit (tristate)
    bits[clk_idx++] = 0;

    /* save the currently selected bank :*/
    old_bank = e->bank_0.np_bank;
    e->bank_0.np_bank = 3;

    // Get the current MII register value
    mii_reg = e->bank_3.np_mgmt;

    // Turn off all MII Interface bits
    mii_reg &= ~(MII_MDOE | MII_MCLK | MII_MDI | MII_MDO);

    // Clock all cycles
    for (i = 0; i < sizeof(bits); ++i)
        {
        // Clock Low - output data
        e->bank_3.np_mgmt = mii_reg | bits[i];
        sft_loop_delay(1);

        // Clock Hi - input data
        e->bank_3.np_mgmt = mii_reg | bits[i] | MII_MCLK;
        sft_loop_delay(1);
        bits[i] |= e->bank_3.np_mgmt & MII_MDI;
        }

    // Return to idle state
    // Set clock to low, data to low, and output tristated
    e->bank_3.np_mgmt = mii_reg;
    sft_loop_delay(1);

    // Restore original bank select
    e->bank_0.np_bank = old_bank;
}

/*------------------------------------------------------------
 . Finds and reports the PHY address
 .-------------------------------------------------------------*/
static int r_lan91c111_detect_phy
        (
        void *hardware_base_address,
        int *phy_address_out
        )
{
    r16 phy_id1 = 0;
    r16 phy_id2 = 0;
    int phy_address;
    int found = 0;
    int result = 0;
    // Scan all 32 PHY addresses if necessary
    for (phy_address = 0; phy_address < 32; ++phy_address)
        {
        // Read the PHY identifiers
        phy_id1 = r_read_phy_register
                (
                hardware_base_address,
                phy_address,
                PHY_ID1_REG 
                );
        phy_id2 = r_read_phy_register
                (
                hardware_base_address,
                phy_address,
                PHY_ID2_REG
                );

        // printf("hba: %X, pa: %X, phy_id1: %X, phy_id2: %X\n", hardware_base_address, phy_address, phy_id1, phy_id2);


        // Make sure it is a valid identifier    
        if ((phy_id2 > 0x0000) && (phy_id2 < 0xffff) &&
            (phy_id1 > 0x0000) && (phy_id1 < 0xffff))
            {
            if ((phy_id1 != 0x8000) && (phy_id2 != 0x8000))
                {
                *phy_address_out = phy_address; // return the phy address
                found = 1;
                break;
                }
            }
        }

    if (!found)
        {
        dprint("r_lan91c111_detect_phy: no phy address");
        result = -1;
        goto go_home;
        }

#if PLUGS_DEBUG
    // Display the PHY type
    if ( (phy_id1 == 0x0016) && ((phy_id2 & 0xFFF0) == 0xF840 ) )
        dprint("r_lan91c111_detect_phy: found lan83C183 (lan91C111 internal)");

    if ( (phy_id1 == 0x0282) && ((phy_id2 & 0xFFF0) == 0x1C50) )
        dprint("r_lan91c111_detect_phy: found lan83C180");
#endif

go_home:
    return result;
}

// --------------------------------------
// Set the LED : CBV - this chip does not
// allow direct control of the LEDs. So,
// for LED A (looking at the Ethernet 
// connector,the one on the right), I will
// set it to Link detect for either 10 OR 
// 100 mbit. LED B will be set to receive 
// packet occurred. I will maintain 
// led_onoff for backword compat.
// If the caller passes back a ZERO (off)
// both leds are programmed for light on
// transmit. Since there is no transmit
// during the led blink test, the LED will
// blink, PROVIDED that a link is ALREADY
// established.
// Register Receive/Phy Control Register

int nr_lan91c111_set_led
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage,
        int led_onoff
        )
{
    np_lan91c111 *e = hardware_base_address;

    LAN91C111_SELECT_BANK(0, hardware_base_address);

    if (led_onoff == 0) {    
        e->bank_0.np_rpcr =     (RPC_ANEG | 
                    (RPC_LED_TX << RPC_LSXB_SHFT) |
                    (RPC_LED_TX << RPC_LSXA_SHFT)  | 
                    RPC_SPEED | 
                    RPC_DPLX);
    }
    else {
        e->bank_0.np_rpcr = RPC_DEFAULT;        
    }
}

// --------------------------------------
// Set loopback mode: pass 0 for off,
// 1 for on.

int nr_lan91c111_set_loopback
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage,
        int loopback_onoff
        )
    {
    dprint("set_loopback: not implemented");
    return -1;
    }

// ---------------------------------
// check for event:
// read the interrupt_reg, and if it's something
// we understand, dispatch it.
// else ignore it.
//

int nr_lan91c111_check_for_events
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage,
        int (*process_frame)(r16 *, int)
        //void *context
        )
{
    np_lan91c111 *e = hardware_base_address;
    int frame_length;
    int frame_length_r16;
    int i;
    int result = 0;
    int watchdog = 50;    // read no more than this many packets
    long timeout = 0;

    __lan91c111_data_word_type__ *lan91c111_data_reg_ptr; // | Pointer within chip for data source
    
    int rx_packet;
    int    status;

    int    saved_pointer;
    int    saved_bank;

    s_lan91c111_state *sls = (s_lan91c111_state *)adapter_storage;


    lan91c111_data_reg_ptr = (__lan91c111_data_word_type__ *)&(e->bank_2.np_data);

    // +------------------------------------
    // | Save the things we'll restore later
    // | (We use bank 2 exclusively in this routine)
        // |
    saved_bank = e->bank_0.np_bank;
    e->bank_2.np_bank = 2;
    saved_pointer = e->bank_2.np_pointer;  
    
check_for_interrupt:

    // +--------------------------------------------
    // | We come back here until all the interrupts
    // | are taken care of... or until the watchdog
    // | expires and we leave anyway.
    // |

    if(watchdog-- <= 0)
        goto go_home;

    // +-------------------------------------
    // | Check for an overrun interrupt
    // | All we'll do is clear it, and report
    // | it (if PLUGS_DEBUG is on). The packets
    // | will be received as usual below.
    // |

    if (e->bank_2.np_interrupt & IM_RX_OVRN_INT)
    {
        // | clear the interrupt
                LAN91C111_ACKNOWLEDGE_INTERRUPT(e, IM_RX_OVRN_INT);
        // | report error
        // | Ok, overruns are so common we dont print anything
        // | but, we could.
        // | dprint("check_for_events %d: overrun");
                printf ("OVRN!");
    }

    if (e->bank_2.np_interrupt & IM_EPH_INT)
    {

        dprint("check_for_events: eph interrupt");
                LAN91C111_ACKNOWLEDGE_INTERRUPT(e, IM_EPH_INT);
        result = -1;
                printf ("EPH!");
    }

    // +-----------------------------------
    // | Receiver interrupts
    // | (bank MUST be 2 for each iteration of loop)
    // | Loop through til all packets read, and
    // | then fall through to handle other IRQs
    // |

    while (e->bank_2.np_interrupt & IM_RCV_INT)
    {
        rx_packet = e->bank_2.np_fifo_ports;

        // | Unexpected condition, FIFO empty? cannot happen...

        if (rx_packet & RXFIFO_REMPTY)
            {
            dprint1("check_for_events %d: fifo empty",__LINE__);
            result = -1;
            goto go_home;
            }

        // | Point the pointer and read the packet
        // | First two words will be status and frame length.
        // | Status might indicate an error, and we toss the
        // | packet

        e->bank_2.np_pointer = PTR_READ | PTR_RCV | PTR_AUTOINC;

        // |
        // | Read first four bypes in either one access or two
        // |

        #if LAN91C111_DATA_BUS_WIDTH == 32
            {
            unsigned int first_word;

            first_word = *lan91c111_data_reg_ptr;
            frame_length = (first_word >> 16);
            status = first_word & 0xFFFF;
            }
        #else
            {
            status = *lan91c111_data_reg_ptr;
            frame_length = *lan91c111_data_reg_ptr;
            }
        #endif

        frame_length &= 0x07ff;  // mask off top bits
        frame_length -= 4;       // already read first 4 bytes

        // | Error? Set the frame length to zero
        // | (everything below keys off frame length
        // | so the packet is effectively tossed out)

        if(status & RS_ERRORS)
            frame_length = 0;
            

        // +------------------------------------
        // | Read frame, with an unrolled loop
        // |
        // | Depending on the address bus scheme (16 or 32)
        // | we do it in one of two interesting ways.
        // |
        // | Num big loops is the unrolled portion of the loop,
        // | and it fetches either longs or shorts.
        // |
        // | We call them "words" here, in either case.
        // |
        
        #define RX_LOOP_UNROLL 8

            {
            __lan91c111_data_word_type__ *w;   // | Working word pointer (16 or 32 bit int)
            unsigned char* wb;                 // | Working BYTE pointer (for leftovers)

            __lan91c111_data_word_type__ last_word;

            int frame_length_in_words;
            int num_big_loops;
            int num_leftover_words;
            int num_leftover_bytes;

            w = (__lan91c111_data_word_type__ *) g_frame_buffer;

            frame_length_in_words = (frame_length) / __lan91c111_data_word_size__;

            num_big_loops = frame_length_in_words / RX_LOOP_UNROLL;
            num_leftover_words = frame_length_in_words - (num_big_loops * RX_LOOP_UNROLL);
            num_leftover_bytes = frame_length - (frame_length_in_words * __lan91c111_data_word_size__);

            for(i = 0; i < num_big_loops; i ++)
                {
                *w++ = *lan91c111_data_reg_ptr;
                *w++ = *lan91c111_data_reg_ptr;
                *w++ = *lan91c111_data_reg_ptr;
                *w++ = *lan91c111_data_reg_ptr;
                *w++ = *lan91c111_data_reg_ptr;
                *w++ = *lan91c111_data_reg_ptr;
                *w++ = *lan91c111_data_reg_ptr;
                *w++ = *lan91c111_data_reg_ptr;
                }

            for(i = 0; i < num_leftover_words; i ++)
                *w++ = *lan91c111_data_reg_ptr;

            // |
            // | Mop-up any remaining bytes.  We have to read the 
            // | last word all at once (Word-wide reads only).
            // |

            if (num_leftover_bytes != 0)
                {
                last_word = *lan91c111_data_reg_ptr;

                wb = (unsigned char *) w;

                for (i = 0; i < num_leftover_bytes; i++)
                    *wb++ = (last_word >> (8*i)) & 0xFF;
                }
            }

            // |
            // | dvb 2003 / jk 2003
            // |
            // | As it turns out, the frame_length at this
            // | point is always even, and the very last
            // | byte is the "control byte".
            // |
            // | It is generated by the chip itself, and
            // | is not part of the transmitted ethernet
            // | packet. It is either 0x40 or 0x60. If
            // | 0x40, the last 2 bytes should be ignored;
            // | if 0x60, the last 1 byte (the control
            // | byte itself) should be ignored.
            // |

                {
                char *frame_buffer_bytes = (char *)g_frame_buffer;
                char control_byte = frame_buffer_bytes[frame_length - 1];

                if(control_byte & 0x20)  // it is 0x60, with "odd" bit
                    frame_length -= 1;
                else
                    frame_length -= 2;
                }

        // |
        // | Thus ends the mildly #ifdef'd extraction of
        // | the packet from the chip
        // +--------------------------------------------------------------

        // | release the received packet

        e->bank_2.np_mmu_command = MC_RELEASE;
                // Wait for MMU to not be busy.
                for (timeout = 1000000; timeout >0; timeout--)
                  {
                    if ((e->bank_2.np_mmu_command & MC_BUSY) == 0)
                      break;
                  }
                
                if (timeout <= 0)
                  {
                    printf ("RX: MMU timeout on packet-release operation\n");
                    return -1;
                  }

        if(frame_length)
            result = (process_frame)(g_frame_buffer,frame_length);//,context);
            //result = (proc)(g_frame_buffer,frame_length,context);

    } // while(anything to receive)

    // | If we get to here, there are no interrupts
    // | set that we care about. Hooray.
    // | fall out.
    
go_home:

    // | This was an interrupt service, so restore
    // | registers as they were
    // | (This matters only for
    // | transmission interruptus)

    e->bank_2.np_pointer = saved_pointer;  
    e->bank_0.np_bank = saved_bank;

    return result;
}


// +--------------------------------
// | int r_allocate_tx_packet(np_lan91c111 *e)
// |
// | The lan91c111 uses an mmu to dole out
// | "packets".   
// |
// | In the past, we tried to play some bizarre game of chess
// | with the chip to try and keep one step ahead of its allocation
// | scheme.  
// |
// | In these modern times, we just set the "AUTO_RELEASE" bit in the
// | control register to zero.  This means: We can keep our packet
// | -forever- if we want to.  We want to.  This function should only
// |
// | Get called at initialization (or re-initialization) time.
// |

static int r_allocate_tx_packet(np_lan91c111 *e, s_lan91c111_state *sls)
{
  unsigned short pnr;

  e->bank_0.np_bank = 2;

  // MC-Alloc is in da house:

  e->bank_2.np_mmu_command = MC_RESET;
  e->bank_2.np_mmu_command = MC_ALLOC;

  // | after an MC_ALLOC, the packet number register (np_pnr) is
  // | *immediately* ready with an answer. (High-bit indicates
  // | the answer we dont like AR_FAILED.)

  pnr = e->bank_2.np_pnr;
  if ((pnr & AR_FAILED))
    {
      dprint ("TX packet allocation failed.  It just failed.\n");
      return -1;
    }

  sls->tx_packet = (pnr >> 8) & 0x3F;

  // The transmit routine needs to know whether or not we've
  // ever sent a packet before.  It will try to wait until any 
  // outgoing packets have been successfully sent.  If we've
  // never sent any, it'll be a long wait.
  //
  sls->ever_sent_packet = 0;

  return 0;
}


// The low-level transmit routine
//
// We follow the lan91c111 transmission ettiquette here.
// return 0 for AOK, or -1 if we couldn't send for some reason
//
#define TX_LOOP_UNROLL  8

int nr_lan91c111_tx_frame
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage,
        const unsigned char *ethernet_frame,
        int frame_length
        )
    {
    np_lan91c111 *e = hardware_base_address;
    s_lan91c111_state *sls = (s_lan91c111_state *)adapter_storage;

    int frame_length_in_words;
    int i;
    int num_big_loops;
    int num_leftover_words;
    int num_leftover_bytes;
    __lan91c111_data_word_type__ *lan91c111_data_reg_ptr; // | Pointer within chip for data source
    volatile unsigned short *lan91c111_data_reg_short_ptr;         // | Sometimes forced to be 16-bit writes

    __lan91c111_data_word_type__ *w; // | walker within frame to transmit

    r16 status;
    int result = 0;
    int old_irq = 0;

    // |
    // | cast to short or long register, for best speed in
    // | shoveling bytes into the chip
    // |

    lan91c111_data_reg_ptr = (__lan91c111_data_word_type__ *)&(e->bank_2.np_data);
    lan91c111_data_reg_short_ptr = (unsigned short *)&(e->bank_2.np_data);

    frame_length_in_words = (frame_length) / __lan91c111_data_word_size__;

    num_big_loops = frame_length_in_words / RX_LOOP_UNROLL;
    num_leftover_words = frame_length_in_words - (num_big_loops * RX_LOOP_UNROLL);
    num_leftover_bytes = frame_length - (frame_length_in_words * __lan91c111_data_word_size__);
        

    ////////////////
    // If we got to here, we really want to start fooling with
    // the hardware.  Better turn off interrupts, or else
    // someone might sneak in underneath us.
    //
    old_irq = nr_lan91c111_set_irq (e, sls,0);

    if (!frame_length)
        {
        result = 0;   // We succeeded! ...in doing nothing.
        goto go_home;
        }

    ////////////////
    //
    // Think about the previous packet we sent...If we ever did.
    //
    if (sls->ever_sent_packet)
        {
        // Be sure previous packet is gone.
        // If not, return a "nice" failure code, that means:
        //
        //    "I didn't send your packet, but I might if you ask
        //    again."
        //
        // TX_INT gets set upon transmit-completion (either
        // successful or not).
        //

        e->bank_0.np_bank = 2;  
        if (!(e->bank_2.np_interrupt & IM_TX_INT))
            {
            result = 1;
            goto go_home;
            }

        // If the packet got sent, be sure it had a nice trip.
        // Check the TX_ENA bit in the transimit-control register.
        // This gets set to zero if something bad happened.
        //
        e->bank_2.np_bank = 0;  
        if (!(e->bank_0.np_tcr & TCR_ENABLE)) 
            {
            // Hm.  Last packet didn't make it.  
            //      No use crying over it.  Well, maybe a little cry:
            dprint ("TX: previous packet failed.");

            // Re-enable transmit
            e->bank_0.np_tcr |= TCR_ENABLE;
            }
        }
           
    e->bank_0.np_bank = 2;  
        
    /* We have a reserved packet address, so tell the card to use it */
    e->bank_2.np_pnr = sls->tx_packet; 

    /* point to the beginning of the packet */
    e->bank_2.np_pointer = PTR_AUTOINC;

    // |
    // | first 4 bytes put into the chip
    // | are "status" and "packet length"
    // | the length gets 6 added to it, for
    // | the status length and control byte
    // |

    *lan91c111_data_reg_short_ptr = 0x0000; // status
    *lan91c111_data_reg_short_ptr = frame_length + 6;

    w = (__lan91c111_data_word_type__ *)ethernet_frame;
        
    for (i=0; i < num_big_loops; i++)
        {
        // The number of writes in this loop must
        // exactly equal TX_LOOP_UNROLL, or else it
        // will be a short trip.

        *lan91c111_data_reg_ptr = *w++;
        *lan91c111_data_reg_ptr = *w++;
        *lan91c111_data_reg_ptr = *w++;
        *lan91c111_data_reg_ptr = *w++;

        *lan91c111_data_reg_ptr = *w++;
        *lan91c111_data_reg_ptr = *w++;
        *lan91c111_data_reg_ptr = *w++;
        *lan91c111_data_reg_ptr = *w++;
        }

    for(i = 0; i < num_leftover_words; i++)
        *lan91c111_data_reg_ptr = *w++;

    // |
    // | Handle the last noodly little bytes as needed
    // |

        {
        unsigned short *w16 = (unsigned short *)w;

        // | Send the last 16-bit word, if there is one

        if(num_leftover_bytes >= 2)
            *lan91c111_data_reg_short_ptr = *w16++;

        // | Send the last byte, if there is one, as part of
        // | the mandatory final 16-bit control word

        if(num_leftover_bytes & 1)
            *lan91c111_data_reg_short_ptr = 0x2000 | (*w16 & 0x00ff);
        else
            *lan91c111_data_reg_short_ptr = 0;
        }


    /* The enqueue command sends the packet out */

    ////////////////
    // 
    // Clear the TX_INT bit before sending.  When 
    // transmit is complete, it will be set.
    // 
    LAN91C111_ACKNOWLEDGE_INTERRUPT(e, IM_TX_INT);

    e->bank_2.np_mmu_command = MC_ENQUEUE;

    // The packet is queued, so we just leave it to the 
    // fates.  Later on, when we try to transmit another packet,
    // we'll find out what happened.  I can hardly wait.  Bye.

go_home:
    nr_lan91c111_set_irq (e, sls, old_irq);
    return result;
    }


// ----------------------------------------
// Turn on this chips promiscuous mode. Or off.

int nr_lan91c111_set_promiscuous
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage,
        int promiscuous_onoff
        )
{
    np_lan91c111 *e = hardware_base_address;

    LAN91C111_SELECT_BANK(0 , hardware_base_address);
    if (!promiscuous_onoff) {
        e->bank_0.np_rcr = (RCR_DEFAULT & ~RCR_PRMS);
    }
    else {
        e->bank_0.np_rcr = (RCR_DEFAULT | RCR_PRMS);
    }
}

// -----------------------------------------
// Enable or disable interrupts for this adapter
//
// return the previous state ("on" or "off")
// of the interrupts.
//

int nr_lan91c111_set_irq
        (
        void *hardware_base_address,
        ns_plugs_adapter_storage *adapter_storage,
        int irq_onoff
        )
    {
    np_lan91c111 *e = hardware_base_address;
    s_lan91c111_state *sls = (s_lan91c111_state *)adapter_storage;

    unsigned char interrupt_mask;
    unsigned char* mask_register_ptr;

    int old_irq_onoff;

    if(irq_onoff)
        interrupt_mask = (LAN91C111_INTERRUPT_MASK);
    else
        interrupt_mask = 0;

    e->bank_0.np_bank = 2;

    // We really only want to write the high byte.

    mask_register_ptr = (unsigned char*) &(e->bank_2.np_interrupt);
    mask_register_ptr++;  // High byte: add 1 to byte-address.
    *mask_register_ptr = interrupt_mask;


    old_irq_onoff = sls->irq_onoff;
    sls->irq_onoff = irq_onoff;

    return old_irq_onoff;
    }

//not part of port - tomx
/*
ns_plugs_adapter_description ng_lan91c111 =
{
    &nr_lan91c111_reset,
    &nr_lan91c111_set_led,
    &nr_lan91c111_set_loopback,
    &nr_lan91c111_check_for_events,
    &nr_lan91c111_tx_frame,
    &nr_lan91c111_dump_registers,
    &nr_lan91c111_set_promiscuous,
    &nr_lan91c111_set_irq,
    "lan91c111"
};
*/

// end of file
