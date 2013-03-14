/****************************************************************************

    Copyright (c) 2013 Kalycito Infotech Private Limited

    Project: openPOWERLINK

    Description: Ethernet driver for Intel's I210 Ethernet Controller

    License:

        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions
        are met:

        1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

        2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

        3. Neither the name of Kalycito Infotech nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without prior written permission. For written
        permission, please contact info@kalycito.com.

        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
        FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
        COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
        INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
        BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
        LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
        LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
        ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
        POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

    If a provision of this License is or becomes illegal, invalid or
    unenforceable in any jurisdiction, that shall not affect:

        1. The validity or enforceability in that jurisdiction of any other
        provision of this License; or
        2. The validity or enforceability in other jurisdictions of that or
        any other provision of this License.

    Based on
        1. Edrv82573.c and igb_main.c
****************************************************************************/
#include "global.h"
#include "EplInc.h"
#include "edrv.h"

#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>

#if EDRV_USE_TTTX == FALSE
#error EdrvI210 only works with time triggered sending (EDRV_USE_TTTX)!
#endif

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//              Intel's I210 Ethernet controller specific defines
//---------------------------------------------------------------------------
#define CONFIG_BAR  0
#define DRIVER_NAME "epl"

#define EDRV_MAX_RX_QUEUES       1
#define EDRV_MAX_TX_QUEUES       1
#define EDRV_MAX_QUEUE_VECTOR    1

#define INTERRUPT_STRING_SIZE    25
#define	SEC_TO_NSEC              1000000000

//------------------------------------------------------------------------------
//                      Device Register offsets
//------------------------------------------------------------------------------
#define EDRV_CTRL_REG            0x00000        // Device Control Register
#define EDRV_CTRL_EXTN_REG       0x00018        // Extended Device Control Register
#define EDRV_STATUS_REG          0x00008        // Device Status Register
#define EDRV_MDIC_REG            0x00020        // MDI Control Register
#define EDRV_INTR_READ_REG       0x01500        // Interrupt Cause Read
#define EDRV_INTR_SET_REG        0x01504        // Interrupt Cause Set
#define EDRV_INTR_MASK_SET_READ  0x01508        // Interrupt Mask Set/Read
#define EDRV_INTR_MASK_CLEAR     0x0150C        // Interrupt Mask Clear
#define EDRV_INTR_ACK_AUTO_MASK  0x01510        // Interrupt Acknowledge Auto Mask
#define EDRV_INTR_EIAC           0x0152C        // Extended Interrupt Auto Clear
#define EDRV_INTR_GPIE_REG       0x01514        // General Purpose Interrupt Enable
#define EDRV_IVAR0_REG           0x01700        // Interrupt Vector Allocation Registers (Tx-Rx queues)
#define EDRV_IVAR_MISC           0x01740        // IVAR for other causes
#define EDRV_EXT_INTR_REG        0X01580        // Extended Interrupt Cause Set
#define EDRV_EXT_INTR_MASK_CLEAR 0x01528        // Extended Interrupt Mask Clear
#define EDRV_EXT_INTR_MASK_SET   0x01524        // Extended Interrupt Mask Set/Read
#define EDRV_MDICNFG_REG         0x00E04        // MDC/MDIO Configuration Register
#define EDRV_SWSM_REG            0x05B50        // Software Semaphore
#define EDRV_TIPG_REG            0x00410        // Transmit Inter Packet Gap
#define EDRV_TXPBSIZE_REG        0x03404        // Transmit Packet Buffer Size
#define EDRV_DCA_CTRL_REG        0x05B74        // Direct Cache Access control register
#define EDRV_EECD_REG            0x00010        // EEPROM-Mode Control Register
#define EDRV_SW_FW_SYNC          0x05B5C        // Software-Firmware Synchronization
#define	EDRV_SYSTIMR_REG         0x0B6F8        // System time register Residue
#define EDRV_SYSTIML_REG         0x0B600        // System time register Low
#define EDRV_SYSTIMH_REG         0x0B604        // System time register High
#define EDRV_STAT_TPT            0x040D4        // Total Packets Transmitted
#define EDRV_MRQC_REG            0x05818        // Multiple Receive Queues Command
#define EDRV_EEER_REG            0x00E30        // Energy Efficient Ethernet (EEE) Register
// Semaphore defines used for Software/Firmware synchronization
#define EDRV_SWSM_SMBI           0x00000001     // Software
#define EDRV_SWSM_SWESMBI        0x00000002     // Software EEPROM Semaphore Bit
#define EDRV_EECD_AUTO_RD        0x00000200
#define EDRV_SWFW_PHY0_SM	     0x02           // Software Phy Semaphore Bit
//------------------------------------------------------------------------------
//                  Transmit Register offset and defines
//------------------------------------------------------------------------------

#define EDRV_TCTL_REG            0x00400        // Tx Control
#define EDRV_TCTL_EXT_REG        0x00404        // Tx Control Extended
#define EDRV_DTXCTL_REG          0x03590        // DMA Tx Control
#define EDRV_DTX_MAX_PKTSZ_REG   0x0355C        // DMA Tx Max Allowable Packet Size
#define EDRV_TXPBSIZE_REG        0x03404        // Transmit Packet Buffer Size
#define EDRV_TQAVCTRL_REG        0x03570        // Transmit Qav Control
#define EDRV_TDBAL(n)            ((n < 4) ? (0x0E000 + 0x40 * n) :\
                                 (0x0E000 + 0x40 * (n - 4)))             // Tx Descriptor Base Low ( n: 0-3 )
#define EDRV_TDBAH(n)            ((n < 4) ? (0x0E004 + 0x40 * n) :\
                                 (0x0E004 + 0x40 * (n - 4)))            // Tx Descriptor Base High ( n: 0-3 )
#define EDRV_TDLEN(n)            ((n < 4) ? (0x0E008 + 0x40 * n) :\
                                 (0x0E008 + 0x40 * (n - 4)))            // Tx Descriptor Ring Length ( n: 0-3 )
#define EDRV_TDHEAD(n)           ((n < 4) ? (0x0E010 + 0x40 * n) :\
                                 (0x0E010 + 0x40 * (n - 4)))            // Tx Descriptor Head ( n: 0-3 )
#define EDRV_TDTAIL(n)           ((n < 4) ? (0x0E018 + 0x40 * n) :\
                                 (0x0E018 + 0x40 * (n - 4)))            // Tx Descriptor Tail ( n: 0-3 )
#define EDRV_TXDCTL(n)           ((n < 4) ? (0x0E028 + 0x40 * n) :\
                                 (0x0E028 + 0x40 * (n - 4)))            // Tx Descriptor Control Queue ( n: 0-3 )
#define EDRV_TQAVCC(n)           ((n < 2) ? (0x03004 + 0x40 * n) :\
                                 (0x03004 + 0x40 * (n - 2)))            // Tx Qav Credit Control ( n: 0-1 )
#define EDRV_TQAVHC(n)           ((n < 2) ? (0x0300C + 0x40 * n) :\
                                 (0x0300C + 0x40 * (n - 2)))            // Tx Qav High Credits ( n: 0-1 )
#define EDRV_TDWBAL(n)           ((n < 4) ? (0x0E038 + 0x40 * n) :\
                                 (0x0E038 + 0x40 * (n - 1)))            // Tx Descriptor WB Address Low Queue ( n: 0-3 )
#define EDRV_TIPG_DEF            0x00702008      // default according to Intel PCIe GbE Controllers Open Source Software Developer's Manual
#define EDRV_TXPBSIZE_DEF        0x04104208      // 8 Kb TQ0, 8Kb TQ1, 4 Kb TQ2, 4Kb TQ3 and 4 Kb Os2Bmc
#define EDRV_MAX_TX_DESCRIPTOR   256             // Max no of Desc in mem
#define EDRV_MAX_TX_DESC_LEN     (EDRV_MAX_TX_DESCRIPTOR - 1)
// One slot to diff full
#define EDRV_MAX_TTX_DESC_LEN    ((EDRV_MAX_TX_DESCRIPTOR >> 1) -1)
#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS      128        //Max no of Buffers
#endif

#define EDRV_MAX_FRAME_SIZE      0x600      // 1536
#define EDRV_TX_BUFFER_SIZE      ( EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE)
#define EDRV_TX_DESCS_SIZE       (EDRV_MAX_TX_DESCRIPTOR * sizeof(tEdrvAdvTxDesc))

#define EDRV_TDESC_CMD_DEXT      (1 << 29)      // Descriptor type
#define EDRV_TDESC_CMD_RS        (1 << 27)      // Report Status
#define EDRV_TDESC_CMD_IC        (1 << 26)      // Insert Checksum
#define EDRV_TDESC_CMD_IFCS      (1 << 25)      // Insert FCS
#define EDRV_TDESC_CMD_EOP       (1 << 24)      // End of Packet
#define EDRV_TDESC_STATUS_DD     (1 << 0 )      // Descriptor done
#define EDRV_TDESC_DTYP_CTXT     (2 << 20)      // Context Descriptor
#define EDRV_TDESC_DTYP_ADV      (3 << 20)      // Advance Tx Descriptor
#define EDRV_TCTL_CT             0x000000F0     // Collision Threshold
#define EDRV_TCTL_CLEAR_CT       0x00000FF0     // Clear Collision Threshold bits
#define EDRV_TCTL_EN             0x00000002     // Transmit Enable
#define EDRV_TCTL_PSP            0x00000008     // Pad short packets
#define EDRV_TCTL_RTLC           0x01000000     // Re-Transmit on Late collision
#define EDRV_TCTL_EXT_COLD_CLEAR 0x000FFC00     // Clear Collison threshold
#define EDRV_TCTL_EXT_COLD       0x0003F000     // default value as per 802.3 spec
#define EDRV_TXDCTL_PTHRESH      0              // Prefetch threshold
#define EDRV_TXDCTL_HTHRESH      0              // Host threshold
#define EDRV_TXDCTL_WTHRESH      0              // Write-Back threshold
#define EDRV_TXDCTL_QUEUE_EN     0x02000000     // Transmit Queue Enable
#define EDRV_TXDCTL_SWFLSH      (1 << 26)       // Software Flush of queue
#define EDRV_TXDCTL_PRIORITY    (1 << 27)       // Transmit Queue Priority
#define EDRV_TQAVCTRL_TXMODE     (1 << 0 )      // Transmit mode configuration
#define EDRV_TQAVCTRL_FETCH_ARB  (1 << 4 )      // Data Fetch arbitration
#define EDRV_TQAVCTRL_DTRANSARB  (1 << 8 )      // Data Transmit arbitration
#define EDRV_TQAVCTRL_TRANSTIM   (1 << 9 )      // Data Launch time valid
#define EDRV_TQAVCTRL_SP_WAIT_SR (1 << 10)      // SP queues wait for SR queue to quarantee SR launch time
#define EDRV_TQAVCTRL_1588_STAT_EN      (1 << 2 )   // Report DMA transmit time in WB descriptors
#define EDRV_TQAVCC_QUEUE_MODE_SR       (1 << 31)   // Queue mode Strict Reservation (Launch time based)
#define EDRV_TQAVCTRL_FETCH_TM_SHIFT    16          // Fetch time shift
#define EDRV_LAUNCH_OSO_SHIFT           5                  // Launch time offset shift (Launch time = Launch time + Off)
#define EDRV_TQAVARBCTRL_TXQPRIO(_q, _n)  (((_n) & 0x3) << (_q << 2))
#define EDRV_TXPBSIZE_TX1PB_SHIFT       6       // Tx packet buffer size shift for 2 queue
#define EDRV_TXPBSIZE_TX2PB_SHIFT    	12      // Tx packet buffer size shift for 3 queue
#define EDRV_TXPBSIZE_TX3PB_SHIFT    	18      // Tx packet buffer size shift for 4 queue
//------------------------------------------------------------------------------
//                      Receive Register offset and defines
//------------------------------------------------------------------------------
#define EDRV_RCTL_REG            0x00100        // Rx Control
#define EDRV_RXPBSIZE_REG        0x02404        // Rx Packet Buffer Size
#define EDRV_SRRCTL(n)           ((n < 4) ? (0x0C00C + 0x40 * n) :\
                                 (0x0C00C + 0x40 * (n - 4)))            // Split and Replication Receive Control Register Queue
#define EDRV_RDBAL(n)            ((n < 4) ? (0x0C000 + 0x40 * n) :\
                                 (0x0C000 + 0x40 * (n -4)))             // Rx Descriptor Base Low Queue
#define EDRV_RDBAH(n)            ((n < 4) ? (0x0C004 + 0x40 * n) :\
                                 (0x0C004 + 0x40 * (n - 4)))            // Rx Descriptor Base High Queue
#define EDRV_RDLEN(n)            ((n < 4) ? (0x0C008 + 0x40 * n) :\
                                 (0x0C008 + 0x40 * (n - 4)))            // Rx Descriptor Ring Length Queue
#define EDRV_RDHEAD(n)           ((n < 4) ? (0x0C010 + 0x40 * n) :\
                                 (0x0C010 + 0x40 * (n - 4)))            // Rx Descriptor Head Queue
#define EDRV_RDTAIL(n)           ((n < 4) ? (0x0C018 + 0x40 * n) :\
                                 (0x0C018 + 0x40 * (n - 4)))            // Rx Descriptor Tail Queue
#define EDRV_RXDCTL(n)           ((n < 4) ? (0x0C028 + 0x40 * n) :\
                                 (0x0C028 + 0x40 * (n - 4)))            // Receive Descriptor Control Queue
#define EDRV_RAL(n)              (0x05400 + 8 * n)    // Receive Address Low (15:0)
#define EDRV_RAH(n)              (0x05404 + 8 * n)    // Receive Address High (15:0)
#define EDRV_MTA(n)              (0x05200 + 4 * n)    // Multicast Table Array
#define EDRV_MAX_RX_DESCRIPTOR   256                  // Max no of Desc in mem
#define EDRV_MAX_RX_DESC_LEN     (EDRV_MAX_RX_DESCRIPTOR - 1)
// Used for buffer handling
#define EDRV_MAX_RX_BUFFERS      256            // Max no of Buffers
#define EDRV_RX_BUFFER_SIZE      (EDRV_MAX_RX_BUFFERS * EDRV_MAX_FRAME_SIZE)
// Rx buffer size
#define EDRV_RX_DESCS_SIZE       (EDRV_MAX_RX_DESCRIPTOR * sizeof(tEdrvAdvRxDesc))

#define EDRV_RDESC_STATUS_EOP    (1 << 1)       // End of Packet
#define EDRV_RDESC_STATUS_DD     (1 << 0)       // Descriptor done
#define EDRV_RDESC_ERRORS_RXE    (1 << 31)      // RX data Error
#define EDRV_RCTL_EN             (1 << 1 )      // Receive Enable
#define EDRV_RCTL_SBP            (1 << 2 )      // Store Bad Packets
#define EDRV_RCTL_UPE            (1 << 3 )      // Unicast Enable
#define EDRV_RCTL_MPE            (1 << 4 )      // Multicast Enable
#define EDRV_RCTL_LPE            (1 << 5 )      // Long packet Reception enable
#define EDRV_RCTL_LBM_MAC        (1 << 6 )      // Loop back mode
#define EDRV_RCTL_LBM_CLEAR      (3 << 6 )      // Clear loop back
#define EDRV_RCTL_MO_SHIFT       12
#define EDRV_RCTL_MO_36_47       (0 << 12)      // Multicast Offset bit[47:36]
#define EDRV_RCTL_MO_35_46       (1 << 12)      // Multicast Offset bit[46:35]
#define EDRV_RCTL_MO_34_45       (1 << 13)      // Multicast Offset bit[45:34]
#define EDRV_RCTL_MO_32_43       (3 << 12)      // Multicast Offset bit[43:32]
#define EDRV_RCTL_BAM            (1 << 15)      // Accept Broadcast packets
#define EDRV_RCTL_BSIZE_OFFSET   16             // Buffer Size Offset (00:2048, 01:1024, 10:512, 11:256)
#define EDRV_RCTL_VFE            (1 << 16)      // Enable Vlan Filter
#define EDRV_RCTL_PSP            (1 << 21)      // Pad Small packets
#define EDRV_RCTL_SECRC          (1 << 26)      // Strip CRC
#define EDRV_SRRCTL_DESCTYPE_ADV (1 << 25)      // Advance Rx Descriptor one buffer
#define EDRV_SRRCTL_TIMESTAMP    (1 << 30)      // Time stamp received packets
#define EDRV_SRRCTL_DROP_EN      (1 << 31)      // drop packets if no descriptors available
#define EDRV_RXDCTL_PTHRESH      0              // Prefetch Threshold
#define EDRV_RXDCTL_HTHRESH      0              // Host Threshold
#define EDRV_RXDCTL_WTHRESH      1              // Write-back threshold
#define EDRV_RXDCTL_QUEUE_EN     (1 << 25)      // Receive Queue Enable
#define EDRV_RXDCTL_SWFLUSH      (1 << 26)      // Software flush
#define EDRV_RXPBSIZE_CLEAR      0x3F           // Clear Packet buffer size
#define EDRV_RXPBSIZE_DEF        0x8000001E     // default configuration
#define	EDRV_RAH_AV              (1 << 31)      // Address Valid
//------------------------------------------------------------------------------
//                      Time Sync Register offset and defines
//------------------------------------------------------------------------------
#define EDRV_LAUNCH_OSO          0x03578        // Launch Time Offset Register 0
#define EDRV_FREQOUT0            0x0B654        // Frequency Out 0 Control Register
#define EDRV_TSIM                0x0B674        // Time Sync Interrupt Mask Register
#define EDRV_TSICR               0x0B66C        // Time Sync Interrupt Cause Register
#define EDRV_TSAUXC              0x0B640        // Auxiliary Control Register
#define EDRV_AUXSTMPL0           0x0B65C        // Auxiliary Time Stamp 0 Reg - Low
#define EDRV_AUXSTMPH0           0x0B660        // Auxiliary Time Stamp 0 Reg - High
#define EDRV_TRGTTIML0           0x0B644        // Target Time Register 0 Low
#define EDRV_TRGTTIMH0           0x0B648        // Target Time Register 0 High
#define EDRV_TSSDP               0x0003C        // Time Sync SDP Configuration Register
#define	EDRV_TSIM_TT0            (1 << 3)       // Target time 0 Trigger Mask.
#define	EDRV_TSIM_TT1            (1 << 4)       // Target time 1 Trigger Mask.
#define EDRV_TSAUXC_SAMP_AUTO    0x00000008     // Sample SYSTIM into AUXSTMP0 register
#define EDRV_TSAUXC_EN_TT0       (1 << 0)       // Enable target time 0
#define EDRV_TSAUXC_EN_TT1       (1 << 1)       // Enable target time 1
#define EDRV_TSAUXC_EN_CLK0      (1 << 2)       // Enable Configurable Frequency Clock 0
#define EDRV_TSAUXC_ST0          (1 << 4)       // Start Clock 0 Toggle on Target Time 0
#define EDRV_TSSDP_TS_SDP3_SEL_CLK0 (2 << 15)   // Freq clock 0 is output on SDP3
#define EDRV_TSSDP_TS_SDP3_EN    (1 << 17)      // SDP3 is assigned to Tsync
#define EDRV_TSSDP_TS_SDP0_SEL_CLK0 (2 << 6 )   // Freq clock 0 is output on SDP0
#define EDRV_TSSDP_TS_SDP0_EN    (1 << 8 )      // SDP0 is assigned to Tsync
#define SDP0_SET_DIR_OUT         0x00400000     // Set direction Out for SDP0
#define SDP0_SET_HIGH            0x00040000     // Set SDP0 High
#define SDP1_SET_DIR_OUT         0x00800000     // Set direction Out for SDP1
#define SDP1_SET_HIGH            0x00080000     // Set SDP1 High
#define SDP2_SET_DIR_OUT         0x00000400     // Set direction Out for SDP2
#define SDP2_SET_HIGH            0x00000040     // Set SDP2 High
#define SDP3_SET_DIR_OUT         0x00000800     // Set direction Out for SDP3
#define SDP3_SET_HIGH            0x00000080     // Set SDP3 High
//------------------------------------------------------------------------------
//                        MDIC specific defines
//------------------------------------------------------------------------------

#define EDRV_MDIC_DATA_MASK      0x0000FFFF     // Data mask
#define EDRV_MDIC_OP_READ        0x08000000     // Read command for PHY register
#define EDRV_MDIC_OP_WRITE       0x04000000     // Write command for PHY register
#define EDRV_MDIC_RD             0x10000000     // Ready Bit to indicate completion of Read or Write
#define EDRV_MDIC_INTR_DIS       0x20000000     // Interrupt Enable at the end of operation specified by opcode
#define EDRV_MDIC_ERR            0x40000000     // Error in read
#define EDRV_MDIC_REGADD_MASK    0x001F0000     // PHY register address mask
//------------------------------------------------------------------------------
//                          Interrupt defines
//------------------------------------------------------------------------------

#define EDRV_EIMC_CLEAR_ALL      0xC000001F     // Clear all interrupts
#define EDRV_INTR_GPIE_NSICR     (1 << 0 )      // Non Selective Interrupt Clear on Read
#define EDRV_INTR_GPIE_MULT_MSIX (1 << 4 )      // Multiple MSIX
#define EDRV_INTR_GPIE_PBA       (1 << 31)      // PBA Support
#define EDRV_INTR_ICR_TXDW       0x00000001     // Transmit descriptor write back
#define EDRV_INTR_ICR_RXDW       (1 << 7 )      // Receive descriptor write back
#define EDRV_INTR_ICR_TIME_SYNC  (1 << 19)      // Time Sync interrupt
#define	EDRV_INTR_ICR_RXDMT0     (1 << 4 )      // Receive Descriptor Minimum Threshold Reached
#define EDRV_INTR_ICR_RXMISS     (1 << 6)       // Missed packet interrupt
#define EDRV_INTR_ICR_FER        (1 << 22)      // Fatal Error
#define EDRV_EIMC_OTHR_EN        (1 << 31)      // Other Interrupt Cause Active
#define EDRV_EICS_OTHER          (1 << 0 )      // Vector for Other Interrupt Cause in MSI-X mode
#define EDRV_EICS_QUEUE          0x0000001E     // All queue interrupts
#define EDRV_EICS_TXRXQUEUE1     (1 << 1 )      // Vector for TX-RX queue 0
#define EDRV_EICS_TXRXQUEUE2     (1 << 2 )      // Vector for TX-RX queue 1
#define EDRV_EICS_TXRXQUEUE3     (1 << 3 )      // Vector for TX-RX queue 2
#define EDRV_EICS_TXRXQUEUE4     (1 << 4 )      // Vector for TX-RX queue 3
#define EDRV_IVAR_VALID          0x80           // Interrupt vector valid bit
#define EDRV_INTR_ICR_MASK_DEF     (EDRV_INTR_ICR_TXDW          /* Transmit descriptor write back */\
                                   | EDRV_INTR_ICR_RXDW         /* Receive descriptor write back */\
                                   | EDRV_INTR_ICR_RXDMT0       /* Receive Descriptor Minimum Threshold Reached */\
                                   | EDRV_INTR_ICR_RXMISS       /* Missed packet interrupt */\
                                   | EDRV_INTR_ICR_FER          /* Fatal Error */\
                                   | EDRV_INTR_ICR_TIME_SYNC)   /* Time Sync interrupt */

//------------------------------------------------------------------------------
//                      Phy register offsets and defines
//------------------------------------------------------------------------------
#define PHY_CONTROL_REG_OFFSET   0x00           // Phy Control register
#define PHY_STATUS_REG_OFFSET    0x01           // Phy Status Register
#define PHY_LINK_SPEED_100       0x2000         // 100 MB/s Link speed
#define PHY_RESET                0x8000         // Phy reset
#define PHY_MODE_FD              0x0100         // Full Duplex
#define PHY_CONTROL_POWER_DOWN   0x0800         // Power down Phy
#define PHY_I210_COPPER_SPEC     0x0010         // I210 specific phy register
#define PHY_I210_CS_POWER_DOWN   0x0002         // Power down phy
//------------------------------------------------------------------------------
//					Control/Control Extension register defines
//------------------------------------------------------------------------------

#define EDRV_CTRL_FD             0x00000001     // Full Duplex
#define EDRV_CTRL_MASTER_DIS     (1 << 2 )      // GIO Master Disable
#define EDRV_CTRL_SLU            (1 << 6 )      // Set Link Up
#define EDRV_CTRL_ILOS           (1 << 7 )      // Invert Loss-of-Signal (LOS/LINK) Signal
#define EDRV_CTRL_RST            (1 << 26)      // Port Software Reset
#define EDRV_CTRL_RFCE           (1 << 27)      // Receive Flow Control Enable
#define EDRV_CTRL_TFCE           (1 << 28)      // Transmit Flow Control Enable
#define EDRV_CTRL_DEV_RST        (1 << 29)      // Device Reset
#define EDRV_CTRL_PHY_RST        (1 << 31)      // PHY Reset
#define EDRV_CTRL_EXTN_DRV_LOAD  (1 << 28)      // Software driver loaded. Used to take control from Firmware
//------------------------------------------------------------------------------
//                       Status register defines
//------------------------------------------------------------------------------
#define EDRV_STATUS_MASTER_EN    (1 << 19)      // GIO Master Enable
#define EDRV_STATUS_PF_RST_DONE  (1 << 21)      // Software reset done
#define EDRV_STATUS_LU           (1 << 1)       // Link Up
#define EDRV_STATUS_DEV_RST_SET  (1 << 20)      // Device reset done
#define EDRV_SW_RST_DONE_TIMEOUT 10   // ms     // Software rest timeout
#define EDRV_MASTER_DIS_TIMEOUT  90   // ms     // Master disable timeout
#define EDRV_LINK_UP_TIMEOUT     3000 // ms     // Link up timeout
#define EDRV_AUTO_READ_DONE_TIMEOUT       10    // Auto register read done timeout
#define EDRV_PHY_SEMPHORE_TIMEOUT         100
#define EDRV_POLL_TIMEOUT        3              // Tx-Rx Enable bit poll timeout
//------------------------------------------------------------------------------
//                      Statistic registers
//------------------------------------------------------------------------------

#define EDRV_STAT_TPR            0x040D0        // Total Packets Received
#define EDRV_STAT_GPRC           0x04074        // Good Packets Received Count
#define EDRV_STAT_BPRC           0x04078        // Broadcast Packets Received Count
#define EDRV_STAT_MPRC           0x0407C        // Multicast Packets Received Count
#define EDRV_PQGPRC(n)           ((n < 4) ? (0x10010 + 0x100 * n) :\
                                 (0x10010 + 0x100  * (n - 4)))          // Per Queue Good Packets Received Count
// Utility Macros
#define EDRV_GET_RX_DESC(pQueue , iIndex) (&(((tEdrvAdvRxDesc *)pQueue->m_pDescVirt)[iIndex]))
#define EDRV_GET_TX_DESC(pQueue , iIndex) (&(((tEdrvAdvTxDesc *)pQueue->m_pDescVirt)[iIndex]))
#define EDRV_GET_CTXT_DESC(pQueue , iIndex) (&(((tEdrvContextDesc *)pQueue->m_pDescVirt)[iIndex]))
#define EDRV_GET_TTX_DESC(pQueue, iIndex ) (&(((tEdrvTtxDesc *)pQueue->m_pDescVirt)[iIndex]))

#define EDRV_REGDW_READ(dwReg)          readl((unsigned char  *)EdrvInstance_l.m_pIoAddr + dwReg)
#define EDRV_REGDW_WRITE(dwReg, dwVal)  writel(dwVal, (unsigned char *)EdrvInstance_l.m_pIoAddr + dwReg)
#define EDRV_REGB_READ(dwReg)           readb(EdrvInstance_l.m_pIoAddr + dwReg)

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
void PUBLIC TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
void PUBLIC TgtDbgPostTraceValue (DWORD dwTraceValue_p);
#define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
#define TGT_DBG_POST_TRACE_VALUE(v)     TgtDbgPostTraceValue(v)
#else
#define TGT_DBG_SIGNAL_TRACE_POINT(p)
#define TGT_DBG_POST_TRACE_VALUE(v)
#endif

#define EDRV_COUNT_SEND                 TGT_DBG_SIGNAL_TRACE_POINT(2)
#define EDRV_COUNT_TIMEOUT              TGT_DBG_SIGNAL_TRACE_POINT(3)
#define EDRV_COUNT_PCI_ERR              TGT_DBG_SIGNAL_TRACE_POINT(4)
#define EDRV_COUNT_TX                   TGT_DBG_SIGNAL_TRACE_POINT(5)
#define EDRV_COUNT_RX                   TGT_DBG_SIGNAL_TRACE_POINT(6)
#define EDRV_COUNT_LATECOLLISION        TGT_DBG_SIGNAL_TRACE_POINT(10)
#define EDRV_COUNT_TX_COL_RL            TGT_DBG_SIGNAL_TRACE_POINT(11)
#define EDRV_COUNT_TX_FUN               TGT_DBG_SIGNAL_TRACE_POINT(12)
#define EDRV_COUNT_TX_TEST              TGT_DBG_SIGNAL_TRACE_POINT(13)
#define EDRV_COUNT_RX_ERR_CRC           TGT_DBG_SIGNAL_TRACE_POINT(14)
#define EDRV_COUNT_RX_ERR_MULT          TGT_DBG_SIGNAL_TRACE_POINT(15)
#define EDRV_COUNT_RX_ERR_SEQ           TGT_DBG_SIGNAL_TRACE_POINT(16)
#define EDRV_COUNT_RX_ERR_OTHER         TGT_DBG_SIGNAL_TRACE_POINT(17)
#define EDRV_COUNT_RX_ORUN              TGT_DBG_SIGNAL_TRACE_POINT(18)

#define EDRV_TRACE_CAPR(x)              TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x06000000)
#define EDRV_TRACE_RX_CRC(x)            TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x0E000000)
#define EDRV_TRACE_RX_ERR(x)            TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x0F000000)
#define EDRV_TRACE_RX_PUN(x)            TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x11000000)
#define EDRV_TRACE(x)                   TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF0000) | 0x0000FEC0)

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

/**
 Advanced Transmit data descriptor union

 Points to buffers used to pass data to the controller
 */
typedef union
{
    struct
    {
        QWORD       m_le_qwBufferAddr;       // Address of descriptor's data buf
        DWORD       m_dwCmdTypeLen;          // Command, Descriptor Type, data length
        DWORD       m_dwStatusIdxPaylen;     // Status, Index , Payload length
    } m_sRead;
    struct
    {
        QWORD       m_le_qwTimeStamp;     // Rsvd or Dma timestamp if enabled
        DWORD       m_le_dwRsvd;          // Reserved
        DWORD       m_le_dwstatus;        // Status
    } m_sWb;
} tEdrvAdvTxDesc;

/**
 Advanced context descriptor structure

 Specifies a launch time for a packet stored in succeeding advanced data descriptor
 */
typedef struct
{
    DWORD           m_dwIpMaclenVlan;
    DWORD           m_dwLaunchTime;        // Launch Time for packet [32:56]
    DWORD           m_dwTucmdType;
    DWORD           m_dwIdxL4lenMss;
} tEdrvContextDesc;

/**
 Advanced Recieve data descriptor union

 points to buffers used to pass data to the device driver from controller
 */
typedef union
{
    struct
    {
        QWORD       m_le_qwBufferAddr;          // Data Buffer address
        QWORD       m_le_qwHeaderAddr;          // Head buffer address
    } sRead;

    struct
    {
        WORD        m_wRssPktType;              // RSS Type, Packet Type
        WORD        m_wHeaderLen;               // Header Len
        DWORD       m_dwRssHash;                // Rss Hash
        DWORD       m_dwExtStatusError;         // Extended Error and Status
        DWORD       m_dwLenVlanTag;             // Data Length, Vlan Tag
    } sWb;
} tEdrvAdvRxDesc;

/**
 Time triggered Send Descriptor

 Structure used to refer to a pair of context descriptor and advanced Tx data descriptor
 pointing to a single data packet with its respective launch time.

 This structure simplifies the queue handling logic in time triggered send mode and
 makes it consistent with older configuration
 */

typedef struct
{
    tEdrvContextDesc    m_CtxtDesc;          // Context Descriptor
    tEdrvAdvTxDesc      m_Advdesc;           // Data Descriptor
} tEdrvTtxDesc;

/**
 Structure for Queue Vector

 Used as reference in ISR to identify the corresponding Tx-Rx queue pair
 */
typedef struct
{
    UINT        m_uiQueueIdx;           // Queue Index
    UINT        m_uiVector;             // Vector Index
    char        m_strName[INTERRUPT_STRING_SIZE];
// Name to be registered for vector
} tEdrvQVector;

// Structure for Bookkeeping DMA address and Length
typedef struct
{
    dma_addr_t  m_DmaAddr;             // DMA address
    BYTE       *m_pVirtAddr;           // Virtual address
    UINT        m_uilen;               // Length of Buffer
} tEdrvPktBuff;

typedef struct
{
    tEdrvQVector    *m_Qvector;             // Pointer to the vector structure for this queue
    INT              m_iIndex;              // Queue index
    dma_addr_t       m_DescDma;             // Dma address for descriptor queue
    void            *m_pDescVirt;           // Virtual address for descriptor queue
    tEdrvTxBuffer   *m_apTxBuffer[EDRV_MAX_TX_DESCRIPTOR];
                                            // Tx buffer Array
    BYTE __iomem    *m_pbBuf;               // pointer to buffer for the Queue
    tEdrvPktBuff    *m_PktBuff;             // Bookkeeping structure
    INT              m_iNextDesc;           // Next Descriptor to used
    INT              m_iNextWb;             // Next descriptor to be Written back
} tEdrvQueue;

typedef struct
{
    struct pci_dev      *m_pPciDev;          // pointer to PCI device structure
    void                *m_pIoAddr;          // pointer to register space of Ethernet controller
    tEdrvQueue          *m_pTxQueue[EDRV_MAX_TX_QUEUES];
                                             // Tx queue array
    tEdrvQueue          *m_pRxQueue[EDRV_MAX_RX_QUEUES];
                                             // Rx Queue Array
    tEdrvQVector        *m_pQvector[EDRV_MAX_QUEUE_VECTOR];
                                             // Vector Array
    BYTE                *m_pbTxBuf;          // Tx Buffer pointer
    BOOL                 m_afTxBufUsed[EDRV_MAX_TX_BUFFERS];
                                             // Array to keep track of used Tx buffers

    unsigned int        m_TxMaxQueue;          // Max Tx Queue
    unsigned int        m_RxMaxQueue;          // Max Rx Queue
    unsigned int        m_NumQVectors;         // No. of Queue vectors (Total = NumQvectors + 1)
    struct msix_entry   *m_pMsixEntry;         // Pointer to MSI-X structure
    // Timer related members
    tEplTimerHdl        m_TimerHdl;            // Timer Handle
    tEplHighResCallback m_HighResTimerCb;      // Timer Callback

    tEdrvInitParam      m_InitParam;           // Initialization Parameters
} tEdrvInstance;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static int EdrvInitOne(struct pci_dev *pPciDev, const struct pci_device_id *pId);

static void EdrvRemoveOne(struct pci_dev *pPciDev);

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static struct pci_device_id aEdrvPciTbl[] = { { 0x8086, 0x1533, PCI_ANY_ID,
        PCI_ANY_ID, 0, 0, 0 },          //I210
        { 0, } };

MODULE_DEVICE_TABLE( pci, aEdrvPciTbl);

tEdrvInstance EdrvInstance_l;

static struct pci_driver EdrvDriver = { .name = DRIVER_NAME, .id_table =
        aEdrvPciTbl, .probe = EdrvInitOne, .remove = EdrvRemoveOne, };

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <edrv>                                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/

//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


//------------------------------------------------------------------------------
/**
 \brief         EdrvInit()

 function for init of the Ethernet controller

 \param     pEdrvInitParam_p    pointer to struct including the init-parameters

 \return    Errorcode

 \retval    kEplSuccessful

 \retval    kEplNoResource

 */
//------------------------------------------------------------------------------
tEplKernel EdrvInit(tEdrvInitParam * pEdrvInitParam_p)
{
    tEplKernel Ret;
    INT iResult;
    INT iIndex;

    Ret = kEplSuccessful;

    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof(EdrvInstance_l));

    if (NULL != pEdrvInitParam_p)
    {
        // save the init data
        EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;
    }

    // clear driver structure
    EPL_MEMSET(&EdrvDriver, 0, sizeof(EdrvDriver));

    EdrvDriver.name = DRIVER_NAME, EdrvDriver.id_table = aEdrvPciTbl, EdrvDriver.probe =
            EdrvInitOne, EdrvDriver.remove = EdrvRemoveOne,

    printk("Registering Driver.....");
    iResult = pci_register_driver(&EdrvDriver);
    if (0 != iResult)
    {
        printk("%s pci_register_driver failed with %d\n", __FUNCTION__,
                iResult);
        Ret = kEplNoResource;
        goto Exit;
    }

    if (NULL == EdrvInstance_l.m_pPciDev)
    {
        printk("%s m_pPciDev=NULL\n", __FUNCTION__);
        Ret = EdrvShutdown();
        Ret = kEplNoResource;
        goto Exit;
    }

    // local MAC address might have been changed in EdrvInitOne
    EPL_MEMCPY(pEdrvInitParam_p->m_abMyMacAddr,
            EdrvInstance_l.m_InitParam.m_abMyMacAddr, 6);

    printk("%s local MAC = ", __FUNCTION__);
    for (iIndex = 0; iIndex < 6; iIndex++)
    {
        printk("%02X ", (unsigned int) pEdrvInitParam_p->m_abMyMacAddr[iIndex]);
    }
    printk("\n");

    Exit: return Ret;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvShutdown()

 Shutdown the Ethernet controller

 \param     void

 \return    Errorcode

 \retval    kEplSuccessful

 */
//------------------------------------------------------------------------------
tEplKernel EdrvShutdown(void)
{
    // unregister PCI driver
    printk("%s calling pci_unregister_driver()\n", __FUNCTION__);
    pci_unregister_driver(&EdrvDriver);

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvDefineRxMacAddrEntry()

 Set a multicast entry into the Ethernet controller

 \param     pbMacAddr_p    pointer to multicast entry to set

 \return    Errorcode

 \retval    kEplSuccessful

 */
//------------------------------------------------------------------------------
tEplKernel EdrvDefineRxMacAddrEntry(BYTE * pbMacAddr_p)
{
    tEplKernel Ret = kEplSuccessful;
    DWORD dwData;
    INT iIndex;

    // entry 0 is used for local MAC address
    for (iIndex = 1; iIndex < 16; iIndex++)
    {
        dwData = EDRV_REGDW_READ(EDRV_RAH(iIndex));
        if (0 == (dwData & EDRV_RAH_AV))
        {          // free MAC address entry
            break;
        }
    }

    if (16 == iIndex)
    {          // no free entry found
        printk("%s Implementation of Multicast Table Array support required\n",
                __FUNCTION__);
        Ret = kEplEdrvInitError;
        goto Exit;
    }
    else
    {
        // write MAC address to free entry
        dwData = 0;
        dwData |= pbMacAddr_p[0] << 0;
        dwData |= pbMacAddr_p[1] << 8;
        dwData |= pbMacAddr_p[2] << 16;
        dwData |= pbMacAddr_p[3] << 24;
        EDRV_REGDW_WRITE(EDRV_RAL(iIndex), dwData);
        dwData = 0;
        dwData |= pbMacAddr_p[4] << 0;
        dwData |= pbMacAddr_p[5] << 8;
        dwData |= EDRV_RAH_AV;
        EDRV_REGDW_WRITE(EDRV_RAH(iIndex), dwData);
    }

    Exit: return Ret;

}

//------------------------------------------------------------------------------
/**
 \brief     EdrvUndefineRxMacAddrEntry()

 SReset a multicast entry in the Ethernet controller

 \param     pbMacAddr_p = pointer to multicast entry to reset

 \return    Errorcode

 \retval    kEplSuccessful

 */
//------------------------------------------------------------------------------
tEplKernel EdrvUndefineRxMacAddrEntry(BYTE * pbMacAddr_p)
{
    tEplKernel Ret = kEplSuccessful;
    DWORD dwData;
    INT iIndex;
    DWORD dwAddrLow;
    DWORD dwAddrHigh;

    dwAddrLow = 0;
    dwAddrLow |= pbMacAddr_p[0] << 0;
    dwAddrLow |= pbMacAddr_p[1] << 8;
    dwAddrLow |= pbMacAddr_p[2] << 16;
    dwAddrLow |= pbMacAddr_p[3] << 24;
    dwAddrHigh = 0;
    dwAddrHigh |= pbMacAddr_p[4] << 0;
    dwAddrHigh |= pbMacAddr_p[5] << 8;
    dwAddrHigh |= EDRV_RAH_AV;

    for (iIndex = 1; iIndex < 16; iIndex++)
    {
        dwData = EDRV_REGDW_READ(EDRV_RAH(iIndex));
        if (dwAddrHigh == (dwData & (EDRV_RAH_AV | 0xFFFF)))
        {
            dwData = EDRV_REGDW_READ(EDRV_RAL(iIndex));
            if (dwData == dwAddrLow)
            {          // set address valid bit to invalid
                EDRV_REGDW_WRITE(EDRV_RAH(iIndex), 0);
                break;
            }
        }
    }

    return Ret;
}

tEplKernel EdrvChangeFilter(tEdrvFilter* pFilter_p, UINT uiCount_p,
        UINT uiEntryChanged_p, UINT uiChangeFlags_p)
{
    tEplKernel Ret = kEplSuccessful;

    return Ret;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvAllocTxMsgBuffer()

 Register a Tx-Buffer

 \param     pBuffer_p   pointer to Buffer structure

 \return    Errorcode

 \retval    kEplSuccessful

 \retval    kEplEdrvNoFreeBufEntry

 */
//------------------------------------------------------------------------------
tEplKernel EdrvAllocTxMsgBuffer(tEdrvTxBuffer *pBuffer_p)
{
    tEplKernel Ret = kEplSuccessful;
    DWORD dwChannel;

    if (pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    if (NULL == EdrvInstance_l.m_pbTxBuf)
    {
        printk("%s Tx buffers currently not allocated\n", __FUNCTION__);
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    for (dwChannel = 0; dwChannel < EDRV_MAX_TX_BUFFERS; dwChannel++)
    {
        if (FALSE == EdrvInstance_l.m_afTxBufUsed[dwChannel])
        {
            // free channel found
            EdrvInstance_l.m_afTxBufUsed[dwChannel] = TRUE;
            pBuffer_p->m_BufferNumber.m_dwVal = dwChannel;
            pBuffer_p->m_pbBuffer = EdrvInstance_l.m_pbTxBuf
                    + (dwChannel * EDRV_MAX_FRAME_SIZE);
            pBuffer_p->m_uiMaxBufferLen = EDRV_MAX_FRAME_SIZE;
            break;
        }
    }

    if (dwChannel >= EDRV_MAX_TX_BUFFERS)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    Exit: return Ret;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvReleaseTxMsgBuffer()

 Return a Tx-Buffer to free

 \param     pBuffer_p   pointer to Buffer structure

 \return    Errorcode

 \retval    kEplSuccessful

 */
//------------------------------------------------------------------------------
tEplKernel EdrvReleaseTxMsgBuffer(tEdrvTxBuffer *pBuffer_p)
{
    UINT uiBufferNumber;

    uiBufferNumber = pBuffer_p->m_BufferNumber.m_dwVal;

    if (uiBufferNumber < EDRV_MAX_TX_BUFFERS)
    {
        EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] = FALSE;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
 \brief         EdrvReadSystim

 I210 specific routine which reads the current SYSTIM register time in
 struct timespec format.

 \param     psTime_p  timespec struct to hold the current time

 \return    DWORD

 \retval    Nano seconds part of current time

 */
//------------------------------------------------------------------------------
static DWORD EdrvReadSystim(struct timespec *psTime_p)
{
    DWORD dwSec, dwNsec, dwPsec;

    dwPsec = EDRV_REGDW_READ(EDRV_SYSTIMR_REG);
    dwNsec = EDRV_REGDW_READ(EDRV_SYSTIML_REG);
    dwSec = EDRV_REGDW_READ(EDRV_SYSTIMH_REG);

    psTime_p->tv_sec = dwSec;
    psTime_p->tv_nsec = dwNsec;

    return dwNsec;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvWriteSystim

 I210 specific routine which writes a value into SYSTIM register

 \param     psTime_p    timespec struct having the time to write

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvWriteSystim(const struct timespec *psTime_p)
{
    EDRV_REGDW_WRITE(EDRV_SYSTIML_REG, psTime_p->tv_nsec);
    EDRV_REGDW_WRITE(EDRV_SYSTIMH_REG, psTime_p->tv_sec);
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvSendTxMsg()

 Handle the packet to be transmitted to the Controller with the specific
 launch time.

 \param     pTxBuffer_p     pointer to TxBuffer structure to be handles to the
                            controller to send.

 \return    Errorcode

 \retval    kEplSuccessful
 \retval    kEplEdrvBufNotExisting
 \retval    kEplEdrvNoFreeTxDesc
 \retval    kEplEdrvNoFreeBufEntry

 */
//------------------------------------------------------------------------------
tEplKernel EdrvSendTxMsg(tEdrvTxBuffer *pTxBuffer_p)
{
    tEplKernel Ret = kEplSuccessful;

    UINT uiBufferNumber;
    tEdrvQueue *pTxQueue;
    INT iQueue = 0, iIndex = 0;
    dma_addr_t TxDma;
    tEdrvTtxDesc *pTtxDesc;

#if EDRV_USE_TTTX != FALSE
    QWORD qwLaunchTime, qwTime;
#endif

    uiBufferNumber = pTxBuffer_p->m_BufferNumber.m_dwVal;

    if ((uiBufferNumber >= EDRV_MAX_TX_BUFFERS)
            || (EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] == FALSE))
    {
        Ret = kEplEdrvBufNotExisting;
        goto Exit;
    }

    pTxQueue = EdrvInstance_l.m_pTxQueue[iQueue];
    iIndex = pTxQueue->m_iNextDesc;

    if (((iIndex + 1) & EDRV_MAX_TTX_DESC_LEN)== pTxQueue->m_iNextWb )
    {
        Ret = kEplEdrvNoFreeTxDesc;
        goto Exit;
    }

    pTtxDesc = EDRV_GET_TTX_DESC(pTxQueue,iIndex) ;

    pTtxDesc->m_CtxtDesc.m_dwIdxL4lenMss = 0;
    pTtxDesc->m_CtxtDesc.m_dwIpMaclenVlan = 0;

#if EDRV_USE_TTTX != FALSE
    qwLaunchTime = pTxBuffer_p->m_qwLaunchTime;

    // Scale the launch time to 32 nsecs unit
    do_div(qwLaunchTime, SEC_TO_NSEC);
    qwTime = pTxBuffer_p->m_qwLaunchTime - (qwLaunchTime * SEC_TO_NSEC);
    do_div(qwTime, 32);
    pTtxDesc->m_CtxtDesc.m_dwLaunchTime = qwTime;
#endif

    // Set descriptor type
    pTtxDesc->m_CtxtDesc.m_dwTucmdType = (EDRV_TDESC_CMD_DEXT
            | EDRV_TDESC_DTYP_CTXT);

    TxDma = dma_map_single(&EdrvInstance_l.m_pPciDev->dev,
            pTxBuffer_p->m_pbBuffer, pTxBuffer_p->m_uiTxMsgLen, DMA_TO_DEVICE);

    if (dma_mapping_error(&EdrvInstance_l.m_pPciDev->dev, TxDma))
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // Store TxBuffer for reference in ISR
    pTxQueue->m_apTxBuffer[iIndex] = pTxBuffer_p;

    EDRV_COUNT_SEND;
    // Store Dma address, length and virtual address for reference
    pTxQueue->m_PktBuff[iIndex].m_DmaAddr = TxDma;
    pTxQueue->m_PktBuff[iIndex].m_pVirtAddr = pTxBuffer_p->m_pbBuffer;
    pTxQueue->m_PktBuff[iIndex].m_uilen = pTxBuffer_p->m_uiTxMsgLen;

    pTtxDesc->m_Advdesc.m_sRead.m_le_qwBufferAddr = cpu_to_le64(TxDma);
    pTtxDesc->m_Advdesc.m_sRead.m_dwCmdTypeLen =
            (unsigned int) pTxBuffer_p->m_uiTxMsgLen;
    pTtxDesc->m_Advdesc.m_sRead.m_dwCmdTypeLen |= (EDRV_TDESC_CMD_DEXT
            | EDRV_TDESC_DTYP_ADV | EDRV_TDESC_CMD_EOP | EDRV_TDESC_CMD_IFCS
            | EDRV_TDESC_CMD_RS);

    pTtxDesc->m_Advdesc.m_sRead.m_dwStatusIdxPaylen = (pTxBuffer_p->m_uiTxMsgLen
            << 14);

    iIndex = ((iIndex + 1) & EDRV_MAX_TTX_DESC_LEN);
    // increment Tx descriptor queue tail pointer
    pTxQueue->m_iNextDesc = iIndex;
    // Handle the frame to Hw
    EDRV_REGDW_WRITE(EDRV_TDTAIL(iQueue), (iIndex * 2));

    Exit: return Ret;

}

//------------------------------------------------------------------------------
/**
 \brief     EdrvReleaseRxBuffer()

 Release a RxBuffer to the Edrv

 \param     pbRxBuffer_p    Pointer to buffer

 \return    None

 \retval    kEplSuccessful

 */
//------------------------------------------------------------------------------
tEplKernel EdrvReleaseRxBuffer(tEdrvRxBuffer *pRxBuffer_p)
{
    tEplKernel Ret = kEplSuccessful;
    return Ret;
}

/**
 * 	Interrupt Handling Routines
 *
 * 	The I210 uses MSI-X interrupt mechanism for interrupt handling.
 * 	Two MSI-X vectors are used by the driver to handle interrupts.
 *
 * 	One for handling Tx-Rx queue cause such as descriptor write back, queue errors etc.
 *
 * 	Second is used to handle timer interrupts and enable the I210 hardware timer as a source
 * 	for HighResk time module
 *
 */
//------------------------------------------------------------------------------
/**
 \brief     EdrvOtherInterrupt()

 ISR for Other interrupt causes ( Timer interrupt). Calls timer callback
 of HighResk module from here.


 \param     iIrqNum_p   System assigned Irq No

 \param     pInstData_p Token passed during interrupt registration

 \return    irqreturn_t

 \retval

 */
//------------------------------------------------------------------------------
static irqreturn_t EdrvOtherInterrupt(INT iIrqNum_p, void *pInstData_p)
{
    DWORD dwStatus;
    DWORD dwReg;
    INT iHandled = IRQ_HANDLED;

    if (pInstData_p != EdrvInstance_l.m_pPciDev)
    {
        iHandled = IRQ_NONE;
        goto Exit;
    }

    dwStatus = EDRV_REGDW_READ(EDRV_INTR_READ_REG);

    if ((dwStatus & EDRV_INTR_ICR_TIME_SYNC)== 0){
        iHandled = IRQ_NONE;
        EDRV_COUNT_PCI_ERR;
        goto Exit;
    }

    dwStatus &= ~EDRV_INTR_ICR_TIME_SYNC;

    EDRV_REGDW_WRITE(EDRV_INTR_SET_REG, dwStatus);
    dwReg = EDRV_REGDW_READ(EDRV_TSICR);

#if EDRV_USE_TTTX != FALSE
    // Call timer CallBack
    if (NULL != EdrvInstance_l.m_HighResTimerCb)
    {
        EdrvInstance_l.m_HighResTimerCb(&EdrvInstance_l.m_TimerHdl);
    }
#endif
    Exit: return iHandled;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvInterruptHandler()

 interrupt handler for Queue causes(Rx - Tx descriptor write back)

 \param     iIrqNum_p       System assigned Irq No

 \param     ppDevInstData_p Token passed during interrupt registration

 \return    Errorcode

 \retval    kEplSuccessful

 */
//------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static irqreturn_t TgtEthIsr(INT nIrqNum_p, void* ppDevInstData_p)
#else
static INT TgtEthIsr (INT nIrqNum_p, void* ppDevInstData_p, struct pt_regs* ptRegs_p)
#endif
{
    DWORD dwReg;
    INT iHandled;
    INT iIndex;
    tEdrvQVector *pQVector = (tEdrvQVector *) ppDevInstData_p;
    tEdrvQueue *pTxQueue;
    tEdrvQueue *pRxQueue;

    iHandled = IRQ_HANDLED;

    if (EdrvInstance_l.m_pQvector[pQVector->m_uiQueueIdx] != pQVector)
    {
        iHandled = IRQ_NONE;
        EDRV_COUNT_PCI_ERR;
        goto Exit;
    }

    // Retrieve the Queue Structure for the vector
    pTxQueue = EdrvInstance_l.m_pTxQueue[pQVector->m_uiQueueIdx];
    pRxQueue = EdrvInstance_l.m_pRxQueue[pQVector->m_uiQueueIdx];

    // Read the interrupt status
    dwReg = EDRV_REGDW_READ(EDRV_INTR_READ_REG);

    // Process Rx with priority over Tx
    if ((pRxQueue != NULL )&& (dwReg & EDRV_INTR_ICR_RXDW) ){
    tEdrvAdvRxDesc* pAdvRxDesc;
    tEdrvRxBuffer RxBuffer;
    iIndex = pRxQueue->m_iNextWb;

    // Clear the Rx write-back bit
    dwReg &= ~EDRV_INTR_ICR_RXDW;

    pAdvRxDesc = EDRV_GET_RX_DESC(pRxQueue,iIndex);

    while((pAdvRxDesc->sWb.m_dwExtStatusError & EDRV_RDESC_STATUS_DD) &&
            (pAdvRxDesc->sWb.m_dwExtStatusError & EDRV_RDESC_STATUS_EOP))
    {
        UINT uiRcvLen;

        if(pAdvRxDesc->sWb.m_dwExtStatusError & EDRV_RDESC_ERRORS_RXE)
        {
            EDRV_COUNT_RX_ERR_CRC;
        }
        else
        {
            // good packet

            RxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;
            uiRcvLen = (0x0000FFFF & pAdvRxDesc->sWb.m_dwLenVlanTag);
            RxBuffer.m_uiRxMsgLen = uiRcvLen;
            RxBuffer.m_pbBuffer = (BYTE *)pRxQueue->m_PktBuff[iIndex].m_pVirtAddr;

            EDRV_COUNT_RX;

            dma_sync_single_for_cpu(&EdrvInstance_l.m_pPciDev->dev,
                    pRxQueue->m_PktBuff[iIndex].m_DmaAddr,
                    uiRcvLen,
                    DMA_FROM_DEVICE);

            // Forward the Rcv packet to DLL
            if( NULL != EdrvInstance_l.m_InitParam.m_pfnRxHandler )
            {
                EdrvInstance_l.m_InitParam.m_pfnRxHandler(&RxBuffer);
            }

        }

        // Prepare descriptor for next Rcv
        pAdvRxDesc->sRead.m_le_qwBufferAddr = cpu_to_le64(pRxQueue->m_PktBuff[iIndex].m_DmaAddr);
        pAdvRxDesc->sRead.m_le_qwHeaderAddr = 0;

        // Handle the processed decriptor to Hw
        EDRV_REGDW_WRITE(EDRV_RDTAIL(pRxQueue->m_iIndex),iIndex);

        // Increment the pointer
        iIndex =((iIndex + 1) & EDRV_MAX_RX_DESC_LEN);
        pRxQueue->m_iNextWb = iIndex;
        pAdvRxDesc = EDRV_GET_RX_DESC(pRxQueue,iIndex);

    }

}

    // Process Tx
    if ((pTxQueue != NULL )&& (dwReg & EDRV_INTR_ICR_TXDW) ){

    tEdrvTtxDesc *pTtxDesc;
    tEdrvAdvTxDesc* pAdvTxDesc;

    // Clear Tx write-Back bit
    dwReg &= ~EDRV_INTR_ICR_TXDW;

    iIndex = 0;
    iIndex = pTxQueue->m_iNextWb;

    pTtxDesc = EDRV_GET_TTX_DESC(pTxQueue,iIndex);
    pAdvTxDesc = &(pTtxDesc->m_Advdesc);

    do
    {
        if ( pAdvTxDesc->m_sWb.m_le_dwstatus & EDRV_TDESC_STATUS_DD )
        {
            // Process the send packet
            tEdrvTxBuffer* pTxBuffer;
            DWORD dwTxStatus;

            dwTxStatus = pAdvTxDesc->m_sWb.m_le_dwstatus;
            pAdvTxDesc->m_sWb.m_le_dwstatus = 0;
            pTxBuffer = pTxQueue->m_apTxBuffer[iIndex];
            pTxQueue->m_apTxBuffer[iIndex] = NULL;

            dma_unmap_single(&EdrvInstance_l.m_pPciDev->dev,
                    pTxQueue->m_PktBuff[iIndex].m_DmaAddr,
                    pTxQueue->m_PktBuff[iIndex].m_uilen,
                    DMA_TO_DEVICE);
            EDRV_COUNT_TX;
            if ( NULL != pTxBuffer )
            {
                // Call Tx handler of Data link layer
                if (NULL != pTxBuffer->m_pfnTxHandler )
                {
                    pTxBuffer->m_pfnTxHandler(pTxBuffer);
                }
            }
            else
            {
                EDRV_COUNT_TX_FUN;
            }

            iIndex =((iIndex + 1) & EDRV_MAX_TTX_DESC_LEN);

            pTxQueue->m_iNextWb = iIndex;

            pTtxDesc = EDRV_GET_TTX_DESC(pTxQueue,iIndex);
            pAdvTxDesc = &(pTtxDesc->m_Advdesc);

        }
        else
        {
            // no completed Packet to process
            break;
        }

    }while(pAdvTxDesc->m_sWb.m_le_dwstatus & EDRV_TDESC_STATUS_DD);
}

    // Set the values in ICR again which were not processed here so that they are available
    // for processing in other ISR
    EDRV_REGDW_WRITE(EDRV_INTR_SET_REG, dwReg);

    Exit: return iHandled;

}

//------------------------------------------------------------------------------
/**
 \brief     EdrvPutHwSemaphoreGeneric()

 I210 specific routine used to release the semaphore acquired before

 \param     void

 \return    void

 */
//------------------------------------------------------------------------------
void EdrvPutHwSemaphoreGeneric(void)
{
    DWORD dwSwsm;

    dwSwsm = EDRV_REGDW_READ(EDRV_SWSM_REG);

    dwSwsm &= ~(EDRV_SWSM_SMBI | EDRV_SWSM_SWESMBI);

    EDRV_REGDW_WRITE(EDRV_SWSM_REG, dwSwsm);
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvGetHwSemaphore

 I210 specific routine used to acquire the Hardware semaphore to access
 the PHY registers

 \param     void

 \return    void

 */
//------------------------------------------------------------------------------
tEplKernel EdrvGetHwSemaphore(void)
{
    DWORD dwSwsm;
    tEplKernel iResult = kEplSuccessful;
    INT iTimeout;
    INT iIndex = 0;

    iTimeout = EDRV_PHY_SEMPHORE_TIMEOUT;

    // Get the FW semaphore.
    for (iIndex = 0; iIndex < iTimeout; iIndex++)
    {
        dwSwsm = EDRV_REGDW_READ(EDRV_SWSM_REG);
        EDRV_REGDW_WRITE(EDRV_SWSM_REG, dwSwsm | EDRV_SWSM_SWESMBI);

        // Semaphore acquired if bit latched
        if (EDRV_REGDW_READ(EDRV_SWSM_REG) & EDRV_SWSM_SWESMBI)
        {
            break;
        }
        udelay(50);
    }

    if (iIndex == iTimeout)
    {
        // Release semaphores
        EdrvPutHwSemaphoreGeneric();
        iResult = kEplNoResource;
        goto Exit;
    }

    Exit: return iResult;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvPutHwSemaphore

 I210 specific routine used to release hardware semaphore used to
 access the PHY

 \param     void

 \return    void

 */
//------------------------------------------------------------------------------
void EdrvPutHwSemaphore(void)
{
    DWORD dwSwsm;

    dwSwsm = EDRV_REGDW_READ(EDRV_SWSM_REG);

    dwSwsm &= ~EDRV_SWSM_SWESMBI;

    EDRV_REGDW_WRITE(EDRV_SWSM_REG, dwSwsm);
}

//------------------------------------------------------------------------------
/**
 \brief    EdrvAcquireSwfwSync

 I210 specific routine used to acquire the Software/Firmware semaphore
 to access the PHY.

 \param     wMask_p      Specifies which semaphore to acquire

 \return    tEplKernel

 \retval    kEplSuccessful

 \retval    kEplNoResource

 */
//------------------------------------------------------------------------------
tEplKernel EdrvAcquireSwfwSync(WORD wMask_p)
{
    DWORD dwSwfwSync;
    DWORD dwSwmask = wMask_p;
    DWORD dwFwmask = wMask_p << 16;
    tEplKernel iResult = kEplSuccessful;
    INT iIndex = 0, iTimeout = 200;

    while (iIndex < iTimeout)
    {
        if (EdrvGetHwSemaphore())
        {
            iResult = kEplNoResource;
            goto Exit;
        }

        dwSwfwSync = EDRV_REGDW_READ(EDRV_SW_FW_SYNC);
        if (!(dwSwfwSync & dwFwmask))
            break;

        // Firmware currently using resource (dwFwmask)
        EdrvPutHwSemaphore();
        mdelay(5);
        iIndex++;
    }

    if (iIndex == iTimeout)
    {
        printk("Driver can't access resource, SW_FW_SYNC timeout.\n");
        iResult = kEplNoResource;
        goto Exit;
    }

    dwSwfwSync |= dwSwmask;
    EDRV_REGDW_WRITE(EDRV_SW_FW_SYNC, dwSwfwSync);

    EdrvPutHwSemaphore();

    Exit: return iResult;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvReleaseSwfwSync

 I210 specific routine used to release the Software/Firmware semaphore
 used to access the PHY.

 \param     wMask_p     Specifies which semaphore to release

 \return    void

 */
//------------------------------------------------------------------------------
void EdrvReleaseSwfwSync(WORD wMask_p)
{
    DWORD dwSwfwSync;

    while (EdrvGetHwSemaphore() != 0)
        ;

    dwSwfwSync = EDRV_REGDW_READ(EDRV_SW_FW_SYNC);
    dwSwfwSync &= ~wMask_p;
    EDRV_REGDW_WRITE(EDRV_SW_FW_SYNC, dwSwfwSync);

    EdrvPutHwSemaphore();
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvMdicWrite()

 I210 specific routine used to write into PHY register through MDIO interface

 \param     iPhyreg_p   Phy Register to write
 \param		wValue_p    Value to write

 \return 	Errorcode

 \retval 	kEplSuccessful

 */
//------------------------------------------------------------------------------
static void EdrvMdicWrite(unsigned int iPhyreg_p, unsigned short wValue_p)
{
    DWORD dwRegVal = 0;
    DWORD dwMdicRd;

    dwRegVal &= ~(EDRV_MDIC_DATA_MASK | EDRV_MDIC_REGADD_MASK);
    dwRegVal |= wValue_p;
    dwRegVal |= (iPhyreg_p << 16);
    dwRegVal |= EDRV_MDIC_OP_WRITE;
    dwRegVal |= EDRV_MDIC_INTR_DIS;
    dwRegVal &= ~EDRV_MDIC_RD;

    EDRV_REGDW_WRITE(EDRV_MDIC_REG, dwRegVal);
    // wait for completion of transfer
    do
    {
        udelay(50);
        dwMdicRd = EDRV_REGDW_READ(EDRV_MDIC_REG);
    } while ((dwMdicRd & EDRV_MDIC_RD) == 0);          //wait PHYIDLE is set 1

}

//------------------------------------------------------------------------------
/**
 \brief     EdrvMdicRead()

 I210 specific routine used to read from PHY register through MDIO interface

 \param     Phyreg_p    PHY register

 \return    WORD

 \retval    Value read from register

 */
//------------------------------------------------------------------------------
static WORD EdrvMdicRead(INT iPhyreg_p)
{

    DWORD dwRegVal = 0;
    WORD wValue = 0;
    DWORD dwMdicRd;

    dwRegVal &= ~(EDRV_MDIC_DATA_MASK | EDRV_MDIC_REGADD_MASK);
    dwRegVal |= (iPhyreg_p << 16);
    dwRegVal |= EDRV_MDIC_OP_READ;
    dwRegVal |= EDRV_MDIC_INTR_DIS;
    dwRegVal &= ~EDRV_MDIC_RD;

    EDRV_REGDW_WRITE(EDRV_MDIC_REG, dwRegVal);

    // wait till the value is being read
    do
    {
        cpu_relax();
        dwMdicRd = EDRV_REGDW_READ(EDRV_MDIC_REG);
    } while ((dwMdicRd & EDRV_MDIC_RD) == 0);          //wait RD is set 1

    wValue = (dwMdicRd & EDRV_MDIC_DATA_MASK);

    return wValue;

}

//------------------------------------------------------------------------------
/**
 \brief     EdrvFreeTxQueueBuffer

 I210 specific routine used to free the buffers for queue

 \param     pTxQueue_p   	Queue to be processed

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvFreeTxQueueBuffer(tEdrvQueue *pTxQueue_p)
{
    INT iIndex;
    for (iIndex = 0; iIndex < EDRV_MAX_RX_DESCRIPTOR; iIndex++)
    {
        // set report status for all descriptor
        if (pTxQueue_p->m_PktBuff[iIndex].m_DmaAddr)
        {
            dma_unmap_single(&EdrvInstance_l.m_pPciDev->dev,
                    pTxQueue_p->m_PktBuff[iIndex].m_DmaAddr,
                    pTxQueue_p->m_PktBuff[iIndex].m_uilen, DMA_TO_DEVICE);
        }

    }
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvFreeTxBuffers

 I210 specific routine used to free all Transmit buffers

 \param     void

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvFreeTxBuffers(void)
{
    INT iIndex;
    for (iIndex = 0; iIndex < EdrvInstance_l.m_TxMaxQueue; iIndex++)
    {
        EdrvFreeTxQueueBuffer(EdrvInstance_l.m_pTxQueue[iIndex]);
    }
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvFreeTxQueues

 I210 specific routine used to free the memory for Tx queues

 \param     void

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvFreeTxQueues(void)
{
    tEdrvQueue *pTxQueue;
    UINT uiIndex;

    for (uiIndex = 0; uiIndex < EdrvInstance_l.m_TxMaxQueue; uiIndex++)
    {
        pTxQueue = EdrvInstance_l.m_pTxQueue[uiIndex];

        if (NULL == pTxQueue)
        {
            break;
        }
        if (NULL != pTxQueue->m_pbBuf)
        {
            kfree(pTxQueue->m_pbBuf);
        }

        if (NULL != pTxQueue->m_PktBuff)
        {
            kfree(pTxQueue->m_PktBuff);
        }

        if (NULL != pTxQueue->m_pDescVirt)
        {
            dma_free_coherent(&EdrvInstance_l.m_pPciDev->dev,
                    EDRV_TX_DESCS_SIZE, pTxQueue->m_pDescVirt,
                    pTxQueue->m_DescDma);
            kfree(pTxQueue);
            EdrvInstance_l.m_pTxQueue[uiIndex] = NULL;

        }

    }

    EdrvInstance_l.m_TxMaxQueue = 0;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvInitTxQueue

 I210 specific routine used to initialize the Tx queue with descriptors
 and memory resources

 \param     pTxQueue_p   Pointer to the queue structure to initialize

 \return    tEplKernel   Error code

 */
//------------------------------------------------------------------------------
static tEplKernel EdrvInitTxQueue(tEdrvQueue *pTxQueue_p)
{
    tEplKernel iResult = kEplSuccessful;
    INT iDescSize;

    // Allocate Tx Descriptor
    printk("{%s}:Allocating Tx Desc %p\n", __FUNCTION__, pTxQueue_p);
    iDescSize = ALIGN(EDRV_TX_DESCS_SIZE, 4096);
    pTxQueue_p->m_pDescVirt = dma_alloc_coherent(&EdrvInstance_l.m_pPciDev->dev,
            iDescSize, &pTxQueue_p->m_DescDma, GFP_KERNEL);
    if (NULL == pTxQueue_p->m_pDescVirt)
    {
        iResult = kEplEdrvInitError;
        goto Exit;
    }
    printk("... Done\n");

    // Clear the descriptor memory
    memset(pTxQueue_p->m_pDescVirt, 0, EDRV_TX_DESCS_SIZE);

    pTxQueue_p->m_PktBuff = kzalloc(
            (EDRV_MAX_TX_DESCRIPTOR * sizeof(tEdrvPktBuff)), GFP_KERNEL);
    if (NULL == pTxQueue_p->m_PktBuff)
    {
        iResult = kEplEdrvInitError;
        goto Exit;
    }
    pTxQueue_p->m_iNextDesc = 0;
    pTxQueue_p->m_iNextWb = 0;

    // Map Queue to its Vector;
    pTxQueue_p->m_Qvector = EdrvInstance_l.m_pQvector[pTxQueue_p->m_iIndex];

    Exit: return iResult;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvConfigureTxQueue

 I210 specific routine used to configure Transmit queue

 \param     pTxQueue_p  Pointer to the queue structure to configure

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvConfigureTxQueue(tEdrvQueue *pTxQueue_p)
{
    QWORD qwTxDescDma;
    DWORD dwReg;
    INT iQueue = pTxQueue_p->m_iIndex;
    INT iIndex;

    printk("Configure Tx Queue %d", iQueue);
    // Disable the Queue
    EDRV_REGDW_WRITE(EDRV_TXDCTL(iQueue), 0);

    // Program the hw with queue values
    EDRV_REGDW_READ(EDRV_STATUS_REG);
    mdelay(10);

    qwTxDescDma = pTxQueue_p->m_DescDma;

    // Initialize the Queue parameters in  controller
    EDRV_REGDW_WRITE(EDRV_TDLEN(iQueue), EDRV_TX_DESCS_SIZE);
    EDRV_REGDW_WRITE(EDRV_TDBAL(iQueue), (qwTxDescDma & 0x00000000ffffffffULL));
    EDRV_REGDW_WRITE(EDRV_TDBAH(iQueue), (qwTxDescDma >> 32));

    EDRV_REGDW_WRITE(EDRV_TDHEAD(iQueue), 0);
    EDRV_REGDW_WRITE(EDRV_TDTAIL(iQueue), 0);

    // Setup the Threshold values for queue and Enable
    dwReg = 0;
    dwReg |= (EDRV_TXDCTL_PTHRESH | (EDRV_TXDCTL_HTHRESH << 8)
            | (EDRV_TXDCTL_WTHRESH << 16));

    // Highest priority to Queue0
    if (0 == iQueue)
    {
        dwReg |= EDRV_TXDCTL_PRIORITY;
    }
    dwReg |= EDRV_TXDCTL_QUEUE_EN;

    EDRV_REGDW_WRITE(EDRV_TXDCTL(iQueue), dwReg);
    // Poll till queue gets enabled
    printk("Poll TXDCTL");
    for (iIndex = 0; iIndex < EDRV_POLL_TIMEOUT; iIndex++)
    {
        if ((EDRV_REGDW_READ(EDRV_TXDCTL(iQueue)) & EDRV_TXDCTL_QUEUE_EN))
        {
            break;
        }
        msleep(1);
    }
    if (iIndex == EDRV_POLL_TIMEOUT)
    {
        printk("...Fail\n");
    }
    else
    {
        printk("....Done\n");
    }

}

//------------------------------------------------------------------------------
/**
 \brief     EdrvFreeRxQueues

 I210 specific routine used to free the memory for Rx queues

 \param     void

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvFreeRxQueues(void)
{
    tEdrvQueue *pRxQueue = NULL;
    UINT uiIndex;

    for (uiIndex = 0; uiIndex < EdrvInstance_l.m_RxMaxQueue; uiIndex++)
    {

        pRxQueue = EdrvInstance_l.m_pRxQueue[uiIndex];

        if (NULL == pRxQueue)
        {
            break;
        }
        if (NULL != pRxQueue->m_pbBuf)
        {
            kfree(pRxQueue->m_pbBuf);
        }

        if (NULL != pRxQueue->m_PktBuff)
        {
            kfree(pRxQueue->m_PktBuff);
        }

        if (NULL != pRxQueue->m_pDescVirt)
        {
            dma_free_coherent(&EdrvInstance_l.m_pPciDev->dev,
                    EDRV_TX_DESCS_SIZE, pRxQueue->m_pDescVirt,
                    pRxQueue->m_DescDma);
            kfree(pRxQueue);
            EdrvInstance_l.m_pRxQueue[uiIndex] = NULL;
        }

    }

    EdrvInstance_l.m_RxMaxQueue = 0;
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvFreeRxQueueBuffer

 I210 specific routine used to free memory allocated for Receive buffers

 \param     pRxQueue_p  Pointer to the queue structure to process

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvFreeRxQueueBuffer(tEdrvQueue *pRxQueue_p)
{
    INT iIndex;

    for (iIndex = 0; iIndex < EDRV_MAX_RX_DESCRIPTOR; iIndex++)
    {
        // set report status for all descriptor
        dma_unmap_single(&EdrvInstance_l.m_pPciDev->dev,
                pRxQueue_p->m_PktBuff[iIndex].m_DmaAddr, EDRV_MAX_FRAME_SIZE,
                DMA_FROM_DEVICE);

    }
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvFreeRxBuffers

 I210 specific routine used to free memory allocated for Receive buffers

 \param     void

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvFreeRxBuffers(void)
{
    INT iIndex;
    for (iIndex = 0; iIndex < EdrvInstance_l.m_RxMaxQueue; iIndex++)
    {
        EdrvFreeRxQueueBuffer(EdrvInstance_l.m_pRxQueue[iIndex]);
    }
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvInitRxQueue

 I210 specific routine used to initialize the Rx queue with descriptors and memory resources

 \param     pRxQueue_p  Pointer to the queue structure to initialize

 \return    tEplKernel  Error code

 */
//------------------------------------------------------------------------------
static tEplKernel EdrvInitRxQueue(tEdrvQueue *pRxQueue_p)
{
    tEplKernel Ret = kEplSuccessful;
    INT iDescSize;

    // Allocate Tx Descriptor
    printk("{%s}:Allocating Rx Desc %p\n", __FUNCTION__, pRxQueue_p);
    iDescSize = ALIGN(EDRV_RX_DESCS_SIZE, 4096);
    pRxQueue_p->m_pDescVirt = dma_alloc_coherent(&EdrvInstance_l.m_pPciDev->dev,
            iDescSize, &pRxQueue_p->m_DescDma, GFP_KERNEL);
    if (NULL == pRxQueue_p->m_pDescVirt)
    {
        Ret = kEplEdrvInitError;
        goto Exit;
    }
    printk("... Done\n");

    // Clear the descriptor memory
    memset(pRxQueue_p->m_pDescVirt, 0, EDRV_RX_DESCS_SIZE);

    pRxQueue_p->m_PktBuff = kzalloc(
            EDRV_MAX_RX_DESCRIPTOR * sizeof(tEdrvPktBuff), GFP_KERNEL);
    if (NULL == pRxQueue_p->m_PktBuff)
    {
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    pRxQueue_p->m_pbBuf = kzalloc(EDRV_RX_BUFFER_SIZE, GFP_KERNEL);

    if (NULL == pRxQueue_p->m_pbBuf)
    {
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    // Initialize head & tail
    pRxQueue_p->m_iNextDesc = 0;
    pRxQueue_p->m_iNextWb = 0;

    // Map Queue to its Vector;
    pRxQueue_p->m_Qvector = EdrvInstance_l.m_pQvector[pRxQueue_p->m_iIndex];

    Exit: return Ret;
}
//------------------------------------------------------------------------------
/**
 \brief     EdrvAllocRxBuffer

 I210 specific routine used to allocate receive buffers and associate
 with corresponding descriptor

 \param     pRxQueue_p  Pointer to the queue structure to allocate buffers

 \return    tEplKernel  Error code

 */
//------------------------------------------------------------------------------
static tEplKernel EdrvAllocRxBuffer(tEdrvQueue *pRxQueue_p)
{
    INT iIndex;
    dma_addr_t RxDma;
    tEplKernel Ret = kEplSuccessful;
    tEdrvAdvRxDesc *RxDesc;

    for (iIndex = 0; iIndex < EDRV_MAX_RX_DESCRIPTOR; iIndex++)
    {

        RxDesc = EDRV_GET_RX_DESC(pRxQueue_p,iIndex) ;

        RxDma = dma_map_single(&EdrvInstance_l.m_pPciDev->dev,
                (pRxQueue_p->m_pbBuf + (iIndex * EDRV_MAX_FRAME_SIZE)),
                EDRV_MAX_FRAME_SIZE,DMA_FROM_DEVICE );

        if (dma_mapping_error(&EdrvInstance_l.m_pPciDev->dev,RxDma))
        {
            Ret = kEplEdrvInitError;
            goto Exit;
        }
        pRxQueue_p->m_PktBuff[iIndex].m_DmaAddr = RxDma;
        pRxQueue_p->m_PktBuff[iIndex].m_pVirtAddr = (pRxQueue_p->m_pbBuf + (iIndex * EDRV_MAX_FRAME_SIZE));
	    RxDesc->sRead.m_le_qwBufferAddr = cpu_to_le64(RxDma);

	    EDRV_REGDW_WRITE(EDRV_RDTAIL(pRxQueue_p->m_iIndex),iIndex);
	 }
    Exit: return Ret;
}
//------------------------------------------------------------------------------
/**
 \brief     EdrvConfigureRxQueue

 I210 specific routine used to configure Receive queue

 \param     pRxQueue_p  Pointer to the queue structure to configure

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvConfigureRxQueue(tEdrvQueue *pRxQueue_p)
{
    QWORD qwRxDescDma;
    DWORD dwReg;
    INT iQueue = pRxQueue_p->m_iIndex;
    INT iIndex;

    printk("Configure Rx Queue %d", iQueue);
    // Disable the Queue
    EDRV_REGDW_WRITE(EDRV_RXDCTL(iQueue), 0);

    EDRV_REGDW_READ(EDRV_STATUS_REG);
    mdelay(10);

    // Program the hw with queue values
    qwRxDescDma = pRxQueue_p->m_DescDma;
    EDRV_REGDW_WRITE(EDRV_RDLEN(iQueue), EDRV_RX_DESCS_SIZE);
    EDRV_REGDW_WRITE(EDRV_RDBAL(iQueue), (qwRxDescDma & 0x00000000ffffffffULL));
    EDRV_REGDW_WRITE(EDRV_RDBAH(iQueue), (qwRxDescDma >> 32));

    EDRV_REGDW_WRITE(EDRV_RDHEAD(iQueue), 0);
    EDRV_REGDW_WRITE(EDRV_RDTAIL(iQueue), 0);

    // Configure Split-Receive register for queue
    dwReg = EDRV_REGDW_READ(EDRV_SRRCTL(iQueue));
    //dwReg |= EDRV_SRRCTL_DROP_EN;
    dwReg |= EDRV_SRRCTL_DESCTYPE_ADV;          // Advance mode with no packet split
    EDRV_REGDW_WRITE(EDRV_SRRCTL(iQueue), dwReg);

    // Enable the rx queue
    dwReg = 0;
    dwReg = (EDRV_RXDCTL_PTHRESH | (EDRV_RXDCTL_HTHRESH << 8)
            | (EDRV_RXDCTL_WTHRESH << 16));
    dwReg |= EDRV_RXDCTL_QUEUE_EN;
    EDRV_REGDW_WRITE(EDRV_RXDCTL(iQueue), dwReg);

    printk("Poll RXDCTL");
    for (iIndex = 0; iIndex < EDRV_POLL_TIMEOUT; iIndex++)
    {
        dwReg = EDRV_REGDW_READ(EDRV_RXDCTL(iQueue));
        if ((dwReg & EDRV_RXDCTL_QUEUE_EN))
        {
            break;
        }
        EDRV_REGDW_WRITE(EDRV_RXDCTL(iQueue), dwReg);
        msleep(1);
    }

    if (iIndex == EDRV_POLL_TIMEOUT)
    {
        printk("...Fail\n");
    }
    else
    {
        printk("....Done\n");
    }
}
//------------------------------------------------------------------------------
/**
 \brief     EdrvInitQavMode

 I210 specific routine used to configure the device for Qav mode using
 1 Tx queue as SR queue with launch time logic

 \param     void

 \return    tEplKernel  Error code

 */
//------------------------------------------------------------------------------
static void EdrvInitQavMode(void)
{
    DWORD dwTqavctrl;
    DWORD dwLaunchOff;
    DWORD dwTqavcc0;
    DWORD dwRxpbsize;

    // reconfigure the rx packet buffer allocation to 30k
    dwRxpbsize = EDRV_REGDW_READ(EDRV_RXPBSIZE_REG);
    dwRxpbsize &= ~EDRV_RXPBSIZE_CLEAR;
    dwRxpbsize |= EDRV_RXPBSIZE_DEF;
    EDRV_REGDW_WRITE(EDRV_RXPBSIZE_REG, dwRxpbsize);

    // DMA Configuration
    EDRV_REGDW_WRITE(EDRV_DTX_MAX_PKTSZ_REG, EDRV_MAX_FRAME_SIZE/64);

    // Configure Q0 as SR queue
    dwTqavcc0 = EDRV_TQAVCC_QUEUE_MODE_SR; /* no idle slope */
    EDRV_REGDW_WRITE(EDRV_TQAVCC(0), dwTqavcc0);

    dwTqavctrl = 0;
    dwTqavctrl = EDRV_TQAVCTRL_TXMODE | EDRV_TQAVCTRL_FETCH_ARB
            | EDRV_TQAVCTRL_TRANSTIM | EDRV_TQAVCTRL_SP_WAIT_SR
            | EDRV_TQAVCTRL_1588_STAT_EN;

    // default to a 10 usec prefetch delta from launch time
    dwTqavctrl |= (10 << 5) << EDRV_TQAVCTRL_FETCH_TM_SHIFT;

    EDRV_REGDW_WRITE(EDRV_TQAVCTRL_REG, dwTqavctrl);

    dwLaunchOff = 0;
    dwLaunchOff |= (4 << 5) << EDRV_LAUNCH_OSO_SHIFT;

    EDRV_REGDW_WRITE(EDRV_LAUNCH_OSO, dwLaunchOff);

}

//------------------------------------------------------------------------------
/**
 \brief     EdrvWriteIvar

 I210 specific routine for interrupt vector mapping of Tx-Rx causes
 in the IVAR register

 \param     iVector_p   Vector No to be assigned
 \param     iIndex_p    Index to determine the queue
 \param     iOffset_p   offset for the queue

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvWriteIvar(INT iVector_p, INT iIndex_p, INT iOffset_p)
{
    DWORD dwIvar = EDRV_REGDW_READ((EDRV_IVAR0_REG + (iIndex_p << 2)));

    // clear any bits that are currently set
    dwIvar &= ~((u32) 0xFF << iOffset_p);

    // write vector and valid bit
    dwIvar |= (iVector_p | EDRV_IVAR_VALID) << iOffset_p;

    EDRV_REGDW_WRITE((EDRV_IVAR0_REG + (iIndex_p << 2)), dwIvar);
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvRequestMsix

 I210 specific routine used to request ISR for MSI-X vectors and
 map vectors to respective causes

 \param     void

 \return    INT     Error Code

 */
//------------------------------------------------------------------------------
static INT EdrvRequestMsix(void)
{
    INT iVector = 0, iIndex = 0, dwReg;
    INT iReturn = 0, iTxQueue, iRxQueue;
    tEdrvQVector *pQvector;

    // Other cause interrupt
    iReturn = request_irq(EdrvInstance_l.m_pMsixEntry[iVector].vector,
            EdrvOtherInterrupt, 0, DRIVER_NAME, EdrvInstance_l.m_pPciDev);
    if (0 != iReturn)
    {
        goto Exit;
    }

    iVector++;
    // Queue interrupts
    for (iIndex = 0; iIndex < EdrvInstance_l.m_NumQVectors; iIndex++, iVector++)
    {
        pQvector = EdrvInstance_l.m_pQvector[iIndex];
        iReturn = request_irq(EdrvInstance_l.m_pMsixEntry[iVector].vector,
                TgtEthIsr, 0, pQvector->m_strName, pQvector);
        if (0 != iReturn)
        {
            goto Exit;
        }
    }

    // Configure MSI-X

    dwReg = 0;
    dwReg = (EDRV_INTR_GPIE_MULT_MSIX |\
                EDRV_INTR_GPIE_PBA |\
                EDRV_INTR_GPIE_NSICR);

    EDRV_REGDW_WRITE(EDRV_INTR_GPIE_REG, dwReg);
    iVector = 0;
    // Enable Msi-x for Other Cause;
    dwReg = EDRV_REGDW_READ(EDRV_IVAR_MISC);
    dwReg |= ((iVector | EDRV_IVAR_VALID) << 8);
    EDRV_REGDW_WRITE(EDRV_IVAR_MISC, dwReg);

    iVector++;
    // Map Tx0 and Rx0 Interrupt cause
    for (iIndex = 0; iIndex < EdrvInstance_l.m_NumQVectors; iIndex++, iVector++)
    {
        iTxQueue = EdrvInstance_l.m_pTxQueue[iIndex]->m_iIndex;
        EdrvWriteIvar(iVector, iTxQueue >> 1, ((iTxQueue & 0x1) << 4) + 8);
        iRxQueue = EdrvInstance_l.m_pRxQueue[iIndex]->m_iIndex;
        EdrvWriteIvar(iVector, iRxQueue >> 1, ((iRxQueue & 0x1) << 4));

    }

    Exit: return iReturn;
}

//------------------------------------------------------------------------------
/**
 \brief 			EdrvInitOne

 initializes one PCIe device

 \param 			pPciDev 		pointer to corresponding PCI device structure

 \param			pId     PCI device ID

 \return 		(int)

 \retval 		0 		 No error

 \retval 		Non-Zero error code

 */
//------------------------------------------------------------------------------
static INT EdrvInitOne(struct pci_dev *pPciDev, const struct pci_device_id *pId)
{
    INT iResult = 0;
    tEdrvQueue *pQueue;
    tEdrvQVector *pVector;
    DWORD dwReg;
    INT iIndex;
    INT iNumVectors;
    struct timespec sSysTime;

    if (NULL != EdrvInstance_l.m_pPciDev)
    {
        // Perform a sanity check to avoid insertion of module again
        printk("%s device %s discarded\n", __FUNCTION__, pci_name(pPciDev));
        iResult = -ENODEV;
        goto Exit;
    }

    // Enable device
    iResult = pci_enable_device(pPciDev);
    if (0 != iResult)
    {
        goto Exit;
    }

    EdrvInstance_l.m_pPciDev = pPciDev;

    if (NULL == EdrvInstance_l.m_pPciDev)
    {
        printk("%s pPciDev==NULL\n", __FUNCTION__);
    }

    iResult = dma_set_mask(&EdrvInstance_l.m_pPciDev->dev, DMA_BIT_MASK(64));
    if (0 == iResult)
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
        dma_set_coherent_mask(&EdrvInstance_l.m_pPciDev->dev, DMA_BIT_MASK(64));
#else
        pci_set_dma_mask(pPciDev, DMA_BIT_MASK(64));
#endif
    }
    else
    {
        printk(" Using 32 bit Mask\n");
        iResult = dma_set_mask(&pPciDev->dev, DMA_BIT_MASK(32));
        if (0 == iResult)
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
            iResult = dma_set_coherent_mask(&EdrvInstance_l.m_pPciDev->dev,
                    DMA_BIT_MASK(32));
#else
            iResult = pci_set_dma_mask(pPciDev, DMA_BIT_MASK(32));
#endif
            if (0 != iResult)
            {
                printk("[EPLi210]: No usable DMA configuration available\n");
                goto ExitFail;
            }
        }
    }

    iResult = pci_request_regions(pPciDev, DRIVER_NAME);

    if (0 != iResult)
    {
        printk("pci_request_regions....Failed\n");
        goto ExitFail;

    }

    pci_set_master(pPciDev);

    EdrvInstance_l.m_pIoAddr = ioremap(pci_resource_start(pPciDev, 0),
            pci_resource_len(pPciDev, 0));
    if (NULL == EdrvInstance_l.m_pIoAddr)
    {
        iResult = -EIO;

        goto ExitFail;
    }
    printk("EdrvInstance_l.m_pIoAddr :0X%p\n", EdrvInstance_l.m_pIoAddr);

    // disable the interrupts
    EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR, EDRV_EIMC_CLEAR_ALL);
    EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR, ~0);
    EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);

    // Disable the Master
    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    dwReg |= (EDRV_CTRL_MASTER_DIS);
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, dwReg);

    for (iIndex = EDRV_MASTER_DIS_TIMEOUT; iIndex > 0; iIndex--)
    {
        if ((EDRV_REGDW_READ(EDRV_STATUS_REG) & EDRV_STATUS_MASTER_EN)== 0){
            break;
        }
        msleep(1);
    }

    if (0 == iIndex)
    {
        iResult = -EIO;
        goto ExitFail;
    }

    // disable the interrupts
    EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR, EDRV_EIMC_CLEAR_ALL);
    EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR, ~0);
    EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);

    EDRV_REGDW_READ(EDRV_STATUS_REG);

    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    dwReg |= (EDRV_CTRL_DEV_RST);
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, dwReg);
    msleep(5);

    iIndex = 0;
    while (iIndex < EDRV_AUTO_READ_DONE_TIMEOUT)
    {
        if (EDRV_REGDW_READ(EDRV_EECD_REG) & EDRV_EECD_AUTO_RD)
        {
            break;
        }

        msleep(1);
        iIndex++;
    }

    if (iIndex == EDRV_AUTO_READ_DONE_TIMEOUT)
    {
        printk("Auto read by HW from NVM has not completed.\n");
        goto ExitFail;
    }

    EDRV_REGDW_WRITE(EDRV_STATUS_REG, EDRV_STATUS_DEV_RST_SET);

    // disable the interrupts
    EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR, EDRV_EIMC_CLEAR_ALL);
    EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR, ~0);
    EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);

    // Set Device configuration
    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    dwReg &= ~(EDRV_CTRL_FD | EDRV_CTRL_ILOS | EDRV_CTRL_TFCE | EDRV_CTRL_RFCE);
    dwReg |= EDRV_CTRL_SLU;
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, dwReg);

    // Reset the phy
    //1. Acquire Phy Sw semaphore for PHY
    EdrvAcquireSwfwSync(EDRV_SWFW_PHY0_SM);
    //2. Set Phy reset bit in CTRL register
    dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    dwReg |= (EDRV_CTRL_PHY_RST);
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, dwReg);
    //3. Wait for 1 ms to complete the effect
    msleep(1);
    //4. Clear the bit
    dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    dwReg &= ~(EDRV_CTRL_PHY_RST);
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, dwReg);
    msleep(10);
    //5. Release semaphore
    EdrvReleaseSwfwSync(EDRV_SWFW_PHY0_SM);

    // Get Control from hardware
    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_CTRL_EXTN_REG);
    dwReg |= EDRV_CTRL_EXTN_DRV_LOAD;
    EDRV_REGDW_WRITE(EDRV_CTRL_EXTN_REG, dwReg);

    // Set the Queue Parameters
    EdrvInstance_l.m_TxMaxQueue = EDRV_MAX_TX_QUEUES;
    EdrvInstance_l.m_RxMaxQueue = EDRV_MAX_RX_QUEUES;
    EdrvInstance_l.m_NumQVectors = EDRV_MAX_QUEUE_VECTOR;

    //Initialise the SYSTIM timer with current system time
    EDRV_REGDW_WRITE(EDRV_TSAUXC, 0x0);
    sSysTime = ktime_to_timespec(ktime_get_real());
    EdrvWriteSystim(&sSysTime);

    // Clear the Statistic register
    dwReg = EDRV_REGDW_READ(EDRV_STAT_TPT);
    dwReg = EDRV_REGDW_READ(EDRV_STAT_TPR);
    dwReg = EDRV_REGDW_READ(EDRV_STAT_GPRC);
    dwReg = EDRV_REGDW_READ(EDRV_STAT_BPRC);
    dwReg = EDRV_REGDW_READ(EDRV_STAT_MPRC);

    EDRV_REGDW_WRITE(EDRV_TIPG_REG, EDRV_TIPG_DEF);

    // Setup Interrupt capability
    printk("Register MSI-X");
    iNumVectors = EdrvInstance_l.m_NumQVectors + 1;
    EdrvInstance_l.m_pMsixEntry = kcalloc(iNumVectors,
            sizeof(struct msix_entry), GFP_KERNEL);
    if (EdrvInstance_l.m_pMsixEntry)
    {
        for (iIndex = 0; iIndex < iNumVectors; iIndex++)
        {
            EdrvInstance_l.m_pMsixEntry[iIndex].entry = iIndex;
        }

        iResult = pci_enable_msix(pPciDev, EdrvInstance_l.m_pMsixEntry,
                iNumVectors);

        if (0 != iResult)
        {
            printk("...Failed\n");
            goto ExitFail;
        }

    }
    else
    {
        goto ExitFail;
    }
    printk("...Done\n");

    // Allocate Queue vectors
    for (iIndex = 0; iIndex < EdrvInstance_l.m_NumQVectors; iIndex++)
    {
        pVector = kzalloc(sizeof(tEdrvQVector), GFP_KERNEL);
        pVector->m_uiQueueIdx = iIndex;
        pVector->m_uiVector = EdrvInstance_l.m_pMsixEntry[iIndex + 1].vector;
        snprintf(pVector->m_strName, (sizeof(pVector->m_strName) - 1),
                "%s-TxRxQ-%u", DRIVER_NAME, pVector->m_uiQueueIdx);

        EdrvInstance_l.m_pQvector[iIndex] = pVector;
    }

    // Allocate Tx Buffer memory
    EdrvInstance_l.m_pbTxBuf = kzalloc(EDRV_TX_BUFFER_SIZE, GFP_KERNEL);
    if (NULL == EdrvInstance_l.m_pbTxBuf)
    {
        iResult = -ENOMEM;
        goto ExitFail;
    }
    for (iIndex = 0; iIndex < EdrvInstance_l.m_TxMaxQueue; iIndex++)
    {
        pQueue = kzalloc(sizeof(tEdrvQueue), GFP_KERNEL);
        if (NULL == pQueue)
        {
            goto ExitFail;
        }
        pQueue->m_iIndex = iIndex;
        EdrvInstance_l.m_pTxQueue[iIndex] = pQueue;
    }

    for (iIndex = 0; iIndex < EdrvInstance_l.m_TxMaxQueue; iIndex++)
    {
        iResult = EdrvInitTxQueue(EdrvInstance_l.m_pTxQueue[iIndex]);
        if (kEplSuccessful != iResult)
        {
            goto ExitFail;
        }
    }

    // check if user specified a MAC address
    if ((EdrvInstance_l.m_InitParam.m_abMyMacAddr[0] != 0)
            | (EdrvInstance_l.m_InitParam.m_abMyMacAddr[1] != 0)
            | (EdrvInstance_l.m_InitParam.m_abMyMacAddr[2] != 0)
            | (EdrvInstance_l.m_InitParam.m_abMyMacAddr[3] != 0)
            | (EdrvInstance_l.m_InitParam.m_abMyMacAddr[4] != 0)
            | (EdrvInstance_l.m_InitParam.m_abMyMacAddr[5] != 0))
    {
        // write specified MAC address to controller
        dwReg = 0;
        EDRV_REGDW_WRITE(EDRV_RAH(0), dwReg);          // disable Entry
        dwReg |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[0] << 0;
        dwReg |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[1] << 8;
        dwReg |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[2] << 16;
        dwReg |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[3] << 24;
        EDRV_REGDW_WRITE(EDRV_RAL(0), dwReg);
        dwReg = 0;
        dwReg |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[4] << 0;
        dwReg |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[5] << 8;
        dwReg |= EDRV_RAH_AV;
        EDRV_REGDW_WRITE(EDRV_RAH(0), dwReg);
    }
    else
    {          // read MAC address from controller
        dwReg = EDRV_REGDW_READ(EDRV_RAL(0));
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[0] = (dwReg >> 0) & 0xFF;
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[1] = (dwReg >> 8) & 0xFF;
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[2] = (dwReg >> 16) & 0xFF;
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[3] = (dwReg >> 24) & 0xFF;
        dwReg = EDRV_REGDW_READ(EDRV_RAH(0));
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[4] = (dwReg >> 0) & 0xFF;
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[5] = (dwReg >> 8) & 0xFF;
    }

    // initialize Multicast Table Array to 0
    for (iIndex = 0; iIndex < 128; iIndex++)
    {
        EDRV_REGDW_WRITE(EDRV_MTA(iIndex), 0);
    }

    // Alloc Rx Queue here
    for (iIndex = 0; iIndex < EdrvInstance_l.m_RxMaxQueue; iIndex++)
    {
        pQueue = kzalloc(sizeof(tEdrvQueue), GFP_KERNEL);
        if (NULL == pQueue)
        {
            goto ExitFail;
        }
        pQueue->m_iIndex = iIndex;
        EdrvInstance_l.m_pRxQueue[iIndex] = pQueue;
    }

    for (iIndex = 0; iIndex < EdrvInstance_l.m_RxMaxQueue; iIndex++)
    {
        iResult = EdrvInitRxQueue(EdrvInstance_l.m_pRxQueue[iIndex]);
        if (kEplSuccessful != iResult)
        {
            goto ExitFail;
        }
    }

    // Configure device for Qav mode
    EdrvInitQavMode();

    // Setup Tx Configuration

    EDRV_REGDW_WRITE(EDRV_TXDCTL(0), 0);          // Disable Q0 before proceeding
    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_TCTL_REG);
    dwReg &= ~(EDRV_TCTL_CLEAR_CT | EDRV_TCTL_RTLC);
    dwReg |= (EDRV_TCTL_PSP);
    EDRV_REGDW_WRITE(EDRV_TCTL_REG, dwReg);

    // Set the default collision threshold
    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_TCTL_EXT_REG);
    dwReg &= ~EDRV_TCTL_EXT_COLD_CLEAR;
    dwReg |= EDRV_TCTL_EXT_COLD;
    EDRV_REGDW_WRITE(EDRV_TCTL_EXT_REG, dwReg);

    // Enable the Tx
    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_TCTL_REG);
    dwReg |= EDRV_TCTL_EN;
    EDRV_REGDW_WRITE(EDRV_TCTL_REG, dwReg);

    for (iIndex = 0; iIndex < EdrvInstance_l.m_TxMaxQueue; iIndex++)
    {
        EdrvConfigureTxQueue(EdrvInstance_l.m_pTxQueue[iIndex]);
    }

    // Diasable 1st Rx Queue
    EDRV_REGDW_WRITE(EDRV_RXDCTL(0), 0);

    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_RCTL_REG);
    dwReg &= ~(3 << EDRV_RCTL_MO_SHIFT);
    dwReg &= ~(3 << EDRV_RCTL_BSIZE_OFFSET);
    dwReg &= ~EDRV_RCTL_LBM_CLEAR;
    dwReg |= (EDRV_RCTL_EN | EDRV_RCTL_BAM | EDRV_RCTL_SECRC | EDRV_RCTL_LPE);
    // Receive all packets
#ifdef PROMISCUOUS_MODE
    dwReg |= (EDRV_RCTL_UPE | EDRV_RCTL_MPE);
#endif

    // Enable Receive
    EDRV_REGDW_WRITE(EDRV_RCTL_REG, dwReg);

    // configure Rx queues
    for (iIndex = 0; iIndex < EdrvInstance_l.m_RxMaxQueue; iIndex++)
    {
        EdrvConfigureRxQueue(EdrvInstance_l.m_pRxQueue[iIndex]);
    }

    // Allocate Rx Buffers
    for (iIndex = 0; iIndex < EdrvInstance_l.m_RxMaxQueue; iIndex++)
    {
        iResult = EdrvAllocRxBuffer(EdrvInstance_l.m_pRxQueue[iIndex]);
        if (kEplSuccessful != iResult)
        {
            goto ExitFail;
        }
    }

    printk("Requesting Interrupt");
    //Request MSI-X
    iResult = EdrvRequestMsix();
    if (0 != iResult)
    {
        printk("... Failed\n");
        goto ExitFail;
    }
    printk("...Done\n");

    // enable interrupts
    EDRV_REGDW_WRITE(EDRV_INTR_MASK_SET_READ, (EDRV_INTR_ICR_TIME_SYNC ));

    dwReg = EDRV_REGDW_READ(EDRV_EXT_INTR_MASK_SET);
    dwReg |= (EDRV_EICS_TXRXQUEUE1 | EDRV_EICS_OTHER);
    EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_SET, dwReg);

    dwReg = EDRV_REGDW_READ(EDRV_INTR_EIAC);
    dwReg |= (EDRV_EICS_TXRXQUEUE1 | EDRV_EICS_OTHER);
    EDRV_REGDW_WRITE(EDRV_INTR_EIAC, dwReg);

    printk("%s waiting for link up...", __FUNCTION__);
    for (iIndex = EDRV_LINK_UP_TIMEOUT; iIndex > 0; iIndex -= 100)
    {
        if ((EDRV_REGDW_READ(EDRV_STATUS_REG) & EDRV_STATUS_LU))
        {
            printk("Link Up\n");
            dwReg = EDRV_REGDW_READ(EDRV_STATUS_REG);
            break;
        }
        msleep(100);
    }

    if (iIndex == 0)
    {
        printk("Link Down\n");
        iResult = -EIO;
        goto Exit;
    }

    goto Exit;

ExitFail:
    EdrvRemoveOne(pPciDev);
Exit:
    printk("%s finished with %d\n", __FUNCTION__, iResult);
    return iResult;

}

//------------------------------------------------------------------------------
/**
 \brief     EdrvRemoveOne

 shuts down one PCIe device

 \param     pPciDev     pointer to corresponding PCI device structure

 \return    void

 */
//------------------------------------------------------------------------------
static void EdrvRemoveOne(struct pci_dev *pPciDev)
{
    DWORD dwReg;
    INT iIndex;
    WORD wReg;
    INT iVector;
    tEdrvQVector *pVector;
    if (pPciDev != EdrvInstance_l.m_pPciDev)
    {
        BUG_ON(EdrvInstance_l.m_pPciDev != pPciDev);
        goto Exit;
    }

    EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR, EDRV_EIMC_CLEAR_ALL);
    EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR, ~0);
    EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);

    dwReg = EDRV_REGDW_READ(EDRV_INTR_READ_REG);

    // Stop the Timer
    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_TSAUXC);
    dwReg &= ~(EDRV_TSAUXC_EN_CLK0);
    EDRV_REGDW_WRITE(EDRV_TSAUXC, dwReg);

    if (EdrvInstance_l.m_pMsixEntry)
    {
        iVector = 0;
        free_irq(EdrvInstance_l.m_pMsixEntry[iVector].vector, pPciDev);
        iVector++;
        for (iIndex = 0; iIndex < EdrvInstance_l.m_NumQVectors; iIndex++)
        {
            free_irq(EdrvInstance_l.m_pMsixEntry[iVector].vector,
                    EdrvInstance_l.m_pQvector[iIndex]);
            iVector++;
        }
        pci_disable_msix(pPciDev);
        kfree(EdrvInstance_l.m_pMsixEntry);

    }

    dwReg = EDRV_REGDW_READ(EDRV_TXDCTL(0));
    dwReg |= EDRV_TXDCTL_SWFLSH;
    EDRV_REGDW_WRITE(EDRV_TXDCTL(0), dwReg);

    //Disable Rx
    dwReg = EDRV_REGDW_READ(EDRV_RXDCTL(0));
    dwReg |= EDRV_RXDCTL_SWFLUSH;
    EDRV_REGDW_WRITE(EDRV_RXDCTL(0), dwReg);

    EDRV_REGDW_READ(EDRV_STATUS_REG);
    msleep(10);

    dwReg = EDRV_REGDW_READ(EDRV_TXDCTL(0));
    dwReg &= ~EDRV_TXDCTL_SWFLSH;
    EDRV_REGDW_WRITE(EDRV_TXDCTL(0), dwReg);

    dwReg = EDRV_REGDW_READ(EDRV_RXDCTL(0));
    dwReg &= ~EDRV_RXDCTL_SWFLUSH;
    EDRV_REGDW_WRITE(EDRV_RXDCTL(0), dwReg);

    // Disable Tx
    dwReg = EDRV_REGDW_READ(EDRV_TCTL_REG);
    dwReg &= ~EDRV_TCTL_EN;
    EDRV_REGDW_WRITE(EDRV_TCTL_REG, dwReg);

    //Disable Rx
    dwReg = EDRV_REGDW_READ(EDRV_RCTL_REG);
    dwReg &= ~EDRV_RCTL_EN;
    EDRV_REGDW_WRITE(EDRV_RCTL_REG, dwReg);

    EdrvFreeTxBuffers();
    EdrvFreeRxBuffers();

    EdrvFreeTxQueues();
    EdrvFreeRxQueues();

    if (NULL != EdrvInstance_l.m_pbTxBuf)
    {
        kfree(EdrvInstance_l.m_pbTxBuf);
    }
    // Power down Phy
    wReg = EdrvMdicRead(PHY_I210_COPPER_SPEC);
    wReg |= PHY_I210_CS_POWER_DOWN;
    EdrvMdicWrite(PHY_I210_COPPER_SPEC, wReg);

    wReg = EdrvMdicRead(PHY_CONTROL_REG_OFFSET);
    wReg |= PHY_CONTROL_POWER_DOWN;
    EdrvMdicWrite(PHY_CONTROL_REG_OFFSET, wReg);
    msleep(1);

    // Release the control to Hardware
    dwReg = EDRV_REGDW_READ(EDRV_CTRL_EXTN_REG);
    dwReg &= ~EDRV_CTRL_EXTN_DRV_LOAD;
    EDRV_REGDW_WRITE(EDRV_CTRL_EXTN_REG, dwReg);

    // Free Queue vectors
    for (iIndex = 0; iIndex < EdrvInstance_l.m_NumQVectors; iIndex++)
    {
        pVector = EdrvInstance_l.m_pQvector[iIndex];
        kfree(pVector);
        EdrvInstance_l.m_pQvector[iIndex] = NULL;
    }
    EdrvInstance_l.m_NumQVectors = 0;

    // unmap controller's register space
    if (NULL != EdrvInstance_l.m_pIoAddr)
    {
        iounmap(EdrvInstance_l.m_pIoAddr);
        EdrvInstance_l.m_pIoAddr = NULL;
    }

    // Release Memory Regions
    pci_release_regions(pPciDev);

    // disable the PCI device
    pci_disable_device(pPciDev);

    EdrvInstance_l.m_pPciDev = NULL;

Exit:
    return;
}

//***************************************************************************//
//              T I M E R    H E L P E R    R O U T I N E S                  //
//***************************************************************************//

//------------------------------------------------------------------------------
/**
 \brief     EdrvGetMacClock()

 I210 specific routine which retrieves the current MAC time

 \param     pqwCurtime_p    pointer to the variable to receive current MAC time

 \return    Errorcode

 \retval    kEplSuccessful

 \retval    kEplNoResource

 */
//------------------------------------------------------------------------------
tEplKernel EdrvGetMacClock(QWORD *pqwCurtime_p)
{
    tEplKernel Ret = kEplSuccessful;
    DWORD timh, timl;
    DWORD dwReg;

    if (NULL == pqwCurtime_p)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    // Sample the current SYSTIM time in Auxiliary registers
    dwReg = EDRV_REGDW_READ(EDRV_TSAUXC);
    dwReg |= EDRV_TSAUXC_SAMP_AUTO;
    EDRV_REGDW_WRITE(EDRV_TSAUXC, dwReg);

    timl = EDRV_REGDW_READ(EDRV_AUXSTMPL0);
    timh = EDRV_REGDW_READ(EDRV_AUXSTMPH0);

    *pqwCurtime_p = (QWORD) timh * SEC_TO_NSEC + (QWORD) timl;

    Exit: return Ret;
}

//------------------------------------------------------------------------------
/**
 \brief      EdrvSetCyclicFrequency()

 Configure the cycle frequency in the I210 timer

 \param      dwOffset   Cyclelen

 \return     void

 */
//------------------------------------------------------------------------------
void EdrvSetCyclicFrequency(DWORD dwOffset)
{
    EDRV_REGDW_WRITE(EDRV_FREQOUT0, dwOffset);
}

//------------------------------------------------------------------------------
/**
 \brief     EdrvStartTimer()

 Start I210 timer

 \param     pTimerHdl_p Timer handle to start
 \param     dwOffset    Cycle time

 \return    Error code

 \retval    kEplSuccessful
 \retval    kEplTimerNoTimerCreated

 */
//------------------------------------------------------------------------------
tEplKernel EdrvStartTimer(tEplTimerHdl* pTimerHdl_p, DWORD dwOffset)
{
    tEplKernel EplRet = kEplSuccessful;
    DWORD dwReg;
    struct timespec ts;

    if (NULL == pTimerHdl_p)
    {
        printk("Error Timer\n");
        EplRet = kEplTimerNoTimerCreated;
        goto Exit;
    }

    EdrvInstance_l.m_TimerHdl = *pTimerHdl_p;

    dwReg = 0;
    dwReg |= (EDRV_TSSDP_TS_SDP0_SEL_CLK0 | EDRV_TSSDP_TS_SDP0_EN);
    EDRV_REGDW_WRITE(EDRV_TSSDP, dwReg);

    EdrvReadSystim(&ts);
    ts.tv_nsec = ts.tv_nsec + dwOffset;
    EDRV_REGDW_WRITE(EDRV_TRGTTIML0, ts.tv_nsec);
    EDRV_REGDW_WRITE(EDRV_TRGTTIMH0, ts.tv_sec);

    EdrvSetCyclicFrequency(dwOffset);

    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    dwReg |= SDP0_SET_DIR_OUT;
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, dwReg);

    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_TSAUXC);
    dwReg |= (EDRV_TSAUXC_EN_CLK0 | EDRV_TSAUXC_EN_TT0 | EDRV_TSAUXC_ST0);
    EDRV_REGDW_WRITE(EDRV_TSAUXC, dwReg);

    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_TSIM);
    dwReg |= (EDRV_TSIM_TT0);
    EDRV_REGDW_WRITE(EDRV_TSIM, dwReg);

Exit:
    return EplRet;
}
//------------------------------------------------------------------------------
/**
 \brief     EdrvStopTimer()

 Stop the timer

 \param     pTimerHdl_p  Handle to the timer to stop

 \return    Errorcode

 \retval    kEplSuccessful
 \retval    kEplTimerInvalidHandle

 */
//------------------------------------------------------------------------------
tEplKernel EdrvStopTimer(tEplTimerHdl* pTimerHdl_p)
{
    tEplKernel EplRet = kEplSuccessful;
    DWORD dwReg;

    if (*pTimerHdl_p != EdrvInstance_l.m_TimerHdl)
    {
        EplRet = kEplTimerInvalidHandle;
        goto Exit;
    }

    //printk("Stop Timer\n");
    EdrvInstance_l.m_TimerHdl = 0;
    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_TSIM);
    dwReg &= ~(EDRV_TSIM_TT0);
    EDRV_REGDW_WRITE(EDRV_TSIM, dwReg);

    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_TSAUXC);
    dwReg &= ~(EDRV_TSAUXC_EN_CLK0);
    EDRV_REGDW_WRITE(EDRV_TSAUXC, dwReg);

Exit:
    return EplRet;
}
//------------------------------------------------------------------------------
/**
 \brief     EdrvEnableTimer()

 Restart the timer

 \param     pTimerHdl_p     handle to the timer

 \return    Errorcode

 \retval    kEplSuccessful

 */
//------------------------------------------------------------------------------
tEplKernel EdrvEnableTimer(tEplTimerHdl* pTimerHdl_p)
{
    tEplKernel EplRet = kEplSuccessful;
    DWORD dwReg;
    struct timespec ts;

    EdrvReadSystim(&ts);

    if (*pTimerHdl_p != EdrvInstance_l.m_TimerHdl)
    {
        EplRet = kEplTimerInvalidHandle;
        goto Exit;
    }

    dwReg = 0;
    dwReg = EDRV_REGDW_READ(EDRV_TSAUXC);
    dwReg |= (EDRV_TSAUXC_EN_TT0);
    EDRV_REGDW_WRITE(EDRV_TSAUXC, dwReg);

Exit:
    return EplRet;
}
//------------------------------------------------------------------------------
/**
 \brief     EdrvRegisterHighResCallback()

 Register Timer callback

 \param     pfnHighResCb_p  pointer to the callback routine

 \return    Errorcode

 \retval    kEplSuccessful

 \retval    kEplTimerThreadError

 */
//------------------------------------------------------------------------------
tEplKernel EdrvRegisterHighResCallback(tEplHighResCallback pfnHighResCb_p)
{
    tEplKernel EplRet = kEplSuccessful;

    if (NULL == pfnHighResCb_p)
    {
        EplRet = kEplTimerThreadError;
        goto Exit;
    }

    EdrvInstance_l.m_HighResTimerCb = pfnHighResCb_p;

Exit:
    return EplRet;
}

