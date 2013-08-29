/****************************************************************************

  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      A-5142 Eggelsberg, B&R Strasse 1
      www.br-automation.com

  Project:      openPOWERLINK

  Description:  Virtual Ethernet Driver for NoOS - Test framework

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

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
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:

----------------------------------------------------------------------------*/

#include "VirtualEthernetNoOsTest.h"

#include "VirtualEthernetApi.h"

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_VETH)) != 0) && \
    defined(EPL_VETH_SEND_TEST)

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

#define EPL_MAC_ADDR_LEN    6

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct _tVEthTestInstance
{
    tEplVethTestCbRcv      m_pfnRcvCb;
} tVEthTestInstance;


//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

//Data from veth pinging 192.168.100.240
/*
 * Frame 22 (42 bytes on wire, 42 bytes captured)
Ethernet II, Src: CamilleB_56:78:9a (00:12:34:56:78:9a), Dst: Broadcast (ff:ff:ff:ff:ff:ff)
    Destination: Broadcast (ff:ff:ff:ff:ff:ff)
    Source: CamilleB_56:78:9a (00:12:34:56:78:9a)
    Type: ARP (0x0806)
Address Resolution Protocol (request)
    Hardware type: Ethernet (0x0001)
    Protocol type: IP (0x0800)
    Hardware size: 6
    Protocol size: 4
    Opcode: request (0x0001)
    Sender MAC address: CamilleB_56:78:9a (00:12:34:56:78:9a)
    Sender IP address: 192.168.100.1 (192.168.100.1) (correct last byte: Node ID)
    Target MAC address: 00:00:00_00:00:00 (00:00:00:00:00:00)
    Target IP address: 192.168.100.240 (192.168.100.240)
*/

static BYTE abNonEplData[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0x0, 0x12, 0x34, 0x56, 0x78, 0x9a,
        0x8, 0x6,
        0x0, 0x1,
        0x8, 0x0,
        0x6,
        0x4,
        0x0, 0x1,
        0x0, 0x12, 0x34, 0x56, 0x78, 0x9a,
        0xc0, 0xa8, 0x64, 0x01,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0xc0, 0xa8, 0x64, 0xF0 };


static tVEthTestInstance VEthTestInstance_g;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static void VEthTestAddressChanged(DWORD dwIpAddr_p, DWORD dwSubNetMask_p, WORD wMtu_p);

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:        VEthTestApiInitialize
//
// Description:     Initialize Test Framework
//
// Parameters:      pfnVethRcvCb_p  - Receive callback
//
// Returns:         kEplSuccessful - always succeed
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC VEthTestApiInitialize(tEplVethTestCbRcv pfnVethTestCbRcv_p)
{
tEplKernel  Ret = kEplSuccessful;
BYTE                   *pbEthMac;

    // make receive callback global
    VEthTestInstance_g.m_pfnRcvCb = pfnVethTestCbRcv_p;

    // register address changed callback function
    VEthApiRegNetAddressCb(VEthTestAddressChanged);

    // get MAC address
    pbEthMac = VEthApiGetEthMac();

    //set correct MAC address
    EPL_MEMCPY(abNonEplData+6, pbEthMac, sizeof(BYTE) * EPL_MAC_ADDR_LEN );
    EPL_MEMCPY(abNonEplData+22, pbEthMac, sizeof(BYTE) * EPL_MAC_ADDR_LEN );

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:        VEthTestApiSend
//
// Description:     Send a ARP test frame to the master which responds
//
// Parameters:      void
//
// Returns:         kEplInvalidParam - maximum MTU is reached
//                  kEplDllAsyncTxBuffer - internal buffer is full
//                  kEplInvalidOperation - Send not allowed in this state
//                  kEplSuccessful - message sent successfully
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC VEthTestApiSend(void)
{
tEplKernel  Ret = kEplSuccessful;

    //transmit test frame
    Ret = VEthApiXmit(abNonEplData,  sizeof(abNonEplData));

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:        VethTestApiProcess
//
// Description:     Process VirtualEthernet driver test frames
//
// Parameters:      void
//
// Returns:         void
//
// State:
//
//---------------------------------------------------------------------------
void PUBLIC VethTestApiProcess(void)
{
tEplKernel  Ret;
BYTE * pEthData;
WORD wEthDataSize;

    Ret = VEthApiCheckAndForwardRxFrame(&pEthData, &wEthDataSize);
    if(Ret == kEplSuccessful)
    {
        if(VEthTestInstance_g.m_pfnRcvCb != NULL)
        {
            // inform user
            VEthTestInstance_g.m_pfnRcvCb(&pEthData, &wEthDataSize);
        }

        // received new data -> free it!
        VEthApiReleaseRxFrame();
    }

}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

static void VEthTestAddressChanged(DWORD dwIpAddr_p, DWORD dwSubNetMask_p, WORD wMtu_p)
{
    // set correct IP address to test frame
    abNonEplData[31] = (BYTE)(0x000000FF & dwIpAddr_p);

}

#endif
