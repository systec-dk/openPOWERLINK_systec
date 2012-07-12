/*******************************************************************************

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Description:  functions to send generic Asnd frames

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Bernecker + Rainer Ges.m.b.H nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

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

*******************************************************************************/


//=========================================================================//
// Includes                                                                //
//=========================================================================//

#include <stddef.h>

#include "Epl.h"
#include "EplDll.h"
#include "user/EplGenericAsndu.h"
#include "user/EplDlluCal.h"
#include "user/EplEventu.h"

//=========================================================================//
// Type definitions                                                        //
//=========================================================================//

tEplKernel PUBLIC EplGenericAsnduCb(tEplFrameInfo *pFrameInfo_p);

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplGenericAsnduSetAsndForward
//
// Description: Enable or disable the forwarding of received Asnd frames
//              Asnd frames from the DLL to the application.
//
// Parameters:  bServiceId_p    The Asnd service ID which should be configured
//              fEnable_p       Flag which specifies whether the forwarding should
//                              be enabled (TRUE) or disabled (FALSE).
//
// Returns:     tEplKernel
//---------------------------------------------------------------------------
tEplKernel EplGenericAsnduSetAsndForward
(
    BYTE            bServiceId_p,
    BOOL            fEnable_p
)
{
    tEplKernel  Ret;

    if( fEnable_p == TRUE )
    {
        Ret = EplDlluCalRegAsndService( bServiceId_p,
                                        EplGenericAsnduCb,
                                        kEplDllAsndFilterLocal);
    }
    else
    {
        Ret = EplDlluCalRegAsndService( bServiceId_p,
                                        NULL,
                                        kEplDllAsndFilterNone);
    }

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplGenericAsnduSendFrame
//
// Description: Send a generic Asnd frame
//
// Parameters:  bDstNodeId_p        Node ID of destination node
//              pAsndFrame_p        Pointer to Asnd frame that should be sent
//              uiAsndSize_p        Size of Asnd frame (service ID + payload)
//
// Returns:     tEplKernel
//---------------------------------------------------------------------------
tEplKernel EplGenericAsnduSendFrame
(
    BYTE            bDstNodeId_p,
    tEplAsndFrame   *pAsndFrame_p,
    size_t          uiAsndSize_p
)
{
    tEplKernel      Ret;
    tEplFrameInfo   FrameInfo;
    BYTE            Buffer[EPL_C_DLL_MAX_ASYNC_MTU];

    // Calculate size of frame (Asnd data + header)
    FrameInfo.m_uiFrameSize = uiAsndSize_p + offsetof(tEplFrame, m_Data);

    // Check for correct input
    if
    (
        ( pAsndFrame_p              == NULL             )  ||
        ( FrameInfo.m_uiFrameSize   >= sizeof(Buffer)   )
    )
    {
        return  kEplReject;
    }

    // Calculate size of frame (Asnd data + header)
    FrameInfo.m_uiFrameSize = uiAsndSize_p + offsetof(tEplFrame, m_Data);
    FrameInfo.m_pFrame      = (tEplFrame *) Buffer;

    // Copy Asnd data
    EPL_MEMSET( FrameInfo.m_pFrame, 0x00, FrameInfo.m_uiFrameSize );
    EPL_MEMCPY( &FrameInfo.m_pFrame->m_Data.m_Asnd, pAsndFrame_p, uiAsndSize_p );

    // Fill in additional data (SrcNodeId is filled by DLL if it is set to 0)
    AmiSetByteToLe( &FrameInfo.m_pFrame->m_le_bMessageType, (BYTE) kEplMsgTypeAsnd  );
    AmiSetByteToLe( &FrameInfo.m_pFrame->m_le_bDstNodeId,   (BYTE) bDstNodeId_p     );
    AmiSetByteToLe( &FrameInfo.m_pFrame->m_le_bSrcNodeId,   (BYTE) 0                );

    // Request frame transmission
    Ret = EplDlluCalAsyncSend( &FrameInfo, kEplDllAsyncReqPrioGeneric);

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplGenericAsnduCb
//
// Description: Callback to handle received Asnd frames.
//              Frames will be forwarded to the API layer.
//
// Parameters:  pFrameInfo_p    Pointer to structure with information
//                              about the received frame
//
// Returns:     tEplKernel
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplGenericAsnduCb
(
    tEplFrameInfo *pFrameInfo_p
)
{
    tEplKernel      Ret = kEplSuccessful;
    tEplEvent       Event;
    unsigned int    uiAsndOffset;

    // Check for correct input
    uiAsndOffset    = offsetof(tEplFrame, m_Data.m_Asnd);

    if
    (
        ( pFrameInfo_p->m_uiFrameSize    <= uiAsndOffset + 1        ) ||
        ( pFrameInfo_p->m_uiFrameSize    >  EPL_C_DLL_MAX_ASYNC_MTU )
    )
    {
        return kEplReject;
    }

    // Set up event
    Event.m_EventSink   = kEplEventSinkApi;
    Event.m_EventType   = kEplEventTypeReceivedAsnd;
    Event.m_uiSize      = pFrameInfo_p->m_uiFrameSize;
    Event.m_pArg        = pFrameInfo_p->m_pFrame;

    Ret = EplEventuPost(&Event);

    return Ret;
}
