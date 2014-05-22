/****************************************************************************

  (c) SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for CFM module

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

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2009/12/14 d.k.:   start of the implementation

****************************************************************************/

#ifndef _EPLCFM_H_
#define _EPLCFM_H_

#include "EplInc.h"

#include "EplObd.h"

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
// typedef
//---------------------------------------------------------------------------

typedef struct
{
    unsigned int        m_uiNodeId;
    unsigned int        m_uiObjectIndex;
    unsigned int        m_uiObjectSubIndex;
    DWORD               m_dwSdoAbortCode;
    tEplKernel          m_EplError;
    DWORD               m_dwTotalNumberOfBytes;
    DWORD               m_dwBytesDownloaded;

} tEplCfmEventCnProgress;



//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

#ifdef __cplusplus
    extern "C" {
#endif

EPLDLLEXPORT tEplKernel PUBLIC EplCfmuCbObdAccess(tEplObdCbParam MEM* pParam_p);

#ifdef __cplusplus
	}
#endif

#endif // _EPLCFM_H_
