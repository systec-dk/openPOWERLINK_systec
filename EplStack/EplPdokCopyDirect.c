/****************************************************************************

  (c) SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  copy the payload to the linked variables without endian conversion

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

****************************************************************************/

#include "kernel/EplPdokCal.h"
#include "kernel/EplPdok.h"
#include "EplObd.h"

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)


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
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static const WORD aSizeList[] = { 0,                    // not used            0x00
                                  1,                    // kEplObdTypBool      0x01
                                  1,                    // kEplObdTypInt8      0x02
                                  2,                    // kEplObdTypInt16     0x03
                                  4,                    // kEplObdTypInt32     0x04
                                  1,                    // kEplObdTypUInt8     0x05
                                  2,                    // kEplObdTypUInt16    0x06
                                  4,                    // kEplObdTypUInt32    0x07
                                  4,                    // kEplObdTypReal32    0x08
                                  0,                    // kEplObdTypVString   0x09
                                  0,                    // kEplObdTypOString   0x0A
                                  0,                    // not used            0x0B
                                  sizeof(tTimeOfDay),   // kEplObdTypTimeOfDay 0x0C
                                  sizeof(tTimeOfDay),   // kEplObdTypTimeDiff  0x0D
                                  0,                    // not used            0x0E
                                  0,                    // kEplObdTypDomain    0x0F
                                  3,                    // kEplObdTypInt24     0x10
                                  8,                    // kEplObdTypReal64    0x11
                                  5,                    // kEplObdTypInt40     0x12
                                  6,                    // kEplObdTypInt48     0x13
                                  7,                    // kEplObdTypInt56     0x14
                                  8,                    // kEplObdTypInt64     0x15
                                  3,                    // kEplObdTypUInt24    0x16
                                  0,                    // not used            0x17
                                  5,                    // kEplObdTypUInt40    0x18
                                  6,                    // kEplObdTypUInt48    0x19
                                  7,                    // kEplObdTypUInt56    0x1A
                                  8,                    // kEplObdTypUInt64    0x1B
            };

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplPdok                                             */
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


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplPdokCopyVarToPdo
//
// Description: This function copies a variable specified by the mapping object
//              to the PDO payload.
//
// Parameters:  pbPayload_p             = pointer to PDO payload in destination frame
//              pMappObject_p           = pointer to mapping object
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EplPdokCopyVarToPdo(BYTE* pbPayload_p, tEplPdoMappObject* pMappObject_p)
{
tEplKernel      Ret = kEplSuccessful;
unsigned int    uiByteOffset;
void*           pVar;
WORD            i = 0;
WORD            wSize;

    uiByteOffset = EPL_PDO_MAPPOBJECT_GET_BITOFFSET(pMappObject_p) >> 3;
    pbPayload_p += uiByteOffset;
    pVar = EPL_PDO_MAPPOBJECT_GET_VAR(pMappObject_p);

    if(EPL_PDO_MAPPOBJECT_GET_TYPE(pMappObject_p) > EPL_PDO_COMMUNICATION_PROFILE_START)
    {
        wSize = EPL_PDO_MAPPOBJECT_GET_BYTESIZE(pMappObject_p);
    }
    else
    {
        wSize = aSizeList[EPL_PDO_MAPPOBJECT_GET_TYPE(pMappObject_p)];
    }

    do
    {
        *((BYTE*)pbPayload_p++) = *((BYTE*)pVar++); // copy bytewise
        i++;
    }
    while ( i < wSize);

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdokCopyVarFromPdo
//
// Description: This function copies a variable specified by the mapping object
//              from the PDO payload.
//
// Parameters:  pbPayload_p             = pointer to PDO payload in destination frame
//              pMappObject_p           = pointer to mapping object
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EplPdokCopyVarFromPdo(BYTE* pbPayload_p, tEplPdoMappObject* pMappObject_p)
{
tEplKernel      Ret = kEplSuccessful;
unsigned int    uiByteOffset;
void*           pVar;
WORD            i = 0;
WORD            wSize;

    uiByteOffset = EPL_PDO_MAPPOBJECT_GET_BITOFFSET(pMappObject_p) >> 3;
    pbPayload_p += uiByteOffset;
    pVar = EPL_PDO_MAPPOBJECT_GET_VAR(pMappObject_p);

    if(EPL_PDO_MAPPOBJECT_GET_TYPE(pMappObject_p) > EPL_PDO_COMMUNICATION_PROFILE_START)
    {
        wSize = EPL_PDO_MAPPOBJECT_GET_BYTESIZE(pMappObject_p);
    }
    else
    {
        wSize = aSizeList[EPL_PDO_MAPPOBJECT_GET_TYPE(pMappObject_p)];
    }

    do
    {
        *((BYTE*)pVar++) = *((BYTE*)pbPayload_p++); // copy bytewise
        i++;
    }
    while ( i < wSize);

    return Ret;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

#endif // #if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)

// EOF

