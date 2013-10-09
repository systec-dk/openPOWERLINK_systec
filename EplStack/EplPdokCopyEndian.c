/****************************************************************************

  (c) SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  copy the payload to the linked variables with endian conversion

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

    uiByteOffset = EPL_PDO_MAPPOBJECT_GET_BITOFFSET(pMappObject_p) >> 3;
    pbPayload_p += uiByteOffset;
    pVar = EPL_PDO_MAPPOBJECT_GET_VAR(pMappObject_p);

    switch (EPL_PDO_MAPPOBJECT_GET_TYPE(pMappObject_p))
    {
        //-----------------------------------------------
        // types without ami
        case kEplObdTypVString:
        case kEplObdTypOString:
        case kEplObdTypDomain:
        default:
        {
            // read value from object
            EPL_MEMCPY (pbPayload_p, pVar, EPL_PDO_MAPPOBJECT_GET_BYTESIZE(pMappObject_p));

            break;
        }

        //-----------------------------------------------
        // numerical type which needs ami-write
        // 8 bit or smaller values
        case kEplObdTypBool:
        case kEplObdTypInt8:
        case kEplObdTypUInt8:
        {
            AmiSetByteToLe(pbPayload_p, *((BYTE*)pVar));
            break;
        }

        // 16 bit values
        case kEplObdTypInt16:
        case kEplObdTypUInt16:
        {
            AmiSetWordToLe(pbPayload_p, *((WORD*)pVar));
            break;
        }

        // 24 bit values
        case kEplObdTypInt24:
        case kEplObdTypUInt24:
        {
            AmiSetDword24ToLe(pbPayload_p, *((DWORD*)pVar));
            break;
        }

        // 32 bit values
        case kEplObdTypInt32:
        case kEplObdTypUInt32:
        case kEplObdTypReal32:
        {
            AmiSetDwordToLe(pbPayload_p, *((DWORD*)pVar));
            break;
        }

        // 40 bit values
        case kEplObdTypInt40:
        case kEplObdTypUInt40:
        {
            AmiSetQword40ToLe(pbPayload_p, *((QWORD*)pVar));
            break;
        }

        // 48 bit values
        case kEplObdTypInt48:
        case kEplObdTypUInt48:
        {
            AmiSetQword48ToLe(pbPayload_p, *((QWORD*)pVar));
            break;
        }

        // 56 bit values
        case kEplObdTypInt56:
        case kEplObdTypUInt56:
        {
            AmiSetQword56ToLe(pbPayload_p, *((QWORD*)pVar));
            break;
        }

        // 64 bit values
        case kEplObdTypInt64:
        case kEplObdTypUInt64:
        case kEplObdTypReal64:
        {
            AmiSetQword64ToLe(pbPayload_p, *((QWORD*)pVar));
            break;
        }

        // time of day
        case kEplObdTypTimeOfDay:
        case kEplObdTypTimeDiff:
        {
            AmiSetTimeOfDay(pbPayload_p, ((tTimeOfDay*)pVar));
            break;
        }

    }

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

    uiByteOffset = EPL_PDO_MAPPOBJECT_GET_BITOFFSET(pMappObject_p) >> 3;
    pbPayload_p += uiByteOffset;
    pVar = EPL_PDO_MAPPOBJECT_GET_VAR(pMappObject_p);

    switch (EPL_PDO_MAPPOBJECT_GET_TYPE(pMappObject_p))
    {
        //-----------------------------------------------
        // types without ami
        case kEplObdTypVString:
        case kEplObdTypOString:
        case kEplObdTypDomain:
        default:
        {
            // read value from object
            EPL_MEMCPY (pVar, pbPayload_p, EPL_PDO_MAPPOBJECT_GET_BYTESIZE(pMappObject_p));

            break;
        }

        //-----------------------------------------------
        // numerical type which needs ami-write
        // 8 bit or smaller values
        case kEplObdTypBool:
        case kEplObdTypInt8:
        case kEplObdTypUInt8:
        {
            *((BYTE*)pVar) = AmiGetByteFromLe(pbPayload_p);
            break;
        }

        // 16 bit values
        case kEplObdTypInt16:
        case kEplObdTypUInt16:
        {
            *((WORD*)pVar) = AmiGetWordFromLe(pbPayload_p);
            break;
        }

        // 24 bit values
        case kEplObdTypInt24:
        case kEplObdTypUInt24:
        {
            *((DWORD*)pVar) = AmiGetDword24FromLe(pbPayload_p);
            break;
        }

        // 32 bit values
        case kEplObdTypInt32:
        case kEplObdTypUInt32:
        case kEplObdTypReal32:
        {
            *((DWORD*)pVar) = AmiGetDwordFromLe(pbPayload_p);
            break;
        }

        // 40 bit values
        case kEplObdTypInt40:
        case kEplObdTypUInt40:
        {
            *((QWORD*)pVar) = AmiGetQword40FromLe(pbPayload_p);
            break;
        }

        // 48 bit values
        case kEplObdTypInt48:
        case kEplObdTypUInt48:
        {
            *((QWORD*)pVar) = AmiGetQword48FromLe(pbPayload_p);
            break;
        }

        // 56 bit values
        case kEplObdTypInt56:
        case kEplObdTypUInt56:
        {
            *((QWORD*)pVar) = AmiGetQword56FromLe(pbPayload_p);
            break;
        }

        // 64 bit values
        case kEplObdTypInt64:
        case kEplObdTypUInt64:
        case kEplObdTypReal64:
        {
            *((QWORD*)pVar) = AmiGetQword64FromLe(pbPayload_p);
            break;
        }

        // time of day
        case kEplObdTypTimeOfDay:
        case kEplObdTypTimeDiff:
        {
            AmiGetTimeOfDay(pVar, ((tTimeOfDay*)pbPayload_p));
            break;
        }

    }

    return Ret;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

#endif // #if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)

// EOF

