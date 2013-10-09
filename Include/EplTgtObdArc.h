/****************************************************************************

  (c) SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  target specific functions for OD store/restore

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

  2013/01/08 d.k.:   start of the implementation

****************************************************************************/

#ifndef _EPLTGTOBDARC_H_
#define _EPLTGTOBDARC_H_


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

// storage read access parameters for objects 0x1010 and 0x1011 (see EPSG 301)
#define EPL_OBD_STORE_UNSUPPORTED   0x00000000L
#define EPL_OBD_STORE_ON_COMMAND    0x00000001L
#define EPL_OBD_STORE_AUTONOMOUSLY  0x00000002L


//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//
// Function:     EplTgtObdArcInit()
//
// Description:  initializes functionality for STORE, RESTORE and LOAD
//               for OD values in other memory media, initializes
//               interface to non-volatile memory
//
// Parameters:   void
//
// Returns:      tEplKernel             = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcInit (void);


//---------------------------------------------------------------------------
//
// Function:     EplTgtObdArcShutdown()
//
// Description:  disable interface to non-volatile memory
//
// Parameters:   void
//
// Returns:      tEplKernel             = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcShutdown (void);


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcCreate()
//
// Description: Function creates an archive for the selected OD part. An
//              existing archive is set invalid.
//
// Parameters:  CurrentOdPart_p         = selected OD-part
//
// Returns:     tEplKernel              = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcCreate (
    tEplObdPart CurrentOdPart_p);


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcDelete()
//
// Description: Function sets an archive invalid.
//
// Parameters:  CurrentOdPart_p         = selected OD-part
//
// Returns:     tEplKernel              = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcDelete (
    tEplObdPart CurrentOdPart_p);


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcOpen()
//
// Description: Function opens an existing archive for reading.
//
// Parameters:  CurrentOdPart_p         = selected OD-part
//
// Returns:     tEplKernel              = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcOpen (
    tEplObdPart CurrentOdPart_p);


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcClose()
//
// Description: Function closes an archive for the selected OD part by
//              setting a valid signature.
//
// Parameters:  CurrentOdPart_p         = selected OD-part
//
// Returns:     tEplKernel              = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcClose (
    tEplObdPart CurrentOdPart_p);


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcStore()
//
// Description: Function saves the parameter of the select part into
//              non-volatile memory.
//
// Parameters:  CurrentOdPart_p         = selected OD-part
//              pbData                  = pointer to source data
//              dwSize_p                = number of bytes
//
// Returns:     tEplKernel              = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcStore (
    tEplObdPart CurrentOdPart_p, BYTE GENERIC *pbData, DWORD dwSize_p);


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcRestore()
//
// Description: Function reads the parameter from memory and stores the
//              value into the selected OD part.
//
// Parameters:  CurrentOdPart_p         = selected OD-part
//              pbData                  = pointer to destination data
//              dwSize_p                = number of bytes
//
// Returns:     tEplKernel              = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcRestore (
    tEplObdPart CurrentOdPart_p, BYTE GENERIC *pbData, DWORD dwSize_p);


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcGetCapabilities()
//
// Description: The function returns the storage capabilities corresponding
//              to the specified object index and sub-index.
//              Additionally it returns the OD part corresponding to the
//              specified sub-index.
//
// Parameters:  uiIndex_p           = [IN] identifies command (0x1010 = save, 0x1011 = restore)
//              uiSubIndex_p        = [IN] identifies OD part
//              pOdPart_p           = [OUT] pointer to OD part
//              pdwCap_p            = [OUT] pointer to capabilities bit-field
//                  EPL_OBD_STORE_UNSUPPORTED  = 0x00000000 - Device does not save parameters.
//                  EPL_OBD_STORE_ON_COMMAND   = 0x00000001 - Device saves parameters on command.
//                  EPL_OBD_STORE_AUTONOMOUSLY = 0x00000002 - Device saves parameters autonomously.
//
// Returns:     tEplKernel              = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcGetCapabilities (
    unsigned int uiIndex_p, unsigned int uiSubIndex_p,
    tEplObdPart* pOdPart_p, DWORD* pdwCap_p);


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcCheckValid()
//
// Description: Function checks up if signature of selected OD
//              part is valid. EplTgtObdArcOpen() has to be called before.
//
// Parameters:  CurrentOdPart_p         = selected OD-part
//
// Returns:     FALSE  -> signature is invalid
//              TRUE   -> signature is valid
//
//---------------------------------------------------------------------------

BOOL PUBLIC EplTgtObdArcCheckValid (
    tEplObdPart CurrentOdPart_p);


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcSetBackupPath()
//
// Description: Function set the file path for backup archive.
//
// Parameters:  pszBackupPath_p        = selected OD-part
//
// Returns:     tEplKernel -> error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcSetBackupPath (
    const char* pszBackupPath_p);


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcSetSignature()
//
// Description: Function sets the OD signature for backup archive.
//
// Parameters:  CurrentOdPart_p         = selected OD-part
//              dwSignature_p           = OD signature
//
// Returns:     tEplKernel  -> error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcSetSignature (
    tEplObdPart CurrentOdPart_p, DWORD dwSignature_p);


#endif  // #ifndef _EPLTGTOBDARC_H_

