/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  Target specific module for storing and reloading OD entries
                OD data will be stored to an file using POSIX functions.
                The file format includes the target signature and OD signature
                to guarantee that the file still matches to the OD structure.
                The last two bytes of the file archive includes the CRC over
                all bytes within the file (in big endian format).

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
                Microsoft VC7

  -------------------------------------------------------------------------

  Revision History:

  2013/01/08 d.k.:   start of the implementation

****************************************************************************/

#ifdef _WIN32
    #include <windows.h>
    #pragma warning(disable:4996)
#endif

#include "EplInc.h"
#include "EplObd.h"
#include "EplCrc.h"
#include "EplTgtObdArc.h"

#include <sys/stat.h>
#include <assert.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#if (TARGET_SYSTEM == _WIN32_)

    #include <io.h>
    #include <sys/types.h>
    #include <sys/utime.h>
    #include <sys/timeb.h>
    #include <time.h>
    #include <direct.h>
    #include <string.h>

#elif (TARGET_SYSTEM == _LINUX_)

    #include <sys/io.h>
    #include <unistd.h>
    #include <sys/vfs.h>
    #include <sys/types.h>
    #include <sys/timeb.h>
    #include <utime.h>
    #include <limits.h>

#elif (DEV_SYSTEM == _DEV_PAR_BECK1X3_)

    #include <io.h>
    #include <string.h>

#endif

#if (TARGET_SYSTEM == _WIN32_)

    #define flush  _commit

#elif (TARGET_SYSTEM == _LINUX_)

    #define O_BINARY 0
    #define _MAX_PATH PATH_MAX
    #define flush  fsync

#elif (DEV_SYSTEM == _DEV_PAR_BECK1X3_)

    #define flush(h)                    // #define flush() to nothing

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

#ifndef EPLTGTOBDARC_FILENAME_PREFIX
    #define EPLTGTOBDARC_FILENAME_PREFIX        "EplOd"
#endif

#ifndef EPLTGTOBDARC_FILENAME_EXTENSION
    #define EPLTGTOBDARC_FILENAME_EXTENSION     ".bin"
#endif


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel EplTgtObdArcGetFilePath( tEplObdPart CurrentOdPart_p,
    char* pszBkupPath_p, char* pszFilePathName_p);



/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <Store/Load>                                        */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
//  Description:
//
//  File XXX%d_Com.bin:
//          +----------------------+
//  0x0000  | target signature     | (4 byte)
//          +----------------------+
//  0x0004  | OD signature of part | (4 byte)
//          |  0x1000 - 0x1FFF     |
//          +----------------------+
//  0x0008  | all OD data of part  | (n byte)
//          |  0x1000 - 0x1FFF     |
//          +----------------------+
//  0xNNNN  | OD data CRC          | (2 byte)
//          +----------------------+
//
//  File XXX%d_Man.bin:
//          +----------------------+
//  0x0000  | target signature     | (4 byte)
//          +----------------------+
//  0x0004  | OD signature of part | (4 byte)
//          |  0x2000 - 0x5FFF     |
//          +----------------------+
//  0x0008  | all OD data of part  | (n byte)
//          |  0x2000 - 0x5FFF     |
//          +----------------------+
//  0xNNNN  | OD data CRC          | (2 byte)
//          +----------------------+
//
//  File XXX%d_Dev.bin:
//          +----------------------+
//  0x0000  | target signature     | (4 byte)
//          +----------------------+
//  0x0004  | OD signature of part | (4 byte)
//          |  0x6000 - 0x9FFF     |
//          +----------------------+
//  0x0008  | all OD data of part  | (n byte)
//          |  0x6000 - 0x9FFF     |
//          +----------------------+
//  0xNNNN  | OD data CRC          | (2 byte)
//          +----------------------+
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

typedef struct
{
    int         m_hBkupArchiveFile;
    char*       m_pszBackupPath;
    BOOL        m_fOpenForWrite;
    DWORD       m_dwOdSigPartDev;
    DWORD       m_dwOdSigPartGen;
    DWORD       m_dwOdSigPartMan;
    WORD        m_wOdDataCrc;
}
tEplTgtObdArcInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static DWORD dwEplTgtObdArcTgtSignature_l;
static tEplTgtObdArcInstance aEplTgtObdArcInstance_l[1];


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
// Function:    EplTgtObdArcInit()
//
// Description: initializes functionality for STORE, RESTORE and LOAD
//              for OD values in other memory mediums, initializes
//              interface to non-volatile memory if it not done by
//              operating systems
//
// Parameters:  void  = (instance handle)
//
// Returns:     tEplKernel             = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcInit (void)
{
tEplKernel RetCode = kEplSuccessful;
tEplTgtObdArcInstance* pInstEntry;

    dwEplTgtObdArcTgtSignature_l = 0x444F4C50;  // signature PLOD

    // get current instance entry and initialize all members
    pInstEntry = &aEplTgtObdArcInstance_l[0];
    EPL_MEMSET(pInstEntry, 0, sizeof (*pInstEntry));
    pInstEntry->m_hBkupArchiveFile = (-1);

    return RetCode;

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcShutdown()
//
// Description: disable interface to non-volatile memory
//
// Parameters:  void  = (instance handle)
//
// Returns:     tEplKernel             = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcShutdown (void)
{
tEplKernel RetCode;

    RetCode = kEplSuccessful;

    return (RetCode);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcCreate()
//
// Description: Function creates an archiv for the selected OD part. In
//              existence archiv is set unvalid.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     tEplKernel             = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcCreate ( tEplObdPart CurrentOdPart_p)
{
tEplKernel RetCode = kEplStoreHwError;
int        iWrCnt;
char       szPath[_MAX_PATH];
DWORD      dwOdSignature;
tEplTgtObdArcInstance* pInstEntry;

    // get current instance entry
    pInstEntry = &aEplTgtObdArcInstance_l[0];

    // get the file path for current OD part and instance
    RetCode = EplTgtObdArcGetFilePath(
        CurrentOdPart_p, pInstEntry->m_pszBackupPath, &szPath[0]);
    if (RetCode != kEplSuccessful)
    {
        goto Exit;
    }

    // is the file already opened?
    if (pInstEntry->m_hBkupArchiveFile >= 0)
    {
        RetCode = kEplStoreInvalidState;
        goto Exit;
    }

    // check OD part for correct OD signature
    switch (CurrentOdPart_p)
    {
        case kEplObdPartGen: dwOdSignature = pInstEntry->m_dwOdSigPartGen; break;
        case kEplObdPartMan: dwOdSignature = pInstEntry->m_dwOdSigPartMan; break;
        case kEplObdPartDev: dwOdSignature = pInstEntry->m_dwOdSigPartDev; break;

        default:
            RetCode = kEplApiInvalidParam;
            goto Exit;
    }

    // open file for writing
    pInstEntry->m_fOpenForWrite = TRUE;
    pInstEntry->m_hBkupArchiveFile = open(szPath, O_CREAT | O_TRUNC | O_WRONLY | O_BINARY, 0666);
    if (pInstEntry->m_hBkupArchiveFile < 0)
    {
        pInstEntry->m_hBkupArchiveFile = (-1);
        goto Exit;
    }

    // write target signature and calculate CRC for it
    pInstEntry->m_wOdDataCrc = CALCULATE_CRC16(0, (BYTE GENERIC*) &dwEplTgtObdArcTgtSignature_l, sizeof (dwEplTgtObdArcTgtSignature_l));
    iWrCnt = write(pInstEntry->m_hBkupArchiveFile, (BYTE FAR*) &dwEplTgtObdArcTgtSignature_l, sizeof (dwEplTgtObdArcTgtSignature_l));
    if (iWrCnt != (int) sizeof (dwEplTgtObdArcTgtSignature_l))
    {
        goto Exit;
    }

    // write OD signature and calculate CRC for it
    pInstEntry->m_wOdDataCrc = CALCULATE_CRC16(pInstEntry->m_wOdDataCrc, (BYTE GENERIC*) &dwOdSignature, sizeof (dwOdSignature));
    iWrCnt = write(pInstEntry->m_hBkupArchiveFile, (BYTE FAR*) &dwOdSignature, sizeof (dwOdSignature));
    if (iWrCnt != (int) sizeof (dwOdSignature))
    {
        goto Exit;
    }

    RetCode = kEplSuccessful;
Exit:
    return (RetCode);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcDelete()
//
// Description: Function sets an archiv unvalid.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     tEplKernel             = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcDelete ( tEplObdPart CurrentOdPart_p)
{
tEplKernel RetCode = kEplStoreHwError;
char       szPath[_MAX_PATH];
tEplTgtObdArcInstance* pInstEntry;

    // get current instance entry
    pInstEntry = &aEplTgtObdArcInstance_l[0];

    // get the file path for current OD part and instance
    RetCode = EplTgtObdArcGetFilePath(
        CurrentOdPart_p, pInstEntry->m_pszBackupPath, &szPath[0]);
    if (RetCode != kEplSuccessful)
    {
        goto Exit;
    }

    // delete the backup archive file
    if (unlink(szPath) == (-1))
    {
        RetCode = kEplStoreHwError;
        goto Exit;
    }

    RetCode = kEplSuccessful;
Exit:
    return (RetCode);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcOpen()
//
// Description: Function opens an in existence archiv.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     tEplKernel             = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcOpen (
    tEplObdPart CurrentOdPart_p)
{
BYTE  RetCode = kEplStoreHwError;
char  szPath[_MAX_PATH];
tEplTgtObdArcInstance* pInstEntry;

    // get current instance entry
    pInstEntry = &aEplTgtObdArcInstance_l[0];

    // get the file path for current OD part and instance
    RetCode = EplTgtObdArcGetFilePath(
        CurrentOdPart_p, pInstEntry->m_pszBackupPath, &szPath[0]);
    if (RetCode != kEplSuccessful)
    {
        goto Exit;
    }

    RetCode = kEplStoreHwError;

    // is the file already opened?
    if (pInstEntry->m_hBkupArchiveFile >= 0)
    {
        RetCode = kEplStoreInvalidState;
        goto Exit;
    }

    // open backup archive file for read
    pInstEntry->m_fOpenForWrite = FALSE;
    pInstEntry->m_hBkupArchiveFile = open(szPath, O_RDONLY | O_BINARY, 0666);

    if (pInstEntry->m_hBkupArchiveFile < 0)
    {
        // backup archive file could not be opend
        goto Exit;
    }

    // set file position to the begin of the file
    lseek(pInstEntry->m_hBkupArchiveFile, 0, SEEK_SET);

    RetCode = kEplSuccessful;
Exit:
    return (RetCode);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcClose()
//
// Description: Function closes an archiv for the selected OD part by
//              setting a valid signature.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     tEplKernel             = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcClose ( tEplObdPart CurrentOdPart_p)
{
tEplKernel RetCode = kEplSuccessful;
int nRes;
BYTE bData;
tEplTgtObdArcInstance* pInstEntry;

    UNUSED_PARAMETER(CurrentOdPart_p);

    // get current instance entry
    pInstEntry = &aEplTgtObdArcInstance_l[0];

    // is the file not opened?
    if (pInstEntry->m_hBkupArchiveFile < 0)
    {
        RetCode = kEplStoreInvalidState;
        goto Exit;
    }

    // if file was opened for write we have to add the OD data CRC at the end of the file
    if (pInstEntry->m_fOpenForWrite != FALSE)
    {
        // write CRC16 to end of the file (in big endian format)
        bData  = (BYTE) ((pInstEntry->m_wOdDataCrc >> 8) & 0xFF);
        nRes   = write(pInstEntry->m_hBkupArchiveFile, (BYTE FAR*) &bData, sizeof (bData));
        bData  = (BYTE) ((pInstEntry->m_wOdDataCrc >> 0) & 0xFF);
        nRes  += write(pInstEntry->m_hBkupArchiveFile, (BYTE FAR*) &bData, sizeof (bData));
        if (nRes != (int) (sizeof (bData) * 2))
        {   // save error code and close the file
            RetCode = kEplStoreHwError;
        }

        // sync file to disc
        flush(pInstEntry->m_hBkupArchiveFile);
    }

    // close archive file and set file handle invalid
    close(pInstEntry->m_hBkupArchiveFile);
    pInstEntry->m_hBkupArchiveFile = (-1);

Exit:
    return (RetCode);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcStore()
//
// Description: Function saves the parameter of the select part into
//              memory.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//              pointer to source data
//              number of bytes
//
// Returns:     tEplKernel             = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcStore ( tEplObdPart CurrentOdPart_p,
    BYTE GENERIC *pbData, DWORD dwSize_p)
{
tEplKernel RetCode = kEplStoreHwError;
int       iWrCnt;
tEplTgtObdArcInstance* pInstEntry;

    UNUSED_PARAMETER(CurrentOdPart_p);

    // get current instance entry
    pInstEntry = &aEplTgtObdArcInstance_l[0];

    // is the file not opened?
    if (pInstEntry->m_hBkupArchiveFile < 0)
    {
        RetCode = kEplStoreInvalidState;
        goto Exit;
    }

    // write current OD data to the file and calculate the CRC for it
    pInstEntry->m_wOdDataCrc = CALCULATE_CRC16(pInstEntry->m_wOdDataCrc, pbData, dwSize_p);
    iWrCnt = write(pInstEntry->m_hBkupArchiveFile, (BYTE FAR*) pbData, dwSize_p);
    if (iWrCnt != (int) dwSize_p)
    {
        goto Exit;
    }

    RetCode = kEplSuccessful;
Exit:
    return (RetCode);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcRestore()
//
// Description: Function reads the parameter from memory and stores the
//              value into the selected OD part.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//              pointer to destination data
//              number of bytes
//
// Returns:     tEplKernel             = error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcRestore ( tEplObdPart CurrentOdPart_p,
    BYTE GENERIC *pbData, DWORD dwSize_p)
{
tEplKernel RetCode = kEplStoreHwError;
int       iRdCnt;
tEplTgtObdArcInstance* pInstEntry;

    UNUSED_PARAMETER(CurrentOdPart_p);

    // get current instance entry
    pInstEntry = &aEplTgtObdArcInstance_l[0];

    // is the file not opened?
    if (pInstEntry->m_hBkupArchiveFile < 0)
    {
        RetCode = kEplStoreInvalidState;
        goto Exit;
    }

    // read OD data from current file position
    iRdCnt = read(pInstEntry->m_hBkupArchiveFile, (BYTE FAR*) pbData, dwSize_p);
    if (iRdCnt != (int) dwSize_p)
    {
        goto Exit;
    }

    RetCode = kEplSuccessful;
Exit:
    return (RetCode);

}


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
    tEplObdPart* pOdPart_p, DWORD* pdwCap_p)
{
tEplKernel RetCode = kEplSuccessful;

    UNUSED_PARAMETER(uiIndex_p);

    if ((pOdPart_p == NULL)
        || (pdwCap_p == NULL))
    {
        RetCode = kEplApiInvalidParam;
        goto Exit;
    }

    switch (uiSubIndex_p)
    {
        // Device supports save/restore of index ranges
        // 0x1000-0x1FFF, 0x2000-0x5FFF, 0x6000-0x9FFF
        case 1:
        {
            *pdwCap_p = EPL_OBD_STORE_ON_COMMAND;
            *pOdPart_p = kEplObdPartAll;
            break;
        }

        // Device supports save/restore of index range
        // 0x1000-0x1FFF
        case 2:
        {
            *pdwCap_p = EPL_OBD_STORE_ON_COMMAND;
            *pOdPart_p = kEplObdPartGen;
            break;
        }

        // Device supports not save/restore of index
        // 0x6000-0x9FFF
        case 3:
        {
            *pdwCap_p = EPL_OBD_STORE_ON_COMMAND;
            *pOdPart_p = kEplObdPartDev;
            break;
        }

        // Device supports save/restore of index
        // 0x2000-0x5FFF
        case 4:
        {
            *pdwCap_p = EPL_OBD_STORE_ON_COMMAND;
            *pOdPart_p = kEplObdPartMan;
            break;
        }

        default:
        {
            *pdwCap_p = 0;
            break;
        }
    }

Exit:
    return (RetCode);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcCheckValid()
//
// Description: Function checks up if signature of selected OD
//              part is valid.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     FALSE  -> signature is unvalid
//              TRUE   -> signature is valid
//
//---------------------------------------------------------------------------

BOOL PUBLIC EplTgtObdArcCheckValid ( tEplObdPart CurrentOdPart_p)
{
BOOL  fRetCode = FALSE;
int   iRdCnt;
DWORD dwRdTgtSignature;
DWORD dwRdOdSignature;
DWORD dwCurrOdSignature;
BYTE  abTempBuffer[8];
int   nCount;
WORD  wDataCrc;
tEplTgtObdArcInstance* pInstEntry;

    // get current instance entry
    pInstEntry = &aEplTgtObdArcInstance_l[0];

    // is the file not opened?
    if (pInstEntry->m_hBkupArchiveFile < 0)
    {
        goto Exit;
    }

    // check OD part for correct OD signature
    switch (CurrentOdPart_p)
    {
        case kEplObdPartGen: dwCurrOdSignature = pInstEntry->m_dwOdSigPartGen; break;
        case kEplObdPartMan: dwCurrOdSignature = pInstEntry->m_dwOdSigPartMan; break;
        case kEplObdPartDev: dwCurrOdSignature = pInstEntry->m_dwOdSigPartDev; break;

        default:
            goto Exit;
    }

    // read target signature and calculate the CRC for it
    iRdCnt = read(pInstEntry->m_hBkupArchiveFile, &dwRdTgtSignature, sizeof (dwRdTgtSignature));
    wDataCrc = CALCULATE_CRC16(0, (BYTE GENERIC*) &dwRdTgtSignature, sizeof (dwRdTgtSignature));
    if (iRdCnt != (int) sizeof (dwRdTgtSignature))
    {
        goto Exit;
    }

    // read OD signature and calculate the CRC for it
    iRdCnt = read(pInstEntry->m_hBkupArchiveFile, &dwRdOdSignature, sizeof (dwRdOdSignature));
    wDataCrc = CALCULATE_CRC16(wDataCrc, (BYTE GENERIC*) &dwRdOdSignature, sizeof (dwRdOdSignature));
    if (iRdCnt != (int) sizeof (dwRdOdSignature))
    {
        goto Exit;
    }

    // check if both target signature and OD signature are correct
    if ((dwRdTgtSignature != dwEplTgtObdArcTgtSignature_l) ||
        (dwRdOdSignature  != dwCurrOdSignature) )
    {
        goto Exit;
    }

    // calculate OD data CRC over all data bytes
    for (;;)
    {
        nCount = read(pInstEntry->m_hBkupArchiveFile, &abTempBuffer[0], sizeof (abTempBuffer));
        if (nCount > 0)
        {
            wDataCrc = CALCULATE_CRC16 (wDataCrc, (BYTE GENERIC*) &abTempBuffer[0], nCount);
        }
        else
        {
            break;
        }
    }

    // check OD data CRC (always zero because CRC has to be set at the end of the file in big endian format)
    if (wDataCrc != 0)
    {
        goto Exit;
    }

    // set file position back to OD data
    lseek(pInstEntry->m_hBkupArchiveFile, sizeof (dwRdTgtSignature) + sizeof (dwRdOdSignature), SEEK_SET);

    fRetCode = TRUE;
Exit:
    return (fRetCode);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcSetBackupPath()
//
// Description: Function set the file path for backup archive.
//
// Parameters:  void  = (instance handle)
//              pszBackupPath_p        = selected OD-part
//
// Returns:     tEplKernel -> error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcSetBackupPath (
    const char* pszBackupPath_p)
{
tEplKernel RetCode = kEplStoreHwError;
tEplTgtObdArcInstance* pInstEntry;

    // check pointer to backup path string
    if (pszBackupPath_p == NULL)
    {
        RetCode = kEplApiInvalidParam;
        goto Exit;
    }

    // get current instance entry
    pInstEntry = &aEplTgtObdArcInstance_l[0];

    // save pointer to backup path
    pInstEntry->m_pszBackupPath = (char*) pszBackupPath_p;

    RetCode = kEplSuccessful;
Exit:
    return (RetCode);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcSetSignature()
//
// Description: Function set the signature of object dictionary.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     tEplKernel -> error code
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTgtObdArcSetSignature ( tEplObdPart CurrentOdPart_p, DWORD dwSignature_p)
{
tEplKernel RetCode = kEplSuccessful;
tEplTgtObdArcInstance* pInstEntry;

    // get current instance entry
    pInstEntry = &aEplTgtObdArcInstance_l[0];

    // check OD part
    switch (CurrentOdPart_p)
    {
        case kEplObdPartGen: pInstEntry->m_dwOdSigPartGen = dwSignature_p; break;
        case kEplObdPartMan: pInstEntry->m_dwOdSigPartMan = dwSignature_p; break;
        case kEplObdPartDev: pInstEntry->m_dwOdSigPartDev = dwSignature_p; break;

        default:
            RetCode = kEplApiInvalidParam;
            goto Exit;
    }

Exit:
    return (RetCode);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcGetFilePath()
//
// Description: Function returns the file path for the file archive.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = [in]  OD-part
//              pszBackupPath_p        = [in]  directory path
//              pszBackupFilePath_p    = [out] pointer to a buffer for receiving the file path
//
// Returns:     tEplKernel -> error code
//
//---------------------------------------------------------------------------

static tEplKernel EplTgtObdArcGetFilePath ( tEplObdPart CurrentOdPart_p,
    char* pszBackupPath_p, char* pszBackupFilePath_p)
{
tEplKernel RetCode = kEplStoreHwError;
size_t     len;

    // build complete file path string
    if (pszBackupPath_p != NULL)
    {
        strcpy(pszBackupFilePath_p, pszBackupPath_p);
    }
    else
    {
        pszBackupFilePath_p[0] = '\0';
    }
    len = strlen(pszBackupFilePath_p);
    if ((len > 0)
        && (pszBackupFilePath_p[len-1] != '\\')
        && (pszBackupFilePath_p[len-1] != '/'))
    {
        strcat(pszBackupFilePath_p, "/");
    }
    strcat(pszBackupFilePath_p, EPLTGTOBDARC_FILENAME_PREFIX);

    // check OD part
    switch (CurrentOdPart_p)
    {
        case kEplObdPartGen: strcat(pszBackupFilePath_p, "_Com"); break;
        case kEplObdPartMan: strcat(pszBackupFilePath_p, "_Man"); break;
        case kEplObdPartDev: strcat(pszBackupFilePath_p, "_Dev"); break;

        default:
            RetCode = kEplApiInvalidParam;
            goto Exit;
    }

    strcat (pszBackupFilePath_p, EPLTGTOBDARC_FILENAME_EXTENSION);

    RetCode = kEplSuccessful;
Exit:
    return (RetCode);

}

// EOF

