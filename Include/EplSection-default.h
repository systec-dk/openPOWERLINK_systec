/**
********************************************************************************
\file   EplSection-default.h

\brief  Macros for default function linking

This header file defines empty macros if the specific functions are not linked
to a specific section.

Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2012, Kalycito Infotech Private Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef _INC_EPLSECTION_DEFAULT_H_
#define _INC_EPLSECTION_DEFAULT_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// per default, no function placement in special memory
    #ifndef EPL_SECTION_DLLK_FRAME_RCVD_CB
        #define EPL_SECTION_DLLK_FRAME_RCVD_CB
    #endif
    #ifndef EPL_SECTION_PDOK_PROCESS_TPDO_CB
        #define EPL_SECTION_PDOK_PROCESS_TPDO_CB
    #endif
    #ifndef EPL_SECTION_PDOK_ENCODE_TPDO_CB
        #define EPL_SECTION_PDOK_ENCODE_TPDO_CB
    #endif
    #ifndef EPL_SECTION_PDOK_PDO_DECODE
        #define EPL_SECTION_PDOK_PDO_DECODE
    #endif
    #ifndef EPL_SECTION_EVENTK_PROCESS
        #define EPL_SECTION_EVENTK_PROCESS
    #endif
    #ifndef EPL_SECTION_EVENTK_POST
        #define EPL_SECTION_EVENTK_POST
    #endif
    #ifndef EPL_SECTION_OMETHLIB_RX_IRQ_HDL
        #define EPL_SECTION_OMETHLIB_RX_IRQ_HDL
    #endif
    #ifndef EPL_SECTION_OMETHLIB_TX_IRQ_HDL
        #define EPL_SECTION_OMETHLIB_TX_IRQ_HDL
    #endif
    #ifndef EPL_SECTION_EDRVOPENMAC_RX_HOOK
        #define EPL_SECTION_EDRVOPENMAC_RX_HOOK
    #endif
    #ifndef EPL_SECTION_EDRVOPENMAC_IRQ_HDL
        #define EPL_SECTION_EDRVOPENMAC_IRQ_HDL
    #endif
    #ifndef EPL_SECTION_MAIN_APP_CB_SYNC
        #define EPL_SECTION_MAIN_APP_CB_SYNC
    #endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_EPLSECTION_DEFAULT_H_ */
