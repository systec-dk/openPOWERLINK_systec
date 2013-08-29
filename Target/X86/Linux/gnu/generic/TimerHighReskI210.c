/****************************************************************************

 Copyright (c) 2013 Kalycito Infotech Private Limited

 Project: openPOWERLINK

 Description: Intel I210's hardware timer module - High Resolution timer.

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
 1. TimerHighreskX86.c
 ****************************************************************************/

#include "EplInc.h"
#include "kernel/EplTimerHighResk.h"
#include "Benchmark.h"
#include "edrv.h"

//#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>

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

#define TIMER_COUNT           1

#define TIMER_MIN_VAL_SINGLE  5000         /* min 5us */
#define TIMER_MIN_VAL_CYCLE   100000       /* min 100us */

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
#define HRT_DBG_POST_TRACE_VALUE(Event_p, uiNodeId_p, wErrorCode_p) \
    TGT_DBG_POST_TRACE_VALUE((0xE << 28) | (Event_p << 24) \
                             | (uiNodeId_p << 16) | wErrorCode_p)

#define TIMERHDL_MASK         0x0FFFFFFF
#define TIMERHDL_SHIFT        28
#define HDL_TO_IDX(Hdl)       ((Hdl >> TIMERHDL_SHIFT) - 1)
#define HDL_INIT(Idx)         ((Idx + 1) << TIMERHDL_SHIFT)
#define HDL_INC(Hdl)          (((Hdl + 1) & TIMERHDL_MASK) \
                               | (Hdl & ~TIMERHDL_MASK))

//---------------------------------------------------------------------------
//                           module global types
//---------------------------------------------------------------------------

typedef struct
{
    tEplTimerEventArg       m_EventArg;
    tEplTimerkCallback      m_pfnCallback;
    BOOL                    m_fContinuously;
    unsigned long long      m_ullPeriod;

} tEplTimerHighReskTimerInfo;

typedef struct
{
    tEplTimerHighReskTimerInfo m_aTimerInfo[TIMER_COUNT];

} tEplTimerHighReskInstance;

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplTimerHighReskInstance EplTimerHighReskInstance_l;
//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

void EplTimerHighReskCallback(tEplTimerHdl* pTimerHdl_p);

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//------------------------------------------------------------------------------
/**
 \brief     EplTimerHighReskInit()
 initializes the high resolution timer module.

 \param     void

 \return    tEplKernel

 \retval    error code

 */
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskInit(void)
{
    tEplKernel Ret;

    Ret = EplTimerHighReskAddInstance();

    return Ret;

}
//------------------------------------------------------------------------------
/**
 \brief     EplTimerHighReskAddInstance()
 initializes the high resolution timer module.

 \param     void

 \return    tEplKernel

 \retval    error code

 */
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskAddInstance(void)
{
    tEplKernel Ret;

    Ret = kEplSuccessful;

    EPL_MEMSET(&EplTimerHighReskInstance_l, 0, sizeof(EplTimerHighReskInstance_l));

    return Ret;
}

//------------------------------------------------------------------------------
/**
 \brief     EplTimerHighReskDelInstance()
 initializes the high resolution timer module.

 \param     void

 \return    tEplKernel

 \retval    error code

 */
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskDelInstance(void)
{
    tEplTimerHighReskTimerInfo* pTimerInfo;
    tEplKernel Ret;
    unsigned int uiIndex;

    Ret = kEplSuccessful;

    for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++)
    {
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        pTimerInfo->m_pfnCallback = NULL;

        Ret = EdrvStopTimer(&pTimerInfo->m_EventArg.m_TimerHdl);
        if (Ret != kEplSuccessful)
        {
            break;
        }
        pTimerInfo->m_EventArg.m_TimerHdl = 0;
    }

    return Ret;
}

//------------------------------------------------------------------------------
/**
 \brief     EplTimerHighReskModifyTimerNs()

     modifies the timeout of the timer with the specified handle.
     If the handle the pointer points to is zero, the timer must
     be created first.
     If it is not possible to stop the old timer,
     this function always assures that the old timer does not
     trigger the callback function with the same handle as the new
     timer. That means the callback function must check the passed
     handle with the one returned by this function. If these are
     unequal, the call can be discarded.

 \param     pTimerHdl_p   pointer to timer handle
 \param     ullTimeNs_p   relative timeout in [ns]
 \param     pfnCallback_p callback function, which is called mutual
                          exclusive with the Edrv callback functions
                          (Rx and Tx).
 \param     ulArgument_p  	user-specific argument
 \param     fContinuously_p if TRUE, callback function will be called
            continuously otherwise, it is a oneshot timer.

 \return    tEplKernel

 \retval    error code

 */
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskModifyTimerNs(
                            tEplTimerHdl* pTimerHdl_p,
                            unsigned long long ullTimeNs_p,
                            tEplTimerkCallback pfnCallback_p,
                            unsigned long ulArgument_p,
                            BOOL fContinuously_p)
{
    tEplKernel Ret;
    unsigned int uiIndex;
    tEplTimerHighReskTimerInfo* pTimerInfo;
    DWORD dwTimerFreq;

    Ret = kEplSuccessful;

    // check pointer to handle
    if (pTimerHdl_p == NULL )
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {          // no timer created yet

        // search free timer info structure
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];
        for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
        {
            if (pTimerInfo->m_EventArg.m_TimerHdl == 0)
            {          // free structure found
                break;
            }
        }

        if (uiIndex >= TIMER_COUNT)
        {          // no free structure found
            Ret = kEplTimerNoTimerCreated;
            goto Exit;
        }

        pTimerInfo->m_EventArg.m_TimerHdl = HDL_INIT(uiIndex);
    }
    else
    {
        uiIndex = HDL_TO_IDX(*pTimerHdl_p);
        if (uiIndex >= TIMER_COUNT)
        {          // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }

        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
    }

    /*
     * increment timer handle
     * (if timer expires right after this statement, the user
     * would detect an unknown timer handle and discard it)
     */
    pTimerInfo->m_EventArg.m_TimerHdl =
            HDL_INC(pTimerInfo->m_EventArg.m_TimerHdl);
    *pTimerHdl_p = pTimerInfo->m_EventArg.m_TimerHdl;

    if (fContinuously_p != FALSE)
    {
        if (ullTimeNs_p < TIMER_MIN_VAL_CYCLE)
        {
            ullTimeNs_p = TIMER_MIN_VAL_CYCLE;
        }
    }
    else
    {
        if (ullTimeNs_p < TIMER_MIN_VAL_SINGLE)
        {
            ullTimeNs_p = TIMER_MIN_VAL_SINGLE;
        }
    }

    pTimerInfo->m_EventArg.m_Arg.m_dwVal = ulArgument_p;
    pTimerInfo->m_pfnCallback = pfnCallback_p;
    pTimerInfo->m_ullPeriod = ullTimeNs_p;
    pTimerInfo->m_fContinuously = fContinuously_p;

    Ret = EdrvRegisterHighResCallback(EplTimerHighReskCallback);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    dwTimerFreq = (DWORD) (ullTimeNs_p);

    // Start the Timer

    Ret = EdrvStartTimer(&pTimerInfo->m_EventArg.m_TimerHdl, dwTimerFreq);

    Exit: return Ret;

}

//------------------------------------------------------------------------------
/**
 \brief     EplTimerHighReskDeleteTimer()
 deletes the timer with the specified handle. Afterward the
 handle is set to zero.

 \param     pTimerHdl_p    pointer to timer handle

 \return    tEplKernel

 \retval    error code

 */
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskDeleteTimer(tEplTimerHdl* pTimerHdl_p)
{
    tEplKernel Ret = kEplSuccessful;
    unsigned int uiIndex;
    tEplTimerHighReskTimerInfo* pTimerInfo;

    // check pointer to handle

    if (pTimerHdl_p == NULL )
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {          // no timer created yet
        goto Exit;
    }
    else
    {
        uiIndex = HDL_TO_IDX(*pTimerHdl_p);
        if (uiIndex >= TIMER_COUNT)
        {          // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        if (pTimerInfo->m_EventArg.m_TimerHdl != *pTimerHdl_p)
        {          // invalid handle
            goto Exit;
        }
    }

    EdrvStopTimer(pTimerHdl_p);

    *pTimerHdl_p = 0;
    pTimerInfo->m_EventArg.m_TimerHdl = 0;
    pTimerInfo->m_pfnCallback = NULL;

    Exit: return Ret;

}

//------------------------------------------------------------------------------
/**
 \brief     EplTimerHighReskCallback()
 Callback function commonly used for all timers.

 \param     pTimer_p    pointer to hrtimer

 \return    void

 */
//------------------------------------------------------------------------------
void EplTimerHighReskCallback(tEplTimerHdl* pTimerHdl_p)
{
    unsigned int uiIndex;
    tEplTimerHdl OrgTimerHdl;
    tEplTimerHighReskTimerInfo* pTimerInfo;

    uiIndex = HDL_TO_IDX(*pTimerHdl_p);
    if (uiIndex >= TIMER_COUNT)
    {          // invalid handle
        goto Exit;
    }
    pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];

    OrgTimerHdl = *pTimerHdl_p;

    if (pTimerInfo->m_pfnCallback != NULL )
    {
        pTimerInfo->m_pfnCallback(&pTimerInfo->m_EventArg);
    }

    if (OrgTimerHdl != pTimerInfo->m_EventArg.m_TimerHdl)
    {
        goto Exit;
    }

    if (pTimerInfo->m_fContinuously)
    {
        EdrvEnableTimer(pTimerHdl_p);
    }
    else
    {
        EdrvStopTimer(pTimerHdl_p);
    }
    Exit: return;
}
