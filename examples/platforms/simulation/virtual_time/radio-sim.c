/*
 *  Copyright (c) 2016-2022, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#if OPENTHREAD_SIMULATION_VIRTUAL_TIME

#include "platform-simulation.h"
#include "radio.h"
#include "virtual_time/event-sim.h"

#include <sys/time.h>
#include <stdio.h>
#include <execinfo.h>

#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/time.h>

#include "utils/code_utils.h"
#include "utils/mac_frame.h"
#include "utils/soft_source_match_table.h"

#define FAILSAFE_TIME_US 4

// declaration of radio functions only locally used for virtual-time radio
static void setRadioSubState(RadioSubState aState, uint64_t timeToRemainInState);
static void startCcaForTransmission(otInstance *aInstance);

static otRadioState  sLastReportedState   = OT_RADIO_STATE_DISABLED;
static RadioSubState sLastReportedSubState  = OT_RADIO_SUBSTATE_READY;
static uint8_t       sLastReportedChannel = 0;
static uint64_t      sNextRadioEventTime = 0;
static struct RadioCommEventData sLastTxEventData; // metadata about last/ongoing Tx action.
static RadioSubState       sSubState = OT_RADIO_SUBSTATE_READY;

void radioTransmit(struct RadioMessage *aMessage, const struct otRadioFrame *aFrame)
{
    // ( 4B preamble + 1B SFD + 1B PHY header + MAC frame ) @250kbps
    uint64_t frameDurationUs = (6 + aFrame->mLength) * OT_RADIO_SYMBOLS_PER_OCTET * OT_RADIO_SYMBOL_TIME;

    int8_t maxPower            = sChannelMaxTransmitPower[sCurrentChannel - kMinChannel];
    sLastTxEventData.mChannel  = aFrame->mChannel;
    sLastTxEventData.mPower    = sTxPower < maxPower ? sTxPower : maxPower;
    sLastTxEventData.mError    = OT_ERROR_NONE;
    sLastTxEventData.mDuration = frameDurationUs;

    otSimSendRadioCommEvent(&sLastTxEventData, (const uint8_t*) aMessage, aFrame->mLength);
}

void platformRadioReportStateToSimulator()
{
    struct RadioStateEventData stateReport;

    if (sLastReportedState != sState || sLastReportedChannel != sCurrentChannel || sLastReportedSubState != sSubState)
    {
        sLastReportedState    = sState;
        sLastReportedChannel  = sCurrentChannel;
        sLastReportedSubState = sSubState;

        // determine the energy-state from subState. Only in very particular substates,
        // the radio is actively transmitting.
        uint8_t energyState = sState;
        if ( (sState == OT_RADIO_STATE_TRANSMIT && sSubState != OT_RADIO_SUBSTATE_FRAME_ONGOING))
        {
             energyState = OT_RADIO_STATE_RECEIVE;
        }
        else if ( (sState == OT_RADIO_STATE_RECEIVE && sSubState == OT_RADIO_SUBSTATE_ACK_ONGOING))
        {
            energyState = OT_RADIO_STATE_TRANSMIT;
        }

        stateReport.mChannel     = sCurrentChannel;
        stateReport.mEnergyState = energyState;
        stateReport.mSubState    = sSubState;
        stateReport.mTxPower     = sTxPower;
        stateReport.mState       = sState; // also include the OT radio state.

        // determine next radio-event time, so that simulator can guarantee this node will
        // execute again at that time.
        uint64_t delayUntilNextRadioState = 0;
        if (sNextRadioEventTime > otPlatTimeGet())
        {
            delayUntilNextRadioState = sNextRadioEventTime - otPlatTimeGet();
        }
        otSimSendRadioStateEvent(&stateReport, delayUntilNextRadioState);
    }
}

void setRadioState(otRadioState aState)
{
    if (aState != sState)
    {
        //TODO fprintf(stderr, "setRadioState(): sSt=%i aSt=%i sSub=%i\n", sState, aState, sSubState);
        // Check for valid conditions under which state change is allowed.
        assert(sSubState == OT_RADIO_SUBSTATE_READY ||
               sSubState == OT_RADIO_SUBSTATE_IFS_WAIT ||
               ((aState = OT_RADIO_STATE_RECEIVE) && (sSubState == OT_RADIO_SUBSTATE_TX_TO_RX)));
    }
    sState = aState;
}

static void setRadioSubState(RadioSubState aState, uint64_t timeToRemainInState)
{
    if (timeToRemainInState == UNDEFINED_TIME_US)
    {
        sNextRadioEventTime = UNDEFINED_TIME_US;
    }else
    {
        sNextRadioEventTime = otPlatTimeGet() + timeToRemainInState;
    }
    sSubState = aState;
}

static void startCcaForTransmission(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    sTxWait = true;

    // send CCA event, wait for simulator to send back the channel sampling result.
    struct RadioCommEventData chanSampleData;
    chanSampleData.mChannel = sTransmitFrame.mChannel;
    chanSampleData.mDuration = OT_RADIO_CCA_TIME_US;
    otSimSendRadioChanSampleEvent(&chanSampleData);
}

bool platformRadioIsBusy(void)
{
    return (sState == OT_RADIO_STATE_TRANSMIT || sState == OT_RADIO_STATE_RECEIVE ) &&
           (sSubState != OT_RADIO_SUBSTATE_READY);
}

void platformRadioRxStart(otInstance *aInstance, struct RadioCommEventData *aRxParams)
{
    OT_UNUSED_VARIABLE(aInstance);

    otEXPECT(sCurrentChannel == aRxParams->mChannel); // must be on my listening channel.
    otEXPECT(sState == OT_RADIO_STATE_RECEIVE || sState == OT_RADIO_STATE_TRANSMIT); // and in valid states.
    otEXPECT(sSubState == OT_RADIO_SUBSTATE_READY || sSubState == OT_RADIO_SUBSTATE_IFS_WAIT ||
             sSubState == OT_RADIO_SUBSTATE_AIFS_WAIT);

    if (sState == OT_RADIO_STATE_RECEIVE){
        setRadioSubState(OT_RADIO_SUBSTATE_FRAME_ONGOING, aRxParams->mDuration + FAILSAFE_TIME_US);
    }
    else if (sSubState == OT_RADIO_SUBSTATE_AIFS_WAIT)
    {
        setRadioSubState(OT_RADIO_SUBSTATE_ACK_ONGOING, aRxParams->mDuration + FAILSAFE_TIME_US);
    }
    sReceiveFrame.mInfo.mRxInfo.mRssi = aRxParams->mPower;
    sReceiveFrame.mInfo.mRxInfo.mLqi  = OT_RADIO_LQI_NONE; // No support of LQI reporting.

exit:
    return;
}

void platformRadioRxDone(otInstance *aInstance, const uint8_t *aBuf, uint16_t aBufLength, struct RadioCommEventData *aRxParams)
{
    OT_UNUSED_VARIABLE(aInstance);

    otEXPECT(sCurrentChannel == aRxParams->mChannel); // if frame not on my listening channel, ignore.
    otEXPECT(sState == OT_RADIO_STATE_RECEIVE || sState == OT_RADIO_STATE_TRANSMIT); // Only process in valid states.
    otEXPECT(sSubState == OT_RADIO_SUBSTATE_FRAME_ONGOING || sSubState == OT_RADIO_SUBSTATE_ACK_ONGOING  );

    memcpy(&sReceiveMessage, aBuf, aBufLength);

    // TODO verify why the below is 0 and not 1.
    sReceiveFrame.mLength             = (uint8_t)(aBufLength - 0 /*offsetof(struct RadioMessage, mPsdu)*/ );
    sReceiveFrame.mInfo.mRxInfo.mRssi = aRxParams->mPower;
    sReceiveFrame.mInfo.mRxInfo.mLqi  = OT_RADIO_LQI_NONE; // No support of LQI reporting.

    if (sState == OT_RADIO_STATE_RECEIVE &&
        sSubState == OT_RADIO_SUBSTATE_FRAME_ONGOING && otMacFrameIsAckRequested(&sReceiveFrame))
    {
        // wait exactly time AIFS before sending out the Ack.
        setRadioSubState(OT_RADIO_SUBSTATE_AIFS_WAIT, OT_RADIO_AIFS_TIME_US);
    }
    else
    {
        setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, OT_RADIO_SIFS_TIME_US); // TODO lifs
    }

    radioReceive(aInstance, aRxParams->mError );

exit:
    return;
}

void platformRadioCcaDone(otInstance *aInstance, struct RadioCommEventData *aChanData)
{
    OT_UNUSED_VARIABLE(aInstance);
    otEXPECT(aChanData->mChannel == sTransmitFrame.mChannel);
    otEXPECT(sSubState == OT_RADIO_SUBSTATE_CCA);

    if (aChanData->mPower < sCcaEdThresh || aChanData->mPower == OT_RADIO_RSSI_INVALID)  // channel clear?
    {
        setRadioSubState(OT_RADIO_SUBSTATE_CCA_TO_TX, OT_RADIO_TURNAROUND_TIME_US);
    }
    else
    {
        // CCA failure case - channel not clear.
        sTxWait = false;
        setRadioSubState(OT_RADIO_SUBSTATE_READY, 0);
        setRadioState(OT_RADIO_STATE_RECEIVE);

#if OPENTHREAD_CONFIG_DIAG_ENABLE
        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, OT_ERROR_CHANNEL_ACCESS_FAILURE);
        }
        else
#endif
        {
            otPlatRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_CHANNEL_ACCESS_FAILURE);
        }
    }

exit:
    return;
}

void platformRadioTxDone(otInstance *aInstance, struct RadioCommEventData *aTxDoneParams)
{
    OT_UNUSED_VARIABLE(aInstance);

    if (sState == OT_RADIO_STATE_RECEIVE && sSubState == OT_RADIO_SUBSTATE_ACK_ONGOING)
    {
        // Ack Tx is done now.
        setRadioSubState(OT_RADIO_SUBSTATE_TX_TO_RX, OT_RADIO_TURNAROUND_TIME_US);
    }
    else if (sState == OT_RADIO_STATE_TRANSMIT && sSubState == OT_RADIO_SUBSTATE_FRAME_ONGOING)
    {
        // if not waiting for ACK -> go to Rx state; see state diagram.
        // if Tx was failure: no wait for ACK, abort current Tx and go to Rx state.
        if (!otMacFrameIsAckRequested(&sTransmitFrame) || aTxDoneParams->mError != OT_ERROR_NONE)
        {
            setRadioSubState(OT_RADIO_SUBSTATE_TX_TO_RX, OT_RADIO_TURNAROUND_TIME_US);
#if OPENTHREAD_CONFIG_DIAG_ENABLE
            if (otPlatDiagModeGet())
            {
                otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, aTxDoneParams->mError);
            }
            else
#endif
            {
                otPlatRadioTxDone(aInstance, &sTransmitFrame, NULL, aTxDoneParams->mError);
            }
        }
        else if (otMacFrameIsAckRequested(&sTransmitFrame))
        {
            setRadioSubState(OT_RADIO_SUBSTATE_TX_TO_RX, OT_RADIO_TURNAROUND_TIME_US);
        }
    }
}

void platformRadioProcess(otInstance *aInstance, const fd_set *aReadFdSet, const fd_set *aWriteFdSet)
{
    OT_UNUSED_VARIABLE(aReadFdSet);
    OT_UNUSED_VARIABLE(aWriteFdSet);

    // execute time and data based state transitions for substate. Event based transitions is in platform-sim.c
    if (sState == OT_RADIO_STATE_TRANSMIT && otPlatTimeGet() >= sNextRadioEventTime)
    {
        switch (sSubState)
        {
        case OT_RADIO_SUBSTATE_READY:  // initial substate
            if (platformRadioIsTransmitPending())
            {
                setRadioSubState(OT_RADIO_SUBSTATE_CCA, OT_RADIO_CCA_TIME_US + FAILSAFE_TIME_US);
                startCcaForTransmission(aInstance);
            }
            break;

        case OT_RADIO_SUBSTATE_CCA:
            setRadioSubState(OT_RADIO_SUBSTATE_CCA_TO_TX, OT_RADIO_TURNAROUND_TIME_US);
            break;

        case OT_RADIO_SUBSTATE_CCA_TO_TX:
            setRadioSubState(OT_RADIO_SUBSTATE_FRAME_ONGOING, sLastTxEventData.mDuration + FAILSAFE_TIME_US);
            radioSendMessage(aInstance);
            break;

        case OT_RADIO_SUBSTATE_FRAME_ONGOING:
            setRadioSubState(OT_RADIO_SUBSTATE_TX_TO_RX, OT_RADIO_TURNAROUND_TIME_US);
            break;

        case OT_RADIO_SUBSTATE_TX_TO_RX:
            if (otMacFrameIsAckRequested(&sTransmitFrame))
            {
                // set a max wait time for start of Ack frame to be received.
                setRadioSubState(OT_RADIO_SUBSTATE_AIFS_WAIT, OT_RADIO_MAX_ACK_WAIT_US);
            }
            else
            {
                // no Ack was requested
                setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, OT_RADIO_SIFS_TIME_US - OT_RADIO_TURNAROUND_TIME_US); // TODO LIFS also
                sTxWait = false;
            }
            break;

        case OT_RADIO_SUBSTATE_AIFS_WAIT:
            // if we arrive here on the timeout timer, an Ack or frame start wasn't received in the meantime.
            // so go to ready state and fail the Tx.
            setRadioSubState(OT_RADIO_SUBSTATE_READY, UNDEFINED_TIME_US);
            setRadioState(OT_RADIO_STATE_RECEIVE);
            sTxWait = false;
#if OPENTHREAD_CONFIG_DIAG_ENABLE
            if (otPlatDiagModeGet())
            {
                otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, OT_ERROR_NO_ACK);
            }
            else
#endif
            {
                otPlatRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_NO_ACK);
            }
            break;

        case OT_RADIO_SUBSTATE_ACK_ONGOING:
            // wait until Ack receive is done. In platformRadioRxDone() the next state is selected.
            // if we get here on the timer, this ongoing Ack wasn't received properly.
            setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, OT_RADIO_SIFS_TIME_US); // TODO lifs
            sTxWait = false;
#if OPENTHREAD_CONFIG_DIAG_ENABLE
            if (otPlatDiagModeGet())
            {
                otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, OT_ERROR_NO_ACK);
            }
            else
#endif
            {
                otPlatRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_NO_ACK);
            }
            break;

        case OT_RADIO_SUBSTATE_IFS_WAIT:
            setRadioSubState(OT_RADIO_SUBSTATE_READY, UNDEFINED_TIME_US);
            setRadioState(OT_RADIO_STATE_RECEIVE);
            sTxWait = false;
            break;

        case OT_RADIO_SUBSTATE_ENERGY_SCAN:
            assert(false && "Illegal state energy scan in Tx");

        default:
            assert(false && "Illegal state in Tx");

        }
    }

    // Receive state machine
    if (sState == OT_RADIO_STATE_RECEIVE && otPlatTimeGet() >= sNextRadioEventTime)
    {
        switch (sSubState)
        {
        case OT_RADIO_SUBSTATE_READY:  // initial substate
            // wait until a frame Rx starts. In platformRadioRxStart() the next state is selected.
            break;

        case OT_RADIO_SUBSTATE_FRAME_ONGOING:
            // wait until frame Rx is done. In platformRadioRxDone() the next state is selected.
            // below is a timer-based failsafe in case the RxDone message from simulator was never received.
            setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, OT_RADIO_SIFS_TIME_US); // TODO LIFS also
            break;

        case OT_RADIO_SUBSTATE_AIFS_WAIT:
            // if Ack is ready to be transmitted after AIFS period, send it.
            radioPrepareAck();  // prepare the Ack again, now (redo it - with proper CSL timing)
            radioTransmit(&sAckMessage, &sAckFrame);  // send the Ack
            setRadioSubState(OT_RADIO_SUBSTATE_ACK_ONGOING, sLastTxEventData.mDuration);
            break;

        case OT_RADIO_SUBSTATE_ACK_ONGOING:
            // at end of Ack transmission.
            setRadioSubState(OT_RADIO_SUBSTATE_TX_TO_RX, OT_RADIO_TURNAROUND_TIME_US);
            break;

        case OT_RADIO_SUBSTATE_TX_TO_RX:
            // After Ack Tx.
            setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, OT_RADIO_SIFS_TIME_US - OT_RADIO_TURNAROUND_TIME_US); // TODO LIFS also
            break;

        case OT_RADIO_SUBSTATE_IFS_WAIT:
            setRadioSubState(OT_RADIO_SUBSTATE_READY, UNDEFINED_TIME_US);
            setRadioState(OT_RADIO_STATE_RECEIVE); // ready for next Rx or Tx
            break;

        case OT_RADIO_SUBSTATE_ENERGY_SCAN:
            if (IsTimeAfterOrEqual(otPlatAlarmMilliGetNow(), sEnergyScanEndTime))
            {
                otPlatRadioEnergyScanDone(aInstance, sEnergyScanResult);
                setRadioSubState(OT_RADIO_SUBSTATE_READY, UNDEFINED_TIME_US);
                sEnergyScanning = false;
            }
            break;

        case OT_RADIO_SUBSTATE_CCA:
            assert(false && "Illegal Rx substate OT_RADIO_SUBSTATE_CCA");

        case OT_RADIO_SUBSTATE_CCA_TO_TX:
            assert(false && "Illegal Rx substate OT_RADIO_SUBSTATE_CCA_TO_TX");

        default:
            assert(false && "Illegal state in Rx");

        }
    }
}

#endif // OPENTHREAD_SIMULATION_VIRTUAL_TIME
