//
// TODO
//

#ifndef OPENTHREAD_EVENT_SIM_H
#define OPENTHREAD_EVENT_SIM_H

#include "../platform-simulation.h"

/**
 * This function sends a generic simulation event to the simulator. Event fields are
 * updated to the values as were used for sending it.
 *
 * @param[in,out]   aEvent  A pointer to the simulation event to send.
 *
 */
void otSimSendEvent(struct Event *aEvent);

/**
 * This function sends a sleep event to the simulator. The amount of time to sleep
 * for this node is determined by the alarm timer, by calling platformAlarmGetNext().
 */
void otSimSendSleepEvent(void);

/**
 * This function sends a RadioComm (Tx) simulation event to the simulator.
 *
 * @param[in]       aEventData A pointer to specific data for RadioComm event.
 * @param[in]       aPayload     A pointer to the data payload (radio frame) to send.
 * @param[in]       aLenPayload  Length of aPayload data.
 */
void otSimSendRadioCommEvent(struct RadioCommEventData *aEventData,  const uint8_t *aPayload, size_t aLenPayload);

/**
 * This function sends a Radio State simulation event to the simulator.
 *
 * @param[in]  aStateData   A pointer to specific data for Radio State event.
 */
void otSimSendRadioStateEvent(struct RadioStateEventData *aStateData);

/**
 * This functions sends a channel sample simulation event to the simulator. It is used
 * for CCA or energy scanning on channels.
 *
 * @param[in]  aChanData    A pointer to channel-sample data instructing what to sample.
 */
void otSimSendRadioChanSampleEvent(struct RadioCommEventData *aChanData);

/**
 * This function sends a Uart data event to the simulator.
 *
 * @param[in]   aData       A pointer to the UART data.
 * @param[in]   aLength     Length of UART data.
 *
 */
void otSimSendUartWriteEvent(const uint8_t *aData, uint16_t aLength);

/**
 * This function sends status push data event to the OT-NS simulator.
 *
 * @param[in]   aStatus     A pointer to the status string data.
 * @param[in]   aLength     Length of status string data.
 *
 */
void otSimSendOtnsStatusPushEvent(const char *aStatus, uint16_t aLength);

#endif // OPENTHREAD_EVENT_SIM_H
