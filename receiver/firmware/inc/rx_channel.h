/**
 * @file rx_channel.h
 * @brief Which channel the receiver listens on: the channel scan (resuming
 *        on the last-locked channel), the operator's manual pick and the
 *        scan lock - with the SD rows and the LASTCH.BIN memory they leave.
 */
#ifndef RX_CHANNEL_H
#define RX_CHANNEL_H

#include <stdint.h>
#include "beacon_track.h"

/** Start (or, after a radio recovery, restart) the channel scan: on the
 *  channel LASTCH.BIN remembers, with the long first dwell that outwaits a
 *  quiet-pad heartbeat, else the plain scan. */
void    RxChannel_StartScan(uint8_t sd_ok);

/** The operator stepped the channel. On a change everything received so far
 *  belonged to the previous rocket (beacon state reset); the pick is logged
 *  and remembered. @retval 1 if the channel changed */
uint8_t RxChannel_ManualStep(BeaconTrack_t *beacon, uint8_t sd_ok);

/** Drive the channel scan; on lock, log the channel and remember it.
 *  @retval 1 the moment the scan locks */
uint8_t RxChannel_ServiceScan(uint8_t sd_ok);

#endif /* RX_CHANNEL_H */
