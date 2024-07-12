/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"

#include "io/serial.h"

#if defined(USE_RANGEFINDER_BENEWAKE)
#include "drivers/rangefinder/rangefinder_virtual.h"
#include "drivers/time.h"
#include "io/rangefinder.h"
//AE A7 17 00 85 00 00 00 34 00 00 00 34 00 00 00 00 00 00 00 00 00 00 01 05 BC BE 
// 0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f 10 11 12 13 14 15 16 17 18 19 1a
// 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
typedef struct __attribute__((packed)) {
    uint8_t     header0;//0xAE
    uint8_t     header1;//0xA7

    uint8_t     length;//23 0x17
    uint8_t     addr; //0x00
    uint8_t     cmd; //0x85
    uint8_t     unuse1;
    uint8_t     unuse2;
    uint8_t     length1;
    uint8_t     length2;
    uint8_t     unuse5;
    uint8_t     unuse6;
    uint8_t     length1_1;
    uint8_t     length1_2;
    uint8_t     unuse9;
    uint8_t     unuse10;
    uint8_t     unuse11;
    uint8_t     unuse12;
    uint8_t     unuse13;
    uint8_t     unuse14;
    uint8_t     unuse15;
    uint8_t     unuse16;
    uint8_t     unuse17;
    uint8_t     unuse18;
    uint8_t     unit;
    
    uint8_t     checksum;
    uint8_t     end1;
    uint8_t     end2;
} mini4Packet_t;

#define MINI4_PACKET_SIZE    sizeof(mini4Packet_t)
#define MINI4_MIN_QUALITY    0
#define MINI4_TIMEOUT_MS     500 // 500ms

static serialPort_t * serialPort = NULL;
static serialPortConfig_t * portConfig;
static uint8_t  buffer[MINI4_PACKET_SIZE];
static unsigned bufferPtr;
static timeMs_t lastProtocolActivityMs;

static bool hasNewData = false;
static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;

// TFmini command to initiate 2Hz sampling
static const uint8_t initCmd2Hz[] = {0xAE , 0xA7 , 0x04 , 0x00 , 0x05 , 0x09 , 0xBC , 0xBE};

static bool benewakeRangefinderDetect(void)
{
    portConfig = findSerialPortConfig(FUNCTION_RANGEFINDER);
    if (!portConfig) {
        return false;
    }

    return true;
}

static void benewakeRangefinderInit(void)
{
    if (!portConfig) {
        return;
    }

    serialPort = openSerialPort(portConfig->identifier, FUNCTION_RANGEFINDER, NULL, NULL, 9600, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!serialPort) {
        return;
    }

    lastProtocolActivityMs = 0;
    bufferPtr = 0;
}

static void benewakeRangefinderUpdate(void)
{
    uint8_t checksum = 0;
    uint8_t flag = 0;
    // Initialize the sensor
    if (lastProtocolActivityMs == 0 || (millis() - lastProtocolActivityMs) > MINI4_TIMEOUT_MS) {
        serialWriteBuf(serialPort, initCmd2Hz, sizeof(initCmd2Hz));
        lastProtocolActivityMs = millis();

    // Process incoming bytes
    mini4Packet_t *mini4Packet = (mini4Packet_t *)buffer;
    while (serialRxBytesWaiting(serialPort) > 0) {
        uint8_t c = serialRead(serialPort);

        // Add byte to buffer
        if (bufferPtr < MINI4_PACKET_SIZE) {
            buffer[bufferPtr++] = c;
        }

        // Check header bytes
        if ((bufferPtr == 1) && (mini4Packet->header0 != 0xAE)) {
            bufferPtr = 0;
            continue;
        }

        if ((bufferPtr == 2) && (mini4Packet->header1 != 0xA7)) {
            bufferPtr = 0;
            continue;
        }

        if ((bufferPtr == 3) && (mini4Packet->length == 0x4)) {
            hasNewData = true;
            lastProtocolActivityMs = millis();
            sensorData = 1;
            bufferPtr = 0;
            continue;
        }

        if ((bufferPtr == 3) && (mini4Packet->length != 0x17)) {
            bufferPtr = 0;
            continue;
        }

        if ((bufferPtr == 26) && (mini4Packet->end1 != 0xBC)) {
            bufferPtr = 0;
            continue;
        }

        if ((bufferPtr == 27) && (mini4Packet->end2 != 0xBE)) {
            bufferPtr = 0;
            continue;
        }

        // Check for complete packet
        if (bufferPtr == MINI4_PACKET_SIZE) {
            for(flag = 2;flag < MINI4_PACKET_SIZE - 3;flag++){
                checksum += buffer[flag];
            }
            
            if (mini4Packet->checksum == checksum) {
                // Valid packet
                hasNewData = true;
                sensorData = (mini4Packet->length1 << 8) | (mini4Packet->length2 << 0);
                sensorData *= 10;
                lastProtocolActivityMs = millis();
            }

            // Prepare for new packet
            bufferPtr = 0;
        }
    }
      }
}

static int32_t benewakeRangefinderGetDistance(void)
{
    return (sensorData > 0) ? (sensorData) : 1;
    // if (hasNewData) {
    //     hasNewData = false;
    //     return (sensorData > 0) ? (sensorData) : RANGEFINDER_OUT_OF_RANGE;
    // }
    // else {
    //     return RANGEFINDER_OUT_OF_RANGE;
    // }
}

virtualRangefinderVTable_t rangefinderBenewakeVtable = {
    .detect = benewakeRangefinderDetect,
    .init = benewakeRangefinderInit,
    .update = benewakeRangefinderUpdate,
    .read = benewakeRangefinderGetDistance
};

#endif
