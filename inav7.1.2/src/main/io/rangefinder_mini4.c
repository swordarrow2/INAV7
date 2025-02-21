#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "build/build_config.h"
#include "build/debug.h"
#include "common/maths.h"
#include "io/serial.h"
#include "platform.h"

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_MINI4)
#include "drivers/rangefinder/rangefinder_virtual.h"
#include "drivers/time.h"

//  AE A7 17 00 85 00 00 00 34 00 00 00 34 00 00 00 00 00 00 00 00 00 00 01 05 BC BE

//   0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f 10 11 12 13 14 15 16 17 18 19 1a
//   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
//
typedef struct __attribute__((packed)) {
    uint8_t header0; // 0xAE
    uint8_t header1; // 0xA7

    uint8_t length; // 23 0x17
    uint8_t addr; // 0x00
    uint8_t cmd; // 0x85
    uint8_t unuse1;
    uint8_t unuse2;
    uint8_t length1;
    uint8_t length2;
    uint8_t unuse5;
    uint8_t unuse6;
    uint8_t length1_1;
    uint8_t length1_2;
    uint8_t unuse9;
    uint8_t unuse10;
    uint8_t unuse11;
    uint8_t unuse12;
    uint8_t unuse13;
    uint8_t unuse14;
    uint8_t unuse15;
    uint8_t unuse16;
    uint8_t unuse17;
    uint8_t unuse18;
    uint8_t unit;

    uint8_t checksum;
    uint8_t end1;
    uint8_t end2;
} mini4Packet_t;

#define MINI4_PACKET_SIZE sizeof(mini4Packet_t)
#define MINI4_TIMEOUT_MS 500 // 500ms

static serialPort_t* serialPort = NULL;
static serialPortConfig_t* portConfig;
static uint8_t buffer[MINI4_PACKET_SIZE];
static unsigned bufferPtr;
static timeMs_t lastProtocolActivityMs;

static bool hasNewData = false;
static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;
static const uint8_t initCmd2Hz[] = { 0xAE, 0xA7, 0x04, 0x00, 0x05, 0x09, 0xBC, 0xBE };

static bool mini4RangefinderDetect(void)
{
    portConfig = findSerialPortConfig(FUNCTION_RANGEFINDER);
    if (!portConfig) {
        return false;
    }
    return true;
}

static void mini4RangefinderInit(void)
{
    if (!portConfig) {
        return;
    }
    serialPort = openSerialPort(portConfig->identifier, FUNCTION_RANGEFINDER, NULL, NULL, portConfig->peripheral_baudrateIndex, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!serialPort) {
        return;
    }
    lastProtocolActivityMs = 0;
    bufferPtr = 0;
}

static void mini4RangefinderUpdate(void)
{
    uint8_t checksum = 0;
    uint8_t flag = 0;
    if (lastProtocolActivityMs == 0 || (millis() - lastProtocolActivityMs) > MINI4_TIMEOUT_MS) {
        serialWriteBuf(serialPort, initCmd2Hz, sizeof(initCmd2Hz));
        lastProtocolActivityMs = millis();
        // Process incoming bytes
        mini4Packet_t* mini4Packet = (mini4Packet_t*)buffer;
        while (serialRxBytesWaiting(serialPort) > 0) {
            uint8_t c = serialRead(serialPort);
            if (bufferPtr < MINI4_PACKET_SIZE) {
                buffer[bufferPtr++] = c;
            }
            if (((bufferPtr == 1) && (mini4Packet->header0 != 0xAE)) || ((bufferPtr == 2) && (mini4Packet->header1 != 0xA7))) {
                bufferPtr = 0;
                continue;
            }
            if ((bufferPtr == 3) && (mini4Packet->length == 0x4)) {
                hasNewData = true;
                lastProtocolActivityMs = millis();
                sensorData = RANGEFINDER_OUT_OF_RANGE;
                bufferPtr = 0;
                continue;
            }
            if (((bufferPtr == 3) && (mini4Packet->length != 0x17)) || ((bufferPtr == 26) && (mini4Packet->end1 != 0xBC)) || ((bufferPtr == 27) && (mini4Packet->end2 != 0xBE))) {
                bufferPtr = 0;
                continue;
            }

            // Check for complete packet
            if (bufferPtr == MINI4_PACKET_SIZE) {
                for (flag = 2; flag < MINI4_PACKET_SIZE - 3; flag++) {
                    checksum += buffer[flag];
                }

                if (mini4Packet->checksum == checksum) {
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

static int32_t mini4RangefinderGetDistance(void)
{
    return sensorData;
    // return (sensorData > 0) ? (sensorData) : 1;
    //  if (hasNewData) {
    //      hasNewData = false;
    //      return (sensorData > 0) ? (sensorData) : RANGEFINDER_OUT_OF_RANGE;
    //  }
    //  else {
    //      return RANGEFINDER_OUT_OF_RANGE;
    //  }
}

virtualRangefinderVTable_t rangefinderMini4Vtable = {
    .detect = mini4RangefinderDetect,
    .init = mini4RangefinderInit,
    .update = mini4RangefinderUpdate,
    .read = mini4RangefinderGetDistance
};

#endif
