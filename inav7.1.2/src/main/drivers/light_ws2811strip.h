/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include "common/color.h"
#include "config/parameter_group.h"

#define WS2811_LED_STRIP_LENGTH 128
#define WS2811_BITS_PER_LED 24
#define WS2811_DELAY_BUFFER_LENGTH 42 // for 50us delay 

#define WS2811_DATA_BUFFER_SIZE (WS2811_BITS_PER_LED * WS2811_LED_STRIP_LENGTH)

#define WS2811_DMA_BUFFER_SIZE (WS2811_DELAY_BUFFER_LENGTH + WS2811_DATA_BUFFER_SIZE + 1)   // leading bytes (reset low 302us) + data bytes LEDS*3  + 1 byte(keep line high optionally)

#define WS2811_TIMER_HZ         2400000
#define WS2811_CARRIER_HZ       800000

typedef enum {
    LED_PIN_PWM_MODE_SHARED_LOW = 0,
    LED_PIN_PWM_MODE_SHARED_HIGH = 1,
    LED_PIN_PWM_MODE_LOW = 2,
    LED_PIN_PWM_MODE_HIGH = 3
} led_pin_pwm_mode_e;

typedef struct ledPinConfig_s {
    uint8_t led_pin_pwm_mode;  //led_pin_pwm_mode_e
} ledPinConfig_t;

PG_DECLARE(ledPinConfig_t, ledPinConfig);

void ws2811LedStripInit(void);
void ws2811LedStripHardwareInit(void);
void ws2811LedStripDMAEnable(void);
bool ws2811LedStripDMAInProgress(void);

//value 0...100
void ledPinStartPWM(uint16_t value);
void ledPinStopPWM(void);

void ws2811UpdateStrip(void);

void setLedHsv(uint16_t index, const hsvColor_t *color);
void getLedHsv(uint16_t index, hsvColor_t *color);

void scaleLedValue(uint16_t index, const uint8_t scalePercent);
void setLedValue(uint16_t index, const uint8_t value);

void setStripColor(const hsvColor_t *color);
void setStripColors(const hsvColor_t *colors);

bool isWS2811LedStripReady(void);
