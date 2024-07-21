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

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_bq4050

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/bus_i2c.h"

#include "sensors/sensors.h"

#include "drivers/sensor.h"
#include "drivers/battery/battery_gauge.h"

#include "drivers/battery/battery_gauge_bq4050.h"

#define bq4050_I2C_ADDRESS     0x16

// Registers
#define bq4050_RELATIVE_SOC 0x0E

uint8_t bq4050RefreshFlag = 0; //1s == 50

static bool bq4050Init(batGaugeDev_t * gauge)
{
    bool ack = true;

    ack = ack && busWrite(gauge->busDev, 0x0B, 0x01);
    // ack = ack && i2cWrite(busWrite(gauge->busDev, 0x20, 0x40);
    // ack = ack && i2cWrite(busWrite(gauge->busDev, 0x21, 0x01);
    ack = ack && busWrite(gauge->busDev, bq4050L_REG_CONF1, bq4050L_MODE_CONTINUOUS | bq4050L_ODR_200HZ | bq4050L_OSR_512 | bq4050L_RNG_8G);

    return ack;
}

static bool bq4050Read(batGaugeDev_t * gauge)
{
    uint8_t status;
    uint8_t buf[6];

    // set magData to zero for case of failed read
    gauge->magADCRaw[X] = 0;
    gauge->magADCRaw[Y] = 0;
    gauge->magADCRaw[Z] = 0;

    bool ack = busRead(gauge->busDev, bq4050L_REG_STATUS, &status);
    if (!ack || (status & 0x04) == 0) {
        return false;
    }

    ack = busReadBuf(gauge->busDev, bq4050L_REG_DATA_OUTPUT_X, buf, 6);
    if (!ack) {
        return false;
    }

    gauge->magADCRaw[X] = (int16_t)(buf[1] << 8 | buf[0]);
    gauge->magADCRaw[Y] = (int16_t)(buf[3] << 8 | buf[2]);
    gauge->magADCRaw[Z] = (int16_t)(buf[5] << 8 | buf[4]);

    return true;
}

static bool deviceDetect(batGaugeDev_t * gauge)
{
    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        // Must write reset first  - don't care about the result
        busWrite(gauge->busDev, bq4050L_REG_CONF2, bq4050L_RST);
        delay(30);

        uint8_t sig = 0;
        bool ack = busRead(gauge->busDev, bq4050L_REG_ID, &sig);

        if (ack && sig == bq4050_ID_VAL) {
            // Should be in standby mode after soft reset and sensor is really present
            // Reading ChipID of 0xFF alone is not sufficient to be sure the QMC is present

            ack = busRead(gauge->busDev, bq4050L_REG_CONF1, &sig);
            if (ack && sig == bq4050L_MODE_STANDBY) {
                return true;
            }
        }
    }

    return false;
}

bool bq4050Detect(batGaugeDev_t* gauge)
{
    gauge->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_bq4050, gauge->magSensorToUse, OWNER_COMPASS);
    if (gauge->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(gauge)) {
        busDeviceDeInit(gauge->busDev);
        return false;
    }

    gauge->init = bq4050Init;
    gauge->read = bq4050Read;

    return true;
}
#endif
