/*
 * Arduino Inclinometer Project, calibration sketch
 *
 * Copyright (C) 2017 Alex Harsanyi (alexharsanyi@gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Helper program to calibrate the ADXL345 accelerometer module.  See the
 * 'Calibration' section in README.md on how to perform calibration.  This
 * sketch just prints the X, Y, Z values read from the accelerometer on the
 * serial output at 38400 baud.
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified adxl345;

void setup() {
    // Setup the accelerometer, 4G range is selected, same as the
    // inclinometer.
    adxl345.begin();
    adxl345.setRange(ADXL345_RANGE_4_G);
    adxl345.setDataRate(ADXL345_DATARATE_50_HZ);

    Serial.begin(38400);
}

void loop() {
    int16_t x = adxl345.getX();
    int16_t y = adxl345.getY();
    int16_t z = adxl345.getZ();

    Serial.print("X = ");
    Serial.print(x);
    Serial.print("; Y = ");
    Serial.print(y);
    Serial.print("; Z = ");
    Serial.print(z);
    Serial.print("\n");

    delay(1000);
}

/*
  Local Variables:
  mode: c++
  End:
*/
