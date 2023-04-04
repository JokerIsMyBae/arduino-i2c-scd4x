/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SENSIRIONI2CSCD4X_H
#define SENSIRIONI2CSCD4X_H

#define DEFAULT_I2C_ADDR 0x62

#include <Wire.h>

#include <SensirionCore.h>

class SensirionI2CScd4x {

  public:
    SensirionI2CScd4x();
    /**
     * begin() - Initializes the SensirionI2CScd4x class.
     *
     * @param i2cBus Arduino stream object to use for communication.
     * @param i2cAddr I2C address of the sensor.
     *
     */
    void begin(TwoWire& i2cBus, uint8_t i2cAddr = DEFAULT_I2C_ADDR);

    /**
     * readMeasurementTicks() - read sensor output. The measurement data can
     * only be read out once per signal update interval as the buffer is emptied
     * upon read-out. If no data is available in the buffer, the sensor returns
     * a NACK. To avoid a NACK response the get_data_ready_status can be issued
     * to check data status. The I2C master can abort the read transfer with a
     * NACK followed by a STOP condition after any data byte if the user is not
     * interested in subsequent data.
     *
     * @note This command is only available in measurement mode. The firmware
     * updates the measurement values depending on the measurement mode.
     *
     * @param co2 CO₂ concentration in ppm
     *
     * @param temperature Convert value to °C by: -45 °C + 175 °C * value/2^16
     *
     * @param humidity Convert value to %RH by: 100%RH * value/2^16
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t readMeasurementTicks(uint16_t& co2, uint16_t& temperature,
                                  uint16_t& humidity);

    /**
     * readMeasurement() - read sensor output. The measurement data can
     * only be read out once per signal update interval as the buffer is emptied
     * upon read-out. If no data is available in the buffer, the sensor returns
     * a NACK. To avoid a NACK response the get_data_ready_status can be issued
     * to check data status. The I2C master can abort the read transfer with a
     * NACK followed by a STOP condition after any data byte if the user is not
     * interested in subsequent data.
     *
     * @note This command is only available in measurement mode. The firmware
     * updates the measurement values depending on the measurement mode.
     *
     * @param co2 CO₂ concentration in ppm
     *
     * @param temperature Temperature in °C
     *
     * @param humidity Relative humidity in %RH
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t readMeasurement(uint16_t& co2, float& temperature,
                             float& humidity);

    /**
     * getDataReadyFlag() - Check whether new measurement data is available
     * for read-out.
     *
     * @param dataReadyFlag True if valid data is available, false otherwise.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t getDataReadyFlag(bool& dataReadyFlag);

    /**
     * getSerialNumber() - Reading out the serial number can be used to identify
     * the chip and to verify the presence of the sensor. The get serial number
     * command returns 3 words.  Together, the 3 words constitute a unique
     * serial number with a length of 48 bits (big endian format).
     *
     * @param serial0 First word of the 48 bit serial number
     *
     * @param serial1 Second word of the 48 bit serial number
     *
     * @param serial2 Third word of the 48 bit serial number
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t getSerialNumber(uint16_t& serial0, uint16_t& serial1,
                             uint16_t& serial2);

    /**
     * measureSingleShot() - On-demand measurement of CO₂ concentration,
     * relative humidity and temperature. The sensor output is read with the
     * read_measurement command.
     *
     * @note Only available in idle mode.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t measureSingleShot();

    /**
     * powerDown() - Put the sensor from idle to sleep mode to reduce current
     * consumption.
     *
     * @note Only available in idle mode.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t powerDown();

    /**
     * wakeUp() - Wake up sensor from sleep mode to idle mode.
     *
     * @note Only available in sleep mode.
     *
     * @return 0 on success, an error code otherwise
     */
    uint16_t wakeUp();

  private:
    TwoWire* _i2cBus = nullptr;
    uint8_t _i2cAddr;
};

#endif /* SENSIRIONI2CSCD4X_H */
