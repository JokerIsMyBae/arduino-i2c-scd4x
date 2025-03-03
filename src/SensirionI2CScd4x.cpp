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

#include "SensirionI2CScd4x.h"
#include "Arduino.h"
#include "SensirionCore.h"
#include <Wire.h>

SensirionI2CScd4x::SensirionI2CScd4x() {
}

void SensirionI2CScd4x::begin(TwoWire& i2cBus, uint8_t i2cAddr) {
    _i2cBus = &i2cBus;
    _i2cAddr = i2cAddr;
}

uint16_t SensirionI2CScd4x::readMeasurementTicks(uint16_t& co2,
                                                 uint16_t& temperature,
                                                 uint16_t& humidity) {
    uint16_t error;
    uint8_t buffer[9];
    SensirionI2CTxFrame txFrame(buffer, 9);

    error = txFrame.addCommand(0xEC05);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(_i2cAddr, txFrame, *_i2cBus);
    if (error) {
        return error;
    }

    delay(1);

    SensirionI2CRxFrame rxFrame(buffer, 9);
    error =
        SensirionI2CCommunication::receiveFrame(_i2cAddr, 9, rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt16(co2);
    error |= rxFrame.getUInt16(temperature);
    error |= rxFrame.getUInt16(humidity);
    return error;
}

uint16_t SensirionI2CScd4x::readMeasurement(uint16_t& co2, float& temperature,
                                            float& humidity) {
    uint16_t error;
    uint16_t temperatureTicks;
    uint16_t humidityTicks;

    error = readMeasurementTicks(co2, temperatureTicks, humidityTicks);
    if (error) {
        return error;
    }

    temperature = static_cast<float>(temperatureTicks * 175.0 / 65535.0 - 45.0);
    humidity = static_cast<float>(humidityTicks * 100.0 / 65535.0);
    return NoError;
}

uint16_t SensirionI2CScd4x::getDataReadyFlag(bool& dataReady) {
    uint16_t error;
    uint16_t localDataReady = 0;
    uint8_t buffer[3];
    SensirionI2CTxFrame txFrame(buffer, 3);

    error = txFrame.addCommand(0xE4B8);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(_i2cAddr, txFrame, *_i2cBus);
    if (error) {
        return error;
    }

    delay(1);

    SensirionI2CRxFrame rxFrame(buffer, 3);
    // Changes the frame itself with pointer
    error =
        SensirionI2CCommunication::receiveFrame(_i2cAddr, 3, rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    // Changes the localDataReady variable itself with pointer
    error |= rxFrame.getUInt16(localDataReady);
    dataReady = (localDataReady & 0x07FF) != 0;  // 0x07FF = 0000 0111 1111 1111
    return error;
}

uint16_t SensirionI2CScd4x::getSerialNumber(uint16_t& serial0,
                                            uint16_t& serial1,
                                            uint16_t& serial2) {
    uint16_t error;
    uint8_t buffer[9];
    SensirionI2CTxFrame txFrame(buffer, 9);

    error = txFrame.addCommand(0x3682);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(_i2cAddr, txFrame, *_i2cBus);
    if (error) {
        return error;
    }

    delay(1);

    SensirionI2CRxFrame rxFrame(buffer, 9);
    error =
        SensirionI2CCommunication::receiveFrame(_i2cAddr, 9, rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt16(serial0);
    error |= rxFrame.getUInt16(serial1);
    error |= rxFrame.getUInt16(serial2);
    return error;
}

uint16_t SensirionI2CScd4x::measureSingleShot() {
    uint16_t error;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame(buffer, 2);

    error = txFrame.addCommand(0x219D);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(_i2cAddr, txFrame, *_i2cBus);

    delay(1);

    return error;
}

uint16_t SensirionI2CScd4x::powerDown() {
    uint16_t error;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame(buffer, 2);

    error = txFrame.addCommand(0x36E0);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(_i2cAddr, txFrame, *_i2cBus);

    delay(1);

    return error;
}

uint16_t SensirionI2CScd4x::wakeUp() {
    uint16_t error;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame(buffer, 2);

    error = txFrame.addCommand(0x36F6);
    if (error) {
        return error;
    }

    // Sensor does not acknowledge the wake-up call, error is ignored
    static_cast<void>(
        SensirionI2CCommunication::sendFrame(_i2cAddr, txFrame, *_i2cBus));

    delay(1);

    return NoError;
}
