/*
  backend driver for own airspeed sensor made from two MS5611 from I2C
 */

#include "AP_Airspeed_two_MS5611.h"

#if AP_AIRSPEED_TWO_MS5611_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

namespace {
    constexpr const uint8_t MS5611_I2C_ADDR_1 = 0x76;
    constexpr const uint8_t MS5611_I2C_ADDR_2 = 0x77;

    constexpr const uint8_t REG_RESET    = 0x1E;
    constexpr const uint8_t REG_ADC_READ = 0x00;

/* PROM start address */
    constexpr const uint8_t REG_PROM_BASE = 0xA0;

/* write to one of these addresses to start pressure conversion */
    constexpr const uint8_t REG_CONVERT_D1_OSR_256  = 0x40;
    constexpr const uint8_t REG_CONVERT_D1_OSR_512  = 0x42;
    constexpr const uint8_t REG_CONVERT_D1_OSR_1024 = 0x44;
    constexpr const uint8_t REG_CONVERT_D1_OSR_2048 = 0x46;
    constexpr const uint8_t REG_CONVERT_D1_OSR_4096 = 0x48;

/* write to one of these addresses to start temperature conversion */
    constexpr const uint8_t REG_CONVERT_D2_OSR_256  = 0x50;
    constexpr const uint8_t REG_CONVERT_D2_OSR_512  = 0x52;
    constexpr const uint8_t REG_CONVERT_D2_OSR_1024 = 0x54;
    constexpr const uint8_t REG_CONVERT_D2_OSR_2048 = 0x56;
    constexpr const uint8_t REG_CONVERT_D2_OSR_4096 = 0x58;

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */
    constexpr const uint8_t REG_CONVERT_PRESSURE         = REG_CONVERT_D1_OSR_1024;
    constexpr const uint8_t ADDR_CMD_CONVERT_TEMPERATURE = REG_CONVERT_D2_OSR_1024;
} // end of nameless namespace for constants

uint16_t AP_Airspeed_two_MS5611::_read_prom_word(_dev &dev, uint8_t word) {
    const uint8_t reg = REG_PROM_BASE + (word << 1);
    uint8_t       val[2];
    if (!dev.ptr->transfer(&reg, 1, val, sizeof(val))) {
        return 0;
    }
    return (val[0] << 8) | val[1];
}

bool AP_Airspeed_two_MS5611::_read_prom(_dev &dev) {
    dev.ptr->transfer(&REG_RESET, 1, nullptr, 0);
    hal.scheduler->delay(4);

    bool all_zero = true;

    for (uint8_t i = 0; i < 8; ++i) {
        dev.prom[i] = _read_prom_word(dev, i);
        if (dev.prom[i] != 0) {
            all_zero = false;
        }
    }

    if (all_zero) {
        return false;
    }

    /* save the read crc */
    const uint16_t crc_read = dev.prom[7] & 0xf;

    /* remove CRC byte */
    dev.prom[7] &= 0xff00;

    return crc_read == crc_crc4(dev.prom);
}

bool AP_Airspeed_two_MS5611::_init(_dev &dev) {
    static int number = 1;
    WITH_SEMAPHORE(dev.ptr->get_semaphore());

    dev.ptr->set_retries(10);

    if (!_read_prom(dev)) {
        printf("MS5611_1: CRC mismatch");
        return false;
    }
    printf("MS5611_%d found on bus %u address 0x%02x\n", number, dev.ptr->bus_num(), dev.ptr->get_bus_address());
    number++;

    // Send a command to read temperature first
    WITH_SEMAPHORE(dev.ptr->get_semaphore());
    dev.ptr->transfer(&ADDR_CMD_CONVERT_TEMPERATURE, 1, nullptr, 0);

    dev.ptr->set_device_type(uint8_t(DevType::MS5611_1));
    set_bus_id(dev.ptr->get_bus_id());

    return true;
}

/*
  read from the ADC
 */
int32_t AP_Airspeed_two_MS5611::_read_adc(_dev &dev) {
    uint8_t val[3];
    if (!dev.ptr->read_registers(REG_ADC_READ, val, 3)) {
        return 0;
    }
    return (val[0] << 16) | (val[1] << 8) | val[2];
}

void AP_Airspeed_two_MS5611::_aux_calculate(_dev &dev) {

    float dT   = float(dev.D2) - float(int64_t(dev.prom[5]) * (1L << 8));
    float TEMP = 2000 + (dT * float(int64_t(dev.prom[6])) / (1L << 23));
    float OFF  = float(int64_t(dev.prom[2]) * (1L << 16)) + (float(int64_t(dev.prom[4])) * dT) / (1L << 7);
    float SENS = float(int64_t(dev.prom[1]) * (1L << 15)) + (float(int64_t(dev.prom[3])) * dT) / (1L << 8);

    //second order temperature compensation
    if (TEMP < 2000) {
        float T2    = sq(dT) / (1L << 23);
        float aux   = sq(TEMP - 2000.0);
        float OFF2  = 2.5f * aux;
        float SENS2 = 1.25f * aux;
        if (TEMP < -1500) {
            aux = sq(TEMP + 1500);
            OFF2 += 7 * aux;
            SENS2 += 11 * aux * 0.5;
        }
        TEMP        = TEMP - T2;
        OFF         = OFF - OFF2;
        SENS        = SENS - SENS2;
    }

    const float PSI_to_Pa = 6894.757f;

    float P      = (float(dev.D1) * SENS / (1L << 21) - OFF) / (1L << 15);
    float P_Pa   = PSI_to_Pa * 1.0e-4 * P;
    float Temp_C = TEMP * 0.01f;

    dev.pressure_sum += P_Pa;
    _temperature_sum += Temp_C;
}

void AP_Airspeed_two_MS5611::_calculate() {
    _aux_calculate(_dev1);
    _aux_calculate(_dev2);

    WITH_SEMAPHORE(sem);

    _press_count++;
    _temp_count++;
    _last_sample_time_ms = AP_HAL::millis();
}

void AP_Airspeed_two_MS5611::_timer() {
    if (AP_HAL::micros() - _command_send_us < 10000) {
        // we should avoid trying to read the ADC too soon after
        // sending the command
        return;
    }

    int32_t adc_val_1 = _read_adc(_dev1);
    int32_t adc_val_2 = _read_adc(_dev2);

    if (adc_val_1 == 0 or adc_val_2 == 0) {
        // we have either done a read too soon after sending the
        // request or we have tried to read the same sample twice. We
        // re-send the command now as we don't know what state the
        // sensor is in, and we do want to trigger it sending a value,
        // which we will discard
        if (_dev1.ptr->transfer(&_cmd_sent, 1, nullptr, 0) or _dev2.ptr->transfer(&_cmd_sent, 1, nullptr, 0)) {
            _command_send_us = AP_HAL::micros();
        }
        // when we get adc_val == 0 then both the current value and
        // the next value we read from the sensor are invalid
        _ignore_next = true;
        return;
    }

    /*
     * If read fails, re-initiate a read command for current state or we are
     * stuck
     */
    if (!_ignore_next) {
        if (_cmd_sent == ADDR_CMD_CONVERT_TEMPERATURE) {
            _dev1.D2 = adc_val_1;
            _dev2.D2 = adc_val_2;
        } else if (_cmd_sent == REG_CONVERT_PRESSURE) {
            _dev1.D1 = adc_val_1;
            _dev1.D1 = adc_val_2;
            _calculate();
        }
    }

    _ignore_next = false;

    _cmd_sent        = (_state == 0) ? ADDR_CMD_CONVERT_TEMPERATURE : REG_CONVERT_PRESSURE;
    if (!_dev1.ptr->transfer(&_cmd_sent, 1, nullptr, 0) or !_dev2.ptr->transfer(&_cmd_sent, 1, nullptr, 0)) {
        // we don't know for sure what state the sensor is in when we
        // fail to send the command, so ignore the next response
        _ignore_next = true;
        return;
    }
    _command_send_us = AP_HAL::micros();

    _state = (_state + 1) % 5;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_two_MS5611::get_differential_pressure(float &pressure) {
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_press_count > 0) {
        _pressure    = (_dev1.pressure_sum - _dev1.pressure_sum) / float(_press_count);
        _press_count = 0;
        _dev1.pressure_sum = _dev2.pressure_sum = 0;
    }
    pressure = _pressure;

    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_two_MS5611::get_temperature(float &temperature) {
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_temp_count > 0) {
        _temperature     = _temperature_sum / float(2 * _temp_count);
        _temp_count      = 0;
        _temperature_sum = 0;
    }

    temperature = _temperature;
    return true;
}

bool AP_Airspeed_two_MS5611::init() {
    // probe two sensor, supporting two possible I2C addresses
    _dev1.ptr = hal.i2c_mgr->get_device(get_bus(), MS5611_I2C_ADDR_1);
    _dev2.ptr = hal.i2c_mgr->get_device(get_bus(), MS5611_I2C_ADDR_2);

    if (!_dev1.ptr) {
        printf("MS5611_1 not found");
        return false;
    }
    if (!_dev2.ptr) {
        printf("MS5611_2 not found");
        return false;
    }

    if (!_init(_dev1) and !_init(_dev2)) {
        return false;
    };
    _dev1.ptr->register_periodic_callback(10 * AP_USEC_PER_MSEC,
                                          FUNCTOR_BIND_MEMBER(&AP_Airspeed_two_MS5611::_timer, void));

    _state           = 0;
    _command_send_us = AP_HAL::micros();
    return true;
}


#endif  // AP_AIRSPEED_TWO_MS5611_ENABLED