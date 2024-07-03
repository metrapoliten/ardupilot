#pragma once

/*
  backend driver for own airspeed sensor made from two MS5611 from I2C
 */

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_TWO_MS5611_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_two_MS5611 : public AP_Airspeed_Backend
{
public:
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    struct _dev
    {
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> ptr;
        int32_t D1;
        int32_t D2;
        uint16_t prom[8];
        float pressure_sum;
    };

    uint16_t _read_prom_word(_dev &dev, uint8_t word);
    bool _read_prom(_dev &dev);
    bool _init(_dev &dev);
    void _timer();
    int32_t _read_adc(_dev &dev);
    void _aux_calculate(_dev &dev);
    void _calculate();

    _dev _dev1, _dev2;

    uint8_t _state;

    uint32_t _command_send_us;

    float _pressure;
    float _temperature;
    float _temperature_sum;
    uint32_t _temp_count;
    uint32_t _press_count;

    uint32_t _last_sample_time_ms;

    bool _ignore_next;
    uint8_t _cmd_sent;
};

#endif  // AP_AIRSPEED_TWO_MS5611_ENABLED