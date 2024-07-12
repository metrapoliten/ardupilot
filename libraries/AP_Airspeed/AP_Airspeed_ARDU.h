#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ARDU_ENABLED

#include "AP_Airspeed_Backend.h"
#include <AP_HAL/AP_HAL.h>

// Arduino calculates airspeed and temperature by itself
class AP_Airspeed_ARDU : public AP_Airspeed_Backend {
public:
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    // probe and initialise the sensor
    bool init() override;

    // reads airspeed directly from Arduino
    bool has_airspeed() override { return true; }

    // returns airspeed in m/s if available
    bool get_airspeed(float &airspeed) override;

    // returns the current temperature in degrees C
    bool get_temperature(float &temperature) override;

private:

    // pointer to serial uart
    AP_HAL::UARTDriver *_uart = nullptr;

    char    _term_buf[15];                  // buffer for the current term within the current sentence
    uint8_t _term_buf_len = 0;              // buffer length

    // store last sent speed and temp as update rate is slow
    float _last_speed;
    float _last_temp;

    float _speed_sum = 0;
    float _temp_sum  = 0;

    uint16_t _speed_count = 0;
    uint16_t _temp_count  = 0;

    uint32_t _last_update_ms;               // time last message was received
};

#endif  // AP_AIRSPEED_ARDU_ENABLED

