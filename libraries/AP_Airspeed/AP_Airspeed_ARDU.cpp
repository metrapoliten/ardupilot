#include "AP_Airspeed_ARDU.h"

#if AP_AIRSPEED_ARDU_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>

namespace {
    constexpr uint32_t TIMEOUT_MS = 2000;
}

bool AP_Airspeed_ARDU::init() {
    const AP_SerialManager &serial_manager = AP::serialmanager();

    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AirSpeed, 0);
    if (_uart == nullptr) {
        return false;
    }

    set_bus_id(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, 0, 0, 0));

    _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_AirSpeed, 0));

    set_use_zero_offset();

    return true;
}

bool AP_Airspeed_ARDU::get_airspeed(float &airspeed) {
    if (_uart == nullptr) {
        return false;
    }

    uint32_t now         = AP_HAL::millis();
    bool     parse_start = false;

    uint32_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        int16_t c = _uart->read();
        if (c == -1) {
            return false;
        }
        if (char(c) == '$') {
            parse_start = true;
        } else if (parse_start) {
            if (char(c) == ',') {
                _term_buf[_term_buf_len] = 0;
                _last_speed = strtof(_term_buf, nullptr);
                _speed_sum += _last_speed;
                _speed_count++;
                _term_buf_len = 0;
            } else if (char(c) == ';') {
                _term_buf[_term_buf_len] = 0;
                _last_temp = strtof(_term_buf, nullptr);
                _temp_sum += _last_temp;
                _temp_count++;
                _term_buf_len = 0;
                parse_start   = false;
            } else {
                _term_buf[_term_buf_len++] = char(c);
            }
        }
        _last_update_ms = now;
    }
    if (_speed_count == 0) {
        airspeed = _last_speed;
    } else {
        airspeed    = _speed_sum / _speed_count;
        _last_speed = airspeed;
    }
    return (now - _last_update_ms) < TIMEOUT_MS;
}

bool AP_Airspeed_ARDU::get_temperature(float &temperature) {
    if (_uart == nullptr) {
        return false;
    }

    if (_temp_count == 0) {
        temperature = _last_temp;
    } else {
        temperature = _temp_sum / _temp_count;
        _last_temp  = temperature;
        _temp_count = 0;
        _temp_sum   = 0;
    }
    return true;
}

#endif  // AP_AIRSPEED_ARDU_ENABLED
