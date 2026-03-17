#ifndef MPU6500_H
#define MPU6500_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"

/// @brief Class for interfacing with the MPU6500 module. Must call initialize() before using the sensor.
class MPU6500{
    public:
        /// @brief Create an instance using the i2c as the selected interface.
        /// @param sda SDA pin number.
        /// @param scl SCL pin number.
        /// @param i2c i2c instance(default is i2c0).
        MPU6500(uint sda, uint scl, i2c_inst_t* i2c=i2c0);

        /// @brief Create an instance using the spi as the selected interface.
        /// @param miso MISO pin number.
        /// @param mosi MOSI pin number.
        /// @param sck SCK pin number.
        /// @param cs CS pin number.
        /// @param spi spi instance(default is spi0).
        MPU6500(uint miso, uint mosi, uint sck, uint cs, spi_inst_t* spi=spi0);

        /// @brief Initialize the sensor for use.
        void initialize();

        void get_temperature(float &temperature);
        void get_acceleration(float acceleration[3]);
        void get_gyrosope(float gyrscope[3]);

        void get_temperature_raw(uint16_t temperature);
        void get_acceleration_raw(uint16_t acceleration[3]);
        void get_gyrosope_raw(uint16_t gyrscope[3]);

    private:
        bool is_i2c;
};

#endif //MPU6500_H